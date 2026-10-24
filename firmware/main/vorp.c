// vorp agent (github.com/themid6t/vorp, docs/protocol.md): one outbound TLS connection to the relay with
// ALPN "vorp-agent/1", yamux inside. We open the control stream (RegisterAgent) and one tunnel stream
// (RegisterTunnel); the relay opens a stream per public HTTP request. Requests are answered here directly
// (same page, SSE and commands as web.c); the setup page is never reachable through the tunnel.
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif_sntp.h"
#include <sys/time.h>
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_tls.h"
#include "lwip/sockets.h"
#include "mbedtls/sha256.h"
#include "cJSON.h"
#include "app.h"

enum { Y_DATA, Y_WIN, Y_PING, Y_GOAWAY };
enum { F_SYN = 1, F_ACK = 2, F_FIN = 4, F_RST = 8 };
enum { K_FREE, K_CONTROL, K_TUNNEL, K_REQ };
#define INIT_WIN  (256 * 1024)  // yamux default receive window, both directions
#define MAX_ST    8
#define RXCAP     3072           // request stream message buffer; request heads from browsers are ~1-2 KB
#define RXCAP_CTL 512            // control and tunnel streams only get small JSON
#define CHUNK     2000           // body chunk; fits one TLS record (MBEDTLS_SSL_OUT_CONTENT_LEN)

static const char *TAG = "vorp";

typedef struct {
    uint32_t id, win;            // win = what we may still send
    int kind;
    uint8_t *rx; int rxn, cap;
    char method[8], path[64], body[512]; int bodyn;
    bool sse; uint32_t seq;
} st_t;

static esp_tls_t *tls;
static st_t S[MAX_ST];
static uint8_t txb[12 + 5 + CHUNK];
static double last_pong, last_ping, retry_tunnel;
static uint32_t next_id, sessions;
static bool permanent;           // relay said no for good (bad token, name not allowed): stop until reboot
static char state[16] = "off", url[96], err[96], status_buf[256];

static double now_s(void) { return esp_timer_get_time() / 1e6; }
static void be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }

static void set_state(const char *s, const char *e) {
    snprintf(state, sizeof state, "%s", s);
    snprintf(err, sizeof err, "%s", e ? e : "");
    if (e) ESP_LOGW(TAG, "%s: %s", s, e);
}

const char *vorp_status(void) {
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "state", cfg.token[0] ? state : "off");
    cJSON_AddStringToObject(o, "url", url);
    cJSON_AddStringToObject(o, "error", err);
    int n = 0;
    for (int i = 0; i < MAX_ST; i++) n += S[i].kind == K_REQ;
    cJSON_AddNumberToObject(o, "requests", n);
    cJSON_AddNumberToObject(o, "sessions", sessions);
    cJSON_PrintPreallocated(o, status_buf, sizeof status_buf, false);
    cJSON_Delete(o);
    return status_buf;
}

// ---------- wire ----------
static int tx(const uint8_t *b, int n) {
    while (n > 0) {
        int k = esp_tls_conn_write(tls, b, n);
        if (k == ESP_TLS_ERR_SSL_WANT_READ || k == ESP_TLS_ERR_SSL_WANT_WRITE) continue;
        if (k <= 0) { ESP_LOGW(TAG, "write failed: %d", k); return -1; }
        b += k; n -= k;
    }
    return 0;
}

static int rx(uint8_t *b, int n) {
    for (double until = now_s() + 15; n > 0;) {
        int k = esp_tls_conn_read(tls, b, n);
        if (k == ESP_TLS_ERR_SSL_WANT_READ || k == ESP_TLS_ERR_SSL_WANT_WRITE) { if (now_s() > until) { ESP_LOGW(TAG, "read timeout"); return -1; } continue; }
        if (k <= 0) { ESP_LOGW(TAG, "read failed: %d", k); return -1; }
        b += k; n -= k;
    }
    return 0;
}

static void yhdr(uint8_t *h, int type, int flags, uint32_t id, uint32_t len) {
    h[0] = 0; h[1] = type; h[2] = flags >> 8; h[3] = flags;
    be32(h + 4, id); be32(h + 8, len);
}

static int yframe(int type, int flags, uint32_t id, uint32_t len) {  // header-only frame (window, ping, fin, rst)
    uint8_t h[12];
    yhdr(h, type, flags, id, len);
    return tx(h, 12);
}

// one protocol message (type, BE u32 length, payload) as one yamux data frame. -2 = no send window
static int msg(st_t *s, int type, const void *p, int n, int flags) {
    if (n > CHUNK) return -1;
    if (s->win < (uint32_t)n + 5) return -2;
    s->win -= n + 5;
    yhdr(txb, Y_DATA, flags, s->id, n + 5);
    txb[12] = type;
    be32(txb + 13, n);
    memcpy(txb + 17, p, n);
    return tx(txb, 17 + n);
}

static int body(st_t *s, const void *p, long n) {  // BodyChunks
    for (long off = 0; off < n; off += CHUNK) {
        int e = msg(s, 0x30, (const uint8_t *)p + off, n - off < CHUNK ? n - off : CHUNK, 0);
        if (e) return e;
    }
    return 0;
}

static int msgf(st_t *s, int type, int flags, const char *fmt, ...) {
    char b[400];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    return n < (int)sizeof b ? msg(s, type, b, n, flags) : -1;
}

static st_t *find(uint32_t id) {
    for (int i = 0; i < MAX_ST; i++) if (S[i].kind && S[i].id == id) return &S[i];
    return NULL;
}

static st_t *alloc(uint32_t id, int kind) {
    for (int i = 0; i < MAX_ST; i++) {
        if (S[i].kind) continue;
        int cap = kind == K_REQ ? RXCAP : RXCAP_CTL;
        uint8_t *b = malloc(cap);
        if (!b) return NULL;
        S[i] = (st_t){ .id = id, .win = INIT_WIN, .kind = kind, .rx = b, .cap = cap };
        return &S[i];
    }
    return NULL;
}

static void drop(st_t *s, int flags) {  // flags: F_FIN after a finished response, F_RST to abort, 0 if the peer is gone
    if (flags) yframe(Y_WIN, flags, s->id, 0);
    free(s->rx);
    s->kind = K_FREE;
}

static void clear_all(void) {
    for (int i = 0; i < MAX_ST; i++) if (S[i].kind) drop(&S[i], 0);
}

static char *jstr(cJSON *o, const char *k) {
    cJSON *v = cJSON_GetObjectItem(o, k);
    return cJSON_IsString(v) ? v->valuestring : "";
}

static int open_tunnel(void) {  // after AgentAck, a recoverable close, or a taken name
    retry_tunnel = 0;
    st_t *t = alloc(next_id, K_TUNNEL);
    next_id += 2;
    if (!t) return -1;
    return cfg.sub[0] ? msgf(t, 0x10, F_SYN, "{\"subdomain\":\"%s\",\"upstream_hint\":\"esp32 csi-radar\"}", cfg.sub)
                      : msgf(t, 0x10, F_SYN, "{\"subdomain\":null,\"upstream_hint\":\"esp32 csi-radar\"}");
}

// ---------- requests ----------
static int head(st_t *s, int status, const char *ctype, const char *enc, long len) {
    char h[300], l[16] = "null";
    int n = snprintf(h, sizeof h, "{\"status\":%d,\"headers\":[[\"cache-control\",\"no-cache\"]", status);
    if (ctype) n += snprintf(h + n, sizeof h - n, ",[\"content-type\",\"%s\"]", ctype);
    if (enc) n += snprintf(h + n, sizeof h - n, ",[\"content-encoding\",\"%s\"]", enc);
    if (len >= 0) snprintf(l, sizeof l, "%ld", len);
    n += snprintf(h + n, sizeof h - n, "],\"content_length\":%s}", l);
    return msg(s, 0x21, h, n, 0);
}

static int finish(st_t *s) {  // BodyEnd + close our side
    int e = msg(s, 0x31, "{}", 2, 0);
    drop(s, F_FIN);
    return e == -1 ? -1 : 0;
}

static int respond(st_t *s) {
    ESP_LOGI(TAG, "stream %u: %s %s", (unsigned)s->id, s->method, s->path);
    if (!strcmp(s->method, "GET") && !strcmp(s->path, "/")) {
        long n = index_gz_end - index_gz;
        if (head(s, 200, "text/html; charset=utf-8", "gzip", n) == -1) return -1;
        int e = body(s, index_gz, n);
        if (e == -2) { drop(s, F_RST); return 0; }
        return e ? -1 : finish(s);
    }
    if (!strcmp(s->method, "GET") && !strcmp(s->path, "/events")) {
        if (head(s, 200, "text/event-stream", NULL, -1) == -1) return -1;
        s->seq = bus_head();
        s->sse = true;
        char *m = radar_init_msg();
        int e = m ? body(s, m, strlen(m)) : 0;
        free(m);
        return e == -1 ? -1 : 0;
    }
    int status = 404;
    if (!strcmp(s->method, "POST") && !strcmp(s->path, "/cmd")) status = radar_cmd(s->body);
    else if (!strcmp(s->method, "POST") && !strcmp(s->path, "/config")) status = radar_config(s->body);
    if (head(s, status, NULL, NULL, 0) == -1) return -1;
    return finish(s);
}

static int on_msg(st_t *s, int type, const uint8_t *p, int n) {
    if (s->kind == K_REQ && type == 0x30) {  // BodyChunk: raw bytes
        int k = n < (int)sizeof s->body - 1 - s->bodyn ? n : (int)sizeof s->body - 1 - s->bodyn;
        memcpy(s->body + s->bodyn, p, k);
        s->body[s->bodyn += k] = 0;
        return 0;
    }
    cJSON *o = cJSON_ParseWithLength((const char *)p, n);
    int ret = 0;
    if (s->kind == K_CONTROL && type == 0x02) {          // AgentAck
        ret = open_tunnel();
        set_state("registering", NULL);
    } else if (s->kind == K_CONTROL && type == 0x03) {   // AgentErr
        const char *code = jstr(o, "code");
        permanent = !strcmp(code, "AUTH_FAILED") || !strcmp(code, "UNSUPPORTED_VERSION");
        set_state("error", code);
        ret = -1;
    } else if (s->kind == K_CONTROL && type == 0x05) {   // Pong
        last_pong = now_s();
    } else if (s->kind == K_TUNNEL && type == 0x11) {    // TunnelAck
        snprintf(url, sizeof url, "%s", jstr(o, "url"));
        set_state("online", NULL);
        ESP_LOGI(TAG, "tunnel up: %s", url);
    } else if (s->kind == K_TUNNEL && type == 0x12) {    // TunnelErr
        const char *code = jstr(o, "code");
        set_state("error", code);
        if (!strcmp(code, "SUBDOMAIN_TAKEN")) retry_tunnel = now_s() + 60;  // e.g. the old agent still holds it
        else if (!strncmp(code, "SUBDOMAIN_", 10) || !strcmp(code, "TUNNEL_LIMIT")) permanent = true;
        else ret = -1;
        drop(s, 0);
    } else if (s->kind == K_TUNNEL && type == 0x13) {    // TunnelClose
        const char *reason = jstr(o, "reason");
        msgf(s, 0x14, 0, "{\"subdomain\":\"%s\"}", jstr(o, "subdomain"));
        set_state("closed", reason);
        url[0] = 0;
        if (!strcmp(reason, "RECOVERABLE")) retry_tunnel = now_s() + 1;
        else permanent = true;
        drop(s, 0);
    } else if (s->kind == K_REQ && type == 0x20) {       // RequestHead
        snprintf(s->method, sizeof s->method, "%s", jstr(o, "method"));
        const char *t = jstr(o, "target");
        int k = strcspn(t, "?");
        snprintf(s->path, sizeof s->path, "%.*s", k < 63 ? k : 63, t);
    } else if (s->kind == K_REQ && type == 0x31) {       // BodyEnd
        ret = respond(s);
    } else if (s->kind == K_REQ) {                        // BodyAbort, Error, anything else
        drop(s, F_RST);
    }
    cJSON_Delete(o);
    return ret;
}

static int feed(st_t *s, const uint8_t *p, int n) {
    if (s->rxn + n > s->cap) { drop(s, F_RST); return 0; }
    memcpy(s->rx + s->rxn, p, n);
    s->rxn += n;
    while (s->kind && s->rxn >= 5) {
        uint32_t len = rd32(s->rx + 1);
        if (len > (uint32_t)s->cap - 5) { drop(s, F_RST); return 0; }
        if (s->rxn < 5 + (int)len) break;
        uint32_t id = s->id;
        if (on_msg(s, s->rx[0], s->rx + 5, len) < 0) return -1;
        if (!s->kind || s->id != id) return 0;  // stream was dropped while handling
        memmove(s->rx, s->rx + 5 + len, s->rxn -= 5 + len);
    }
    return 0;
}

static int read_frame(void) {
    uint8_t h[12], buf[512];
    if (rx(h, 12)) return -1;
    if (h[0] != 0) { ESP_LOGW(TAG, "bad yamux version %d", h[0]); return -1; }
    int type = h[1], flags = h[2] << 8 | h[3];
    uint32_t id = rd32(h + 4), len = rd32(h + 8);
    if (type == Y_PING) return flags & F_SYN ? yframe(Y_PING, F_ACK, 0, len) : 0;
    if (type == Y_GOAWAY) { set_state("connecting", "relay going away"); return -1; }
    if (type > Y_GOAWAY) return -1;
    st_t *s = find(id);
    if ((flags & F_SYN) && !s && id % 2 == 0) {          // relay-opened request stream
        s = alloc(id, K_REQ);
        if (yframe(Y_WIN, s ? F_ACK : F_RST, id, 0)) return -1;
    }
    if (type == Y_WIN) { if (s) s->win += len; }
    else {
        for (uint32_t left = len; left;) {
            int k = left < sizeof buf ? left : sizeof buf;
            if (rx(buf, k)) return -1;
            left -= k;
            if (s && s->kind && s->id == id && feed(s, buf, k)) return -1;
        }
        if (len && (s = find(id)) && yframe(Y_WIN, 0, id, len)) return -1;  // hand the window straight back
    }
    if ((flags & (F_FIN | F_RST)) && (s = find(id)) && s->kind == K_REQ && (s->sse || (flags & F_RST))) drop(s, 0);
    return 0;
}

static int pump(void) {  // SSE out, heartbeat
    for (int i = 0; i < MAX_ST; i++) {
        st_t *s = &S[i];
        if (s->kind != K_REQ || !s->sse) continue;
        for (char *m; (m = bus_next(&s->seq)); free(m))
            if (body(s, m, strlen(m)) == -1) { free(m); return -1; }  // -2: relay is behind, drop this frame
    }
    double now = now_s();
    if (now - last_ping >= 20) {
        last_ping = now;
        st_t *c = find(1);
        struct timeval tv;
        gettimeofday(&tv, NULL);  // the relay drops pings older than 40 s by its wall clock: needs real time
        if (!c || msgf(c, 0x04, 0, "{\"timestamp_ms\":%lld}", tv.tv_sec * 1000LL + tv.tv_usec / 1000) == -1) return -1;
    }
    if (now - last_pong > 65) { set_state("connecting", "no pong from relay"); return -1; }
    if (retry_tunnel && now >= retry_tunnel) return open_tunnel();
    return 0;
}

static int session(void) {
    const char *alpn[] = { "vorp-agent/1", NULL };
    esp_tls_cfg_t c = { .alpn_protos = alpn, .crt_bundle_attach = esp_crt_bundle_attach, .timeout_ms = 15000 };
    tls = esp_tls_init();
    if (!tls) return -1;
    set_state("connecting", NULL);
    url[0] = 0;
    sessions++;
    int ret = -1;
    if (esp_tls_conn_new_sync(cfg.relay, strlen(cfg.relay), 443, &c, tls) != 1) { set_state("connecting", "TLS connect failed"); goto out; }

    uint8_t mac[6], dig[32];
    char mid[65], in[40];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    int n = snprintf(in, sizeof in, "csi-radar:" MACSTR, MAC2STR(mac));  // stable, so a reboot displaces our own stale session
    mbedtls_sha256((uint8_t *)in, n, dig, 0);
    for (int i = 0; i < 32; i++) sprintf(mid + 2 * i, "%02x", dig[i]);
    next_id = 1;
    st_t *ctl = alloc(next_id, K_CONTROL);
    next_id += 2;
    last_pong = last_ping = now_s();
    retry_tunnel = 0;
    if (!ctl || msgf(ctl, 0x01, F_SYN, "{\"protocol_version\":1,\"token\":\"%s\",\"machine_id\":\"%s\"}", cfg.token, mid)) goto out;

    int fd;
    esp_tls_get_conn_sockfd(tls, &fd);
    for (;;) {
        fd_set r;
        FD_ZERO(&r);
        FD_SET(fd, &r);
        struct timeval tv = { 0, 40000 };
        if (esp_tls_get_bytes_avail(tls) > 0 || select(fd + 1, &r, NULL, NULL, &tv) > 0) {
            if (read_frame()) { ESP_LOGW(TAG, "session ended reading (%s)", err); break; }
        }
        if (pump() || permanent) { ESP_LOGW(TAG, "session ended in pump (%s)", err); break; }
        if (!strcmp(state, "online")) ret = 0;  // the session worked; reconnect fast if it drops
    }
out:
    clear_all();
    esp_tls_conn_destroy(tls);
    tls = NULL;
    url[0] = 0;
    return ret;
}

static void vorp_task(void *arg) {
    static const int backoff[] = { 1, 2, 4, 8, 16, 30, 60 };
    int fails = 0;
    esp_sntp_config_t sc = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2, ESP_SNTP_SERVER_LIST("time.cloudflare.com", "pool.ntp.org"));
    esp_netif_sntp_init(&sc);
    while (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) set_state("connecting", "waiting for the clock (SNTP)");
    while (!permanent) {
        int was_live = session() == 0;
        if (permanent) break;
        int wait = was_live ? esp_random() % 5 : backoff[fails < 6 ? fails : 6] / 2 + esp_random() % (backoff[fails < 6 ? fails : 6] / 2 + 1);
        fails = was_live ? 0 : fails + 1;
        vTaskDelay(pdMS_TO_TICKS(1000 * wait + 100));
    }
    ESP_LOGE(TAG, "stopped: %s", err);
    vTaskDelete(NULL);
}

void vorp_start(void) {
    static bool started;
    if (started || !cfg.token[0] || !cfg.relay[0]) return;
    started = true;
    xTaskCreatePinnedToCore(vorp_task, "vorp", 8192, NULL, 4, NULL, 0);
}
