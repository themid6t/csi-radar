// CSI capture (ping the AP ~100x/s, read CSI off its replies), detection (dsp.c), and the event bus the
// web page streams from. Settings that persist (threshold, room, recording summaries) live in NVS.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include "app.h"
#include "dsp.h"

#define LED_GPIO   2
#define HOLD_S     1.0    // motion stays on this long after the statistic drops
#define CAL_S      10
#define REC_S      30
#define MAX_EVENTS 50
#define BUS_N      16

static const char *TAG = "radar";
static const char *LABELS[] = { "empty", "walking", "still" };

typedef struct { int8_t rssi; int8_t iq[2 * NSUB]; } sample_t;
typedef struct { uint16_t n; float p50, p99, p20; } recsum_t;
typedef struct {                      // persisted as one NVS blob
    float thr, room[3], ap[3], esp[3];
    recsum_t rec[3];
    uint8_t tuned, separable; float tuned_thr;
} saved_t;

static QueueHandle_t q;
static SemaphoreHandle_t mx;          // guards st, events, job, bus
static uint8_t ap_bssid[6];
static saved_t st = { .thr = 0.2f, .room = { 4, 4, 2.7f }, .ap = { 0.3f, 0.3f, 0.8f }, .esp = { 3.7f, 3.7f, 0.8f } };
static struct { double start, end; } events[MAX_EVENTS];
static int nevents;
static struct { int kind; int label; double until; float v[REC_S * 10 + 20]; int n; } job;  // kind 0 none, 1 cal, 2 rec
static int pps;
static bool unsupported;
static double link_t;
static dsp_t dsp;

// ---------- bus: last BUS_N SSE messages, consumers poll with their own sequence number ----------
static char *bus[BUS_N];
static uint32_t bus_seq;

static void post(const char *event, const char *json) {
    int n = strlen(event) + strlen(json) + sizeof "event: \ndata: \n\n";  // sizeof counts the NUL
    char *m = malloc(n);
    if (!m) return;
    snprintf(m, n, "event: %s\ndata: %s\n\n", event, json);
    xSemaphoreTake(mx, portMAX_DELAY);
    free(bus[bus_seq % BUS_N]);
    bus[bus_seq % BUS_N] = m;
    bus_seq++;
    xSemaphoreGive(mx);
}

uint32_t bus_head(void) { return bus_seq; }

char *bus_next(uint32_t *seq) {
    char *m = NULL;
    xSemaphoreTake(mx, portMAX_DELAY);
    if (bus_seq - *seq > BUS_N) *seq = bus_seq - BUS_N;  // fell behind: skip what's gone
    if (*seq != bus_seq) m = strdup(bus[(*seq)++ % BUS_N]);
    xSemaphoreGive(mx);
    return m;
}

static double now_s(void) { return esp_timer_get_time() / 1e6; }

// ---------- JSON ----------
static int meta_json(char *b, int n) {   // caller holds mx
    int k = snprintf(b, n, "{\"thr\":%.4f,\"config\":{\"room\":[%.2f,%.2f,%.2f],\"pi\":[%.2f,%.2f,%.2f],\"esp\":[%.2f,%.2f,%.2f]},\"tuned\":",
                     st.thr, st.room[0], st.room[1], st.room[2], st.ap[0], st.ap[1], st.ap[2], st.esp[0], st.esp[1], st.esp[2]);
    k += st.tuned ? snprintf(b + k, n - k, "{\"thr\":%.4f,\"separable\":%s}", st.tuned_thr, st.separable ? "true" : "false")
                  : snprintf(b + k, n - k, "null");
    k += snprintf(b + k, n - k, ",\"rec\":{");
    for (int i = 0, first = 1; i < 3; i++) {
        if (!st.rec[i].n) continue;
        k += snprintf(b + k, n - k, "%s\"%s\":{\"n\":%d,\"p50\":%.3f,\"p99\":%.3f}", first ? "" : ",", LABELS[i], st.rec[i].n, st.rec[i].p50, st.rec[i].p99);
        first = 0;
    }
    return k + snprintf(b + k, n - k, "}}");
}

static int events_json(char *b, int n) {  // caller holds mx
    int k = snprintf(b, n, "[");
    for (int i = 0; i < nevents; i++) {
        k += events[i].end ? snprintf(b + k, n - k, "%s{\"start\":%.1f,\"end\":%.1f}", i ? "," : "", events[i].start, events[i].end)
                           : snprintf(b + k, n - k, "%s{\"start\":%.1f,\"end\":null}", i ? "," : "", events[i].start);
    }
    return k + snprintf(b + k, n - k, "]");
}

static void post_meta(void) {
    char b[600];
    xSemaphoreTake(mx, portMAX_DELAY);
    meta_json(b, sizeof b);
    xSemaphoreGive(mx);
    post("meta", b);
}

static void post_events(void) {
    static char b[MAX_EVENTS * 40 + 8];
    xSemaphoreTake(mx, portMAX_DELAY);
    events_json(b, sizeof b);
    xSemaphoreGive(mx);
    post("events", b);
}

char *radar_init_msg(void) {
    int n = MAX_EVENTS * 40 + 900;
    char *b = malloc(n);
    if (!b) return NULL;
    xSemaphoreTake(mx, portMAX_DELAY);
    int k = snprintf(b, n, "event: init\ndata: {\"now\":%.1f,\"history\":[],\"online\":%s,\"unsupported\":%s,\"events\":",
                     now_s(), pps >= 20 ? "true" : "false", unsupported ? "true" : "false");
    k += events_json(b + k, n - k);
    k += snprintf(b + k, n - k, ",\"meta\":");
    k += meta_json(b + k, n - k);
    snprintf(b + k, n - k, "}\n\n");
    xSemaphoreGive(mx);
    return b;
}

static void save(void) {
    nvs_handle_t h;
    if (nvs_open("radar", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "st", &st, sizeof st);
        nvs_commit(h);
        nvs_close(h);
    }
}

// ---------- commands (the page is public through the tunnel: only these pass) ----------
static bool valid_thr(const char *s) {  // 0?\.\d{1,4}
    if (*s == '0') s++;
    if (*s++ != '.') return false;
    int d = 0;
    while (*s >= '0' && *s <= '9') s++, d++;
    return !*s && d >= 1 && d <= 4;
}

int radar_cmd(const char *s) {
    int status = 204;
    xSemaphoreTake(mx, portMAX_DELAY);
    if (!strcmp(s, "cal") || !strncmp(s, "rec ", 4)) {
        int label = -1;
        if (s[0] == 'r') for (int i = 0; i < 3; i++) if (!strcmp(s + 4, LABELS[i])) label = i;
        if (s[0] == 'r' && label < 0) status = 400;
        else if (job.kind) status = 409;
        else { job.kind = s[0] == 'c' ? 1 : 2; job.label = label; job.n = 0; job.until = now_s() + (s[0] == 'c' ? CAL_S : REC_S); }
    } else if (!strcmp(s, "apply")) {
        if (st.tuned) st.thr = st.tuned_thr; else status = 409;
    } else if (!strncmp(s, "thr ", 4) && valid_thr(s + 4)) {
        float v = strtof(s + 4, NULL);
        if (v >= 0.02f && v <= 0.95f) st.thr = v; else status = 400;
    } else status = 400;
    xSemaphoreGive(mx);
    if (status == 204 && s[0] != 'c' && s[0] != 'r') { save(); post_meta(); }
    return status;
}

static bool vec3(cJSON *o, const char *k, float out[3]) {
    cJSON *a = cJSON_GetObjectItem(o, k);
    if (!cJSON_IsArray(a) || cJSON_GetArraySize(a) != 3) return false;
    for (int i = 0; i < 3; i++) {
        cJSON *v = cJSON_GetArrayItem(a, i);
        if (!cJSON_IsNumber(v)) return false;
        out[i] = v->valuedouble;
    }
    return true;
}

int radar_config(const char *json) {
    cJSON *o = cJSON_Parse(json);
    float room[3], ap[3], esp[3];
    bool ok = o && vec3(o, "room", room) && vec3(o, "pi", ap) && vec3(o, "esp", esp);
    cJSON_Delete(o);
    for (int i = 0; ok && i < 3; i++)
        ok = room[i] >= 0.5f && room[i] <= 30 && ap[i] >= 0 && ap[i] <= room[i] && esp[i] >= 0 && esp[i] <= room[i];
    if (!ok) return 400;
    xSemaphoreTake(mx, portMAX_DELAY);
    memcpy(st.room, room, sizeof room); memcpy(st.ap, ap, sizeof ap); memcpy(st.esp, esp, sizeof esp);
    xSemaphoreGive(mx);
    save();
    post_meta();
    return 204;
}

// ---------- capture ----------
// Only the ping replies are evenly spaced; other frames from the AP (TCP for the page and tunnel) arrive in
// bursts. Encrypted payloads can't be inspected, but every ping reply has the same length, and it's the most
// common one: keep frames of the commonest length over the last 200, and skip AP retransmissions.
static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    static uint16_t counts[256], seen, ping_len, recent[8] = { 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff }, nrecent;
    if (info->len < 128 || memcmp(info->mac, ap_bssid, 6)) return;
    uint8_t l = info->payload_len;
    counts[l]++;
    if (++seen == 200) {
        for (int i = 0; i < 256; i++) if (counts[i] > counts[ping_len]) ping_len = i;
        memset(counts, 0, sizeof counts);
        seen = 0;
    }
    if (ping_len && l != ping_len) return;
    uint16_t seq = info->hdr ? (info->hdr[22] | info->hdr[23] << 8) >> 4 : 0;  // 802.11 sequence number
    for (int i = 0; info->hdr && i < 8; i++) if (recent[i] == seq) return;  // AP retransmission of a frame we already have
    recent[nrecent++ % 8] = seq;
    sample_t s = { .rssi = info->rx_ctrl.rssi };
    // buf = (imag, real) int8 pairs; index k<32 is subcarrier k, k>=32 is k-64.
    // Keep -26..-1 (k=38..63) then 1..26 (k=1..26); skip DC and guard bands.
    memcpy(s.iq, info->buf + 2 * 38, 2 * 26);
    memcpy(s.iq + 2 * 26, info->buf + 2 * 1, 2 * 26);
    xQueueSend(q, &s, 0);  // drop if detection is behind
}

// One ICMP echo to the AP every 10 ms; every reply carries CSI. Not esp_ping: it schedules with
// vTaskDelayUntil, so after any stall it fires 200+/s to catch up, and the CSI arrives in bursts.
static void ping_task(void *arg) {
    int s = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    struct sockaddr_in to = { .sin_family = AF_INET, .sin_addr.s_addr = (uint32_t)arg };
    uint8_t p[9] = { 8, 0, 0, 0, 0x43, 0x53 }, junk[64];  // echo request, id "CS", 1 byte of data
    for (uint16_t seq = 0;; seq++) {
        p[2] = p[3] = 0;
        p[6] = seq >> 8; p[7] = seq;
        uint32_t sum = 0;
        for (int i = 0; i < 8; i += 2) sum += p[i] << 8 | p[i + 1];
        sum += p[8] << 8;
        while (sum >> 16) sum = (sum & 0xffff) + (sum >> 16);
        p[2] = ~sum >> 8; p[3] = ~sum;
        sendto(s, p, sizeof p, 0, (struct sockaddr *)&to, sizeof to);  // a failed send just skips one sample
        while (recv(s, junk, sizeof junk, MSG_DONTWAIT) > 0) {}        // replies: only their CSI matters
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void radar_link_up(const uint8_t bssid[6], uint32_t gw) {
    memcpy(ap_bssid, bssid, 6);
    link_t = now_s();
    static bool started;
    if (started) return;  // the pinger keeps the gateway it started with; it survives reconnects
    started = true;
    wifi_csi_config_t csi = { .lltf_en = true, .htltf_en = false, .stbc_htltf2_en = false,
                              .ltf_merge_en = true, .channel_filter_en = false, .manu_scale = false };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    xTaskCreatePinnedToCore(ping_task, "ping", 2560, (void *)gw, 6, NULL, 1);
}

// ---------- detection loop ----------
static void finish_job(void) {   // caller holds mx
    int n = job.n;
    float *v = job.v;
    for (int i = 1; i < n; i++) for (int j = i; j && v[j - 1] > v[j]; j--) { float t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
    float p99 = n ? v[n * 99 / 100] : 0;
    if (job.kind == 1) {
        // ponytail: assumes the room is empty during calibration; the record/tune flow is the precise way
        st.thr = fminf(0.6f, fmaxf(0.12f, p99 * 1.5f));
    } else if (n) {
        st.rec[job.label] = (recsum_t){ n, v[n / 2], p99, v[n / 5] };
        if (st.rec[0].n && st.rec[1].n) {
            int sep;
            st.tuned_thr = dsp_tune(st.rec[0].p99, st.rec[1].p20, &sep);
            st.tuned = 1; st.separable = sep;
        }
    }
    job.kind = 0;
}

static void radar_task(void *arg) {
    sample_t s;
    float amp[NSUB] = { 0 };
    int namp = 0, cnt = 0, streak = 0, rssi = 0;
    bool motion = false;
    double hold_until = 0, last_motion = -1e9, t_sec = now_s(), t_status = 0;
    float bpm = 0, prev_b = 0;
    char *f = malloc(1024);
    for (;;) {
        bool got = xQueueReceive(q, &s, pdMS_TO_TICKS(100));
        double now = now_s();
        if (got) {
            float na[NSUB];
            dsp_amplitudes(s.iq, na);
            dsp_push(&dsp, na);
            for (int i = 0; i < NSUB; i++) amp[i] += hypotf(s.iq[2 * i], s.iq[2 * i + 1]);
            namp++; cnt++; rssi = s.rssi;
        }
        if (now - t_sec >= 1) {
            pps = lround(cnt / (now - t_sec));
            cnt = 0; t_sec = now;
            dsp_second(&dsp, pps > 50 && pps < 130 ? pps : 100);
            float b = now - last_motion < 30 ? 0 : dsp.bpm;  // gross motion swamps breathing for the whole window
            // a single prominent peak can be noise; only report a rate that holds (+-2/min) for 5 checks in a row
            streak = b && prev_b && fabsf(b - prev_b) <= 2 ? streak + 1 : b ? 1 : 0;
            prev_b = b;
            bpm = streak >= 5 ? b : 0;
            unsupported = link_t && now - link_t > 15 && pps < 20;
        }
        if (now - t_status >= 2) {
            t_status = now;
            snprintf(f, 1024, "{\"online\":%s,\"unsupported\":%s,\"heap\":%u,\"block\":%u}", pps >= 20 ? "true" : "false",
                     unsupported ? "true" : "false", (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            post("status", f);
        }
        if (!got || dsp.n % 10) continue;

        float ms = dsp_motion(&dsp);
        xSemaphoreTake(mx, portMAX_DELAY);
        float thr = st.thr;
        if (ms > thr) { hold_until = now + HOLD_S; last_motion = now; }
        bool m = now < hold_until, changed = m != motion;
        if (changed) {
            motion = m;
            gpio_set_level(LED_GPIO, m);
            if (m) {
                if (nevents == MAX_EVENTS) memmove(events, events + 1, --nevents * sizeof events[0]);
                events[nevents++] = (typeof(events[0])){ now, 0 };
            } else if (nevents && !events[nevents - 1].end) events[nevents - 1].end = now;
        }
        char jb[80] = "null";
        bool done = false;
        if (job.kind) {
            if (job.n < (int)(sizeof job.v / sizeof job.v[0])) job.v[job.n++] = ms;
            int total = job.kind == 1 ? CAL_S : REC_S;
            int pct = lround(100 * (1 - (job.until - now) / total));
            snprintf(jb, sizeof jb, "{\"kind\":\"%s\",\"label\":%s%s%s,\"pct\":%d}", job.kind == 1 ? "cal" : "rec",
                     job.kind == 2 ? "\"" : "", job.kind == 2 ? LABELS[job.label] : "null", job.kind == 2 ? "\"" : "", pct > 100 ? 100 : pct);
            if (now >= job.until) { finish_job(); done = true; }
        }
        xSemaphoreGive(mx);
        if (done) { save(); post_meta(); }
        if (changed) post_events();

        int k = snprintf(f, 1024, "{\"t\":%.1f,\"ms\":%.4f,\"thr\":%.4f,\"motion\":%d,\"rssi\":%d,\"pps\":%d,\"bpm\":",
                         now, ms, thr, motion, rssi, pps);
        k += bpm ? snprintf(f + k, 1024 - k, "%.1f", bpm) : snprintf(f + k, 1024 - k, "null");
        k += snprintf(f + k, 1024 - k, ",\"tones\":[");
        for (int i = 0, n = 0; i < dsp.ntones; i++) {
            // multiples of 10 Hz are our own page updates (sent every 10 packets) disturbing reception; they're
            // still notched out of the motion statistic, just not reported as room interference
            float r = fmodf(dsp.tone_f[i] + 0.3f, 10);
            if (dsp.tone_f[i] > 5 && r < 0.6f) continue;
            k += snprintf(f + k, 1024 - k, "%s[%.2f,%.0f]", n++ ? "," : "", dsp.tone_f[i], dsp.tone_p[i]);
        }
        k += snprintf(f + k, 1024 - k, "],\"job\":%s,\"amp\":[", jb);
        for (int i = 0; i < NSUB; i++) k += snprintf(f + k, 1024 - k, "%s%d", i ? "," : "", (int)lroundf(amp[i] / namp));
        snprintf(f + k, 1024 - k, "]}");
        memset(amp, 0, sizeof amp);
        namp = 0;
        post("frame", f);
    }
}

void radar_start(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    mx = xSemaphoreCreateMutex();
    nvs_handle_t h;
    if (nvs_open("radar", NVS_READONLY, &h) == ESP_OK) {
        saved_t s;
        size_t n = sizeof s;
        if (nvs_get_blob(h, "st", &s, &n) == ESP_OK && n == sizeof s) st = s;
        nvs_close(h);
    }
    dsp_init(&dsp);
    q = xQueueCreate(64, sizeof(sample_t));
    xTaskCreatePinnedToCore(radar_task, "radar", 8192, NULL, 5, NULL, 1);  // core 1: Wi-Fi runs on core 0
    ESP_LOGI(TAG, "threshold %.3f", st.thr);
}
