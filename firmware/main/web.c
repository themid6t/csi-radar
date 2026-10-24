// Local web server: the radar page + SSE stream + commands, the setup page (local only: the tunnel in
// vorp.c never routes /setup or /scan), and in portal mode a DNS server that answers every name with
// our AP address so phones pop up the setup page.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include "app.h"

#define MAX_SSE 4

static const char *TAG = "web";
static httpd_handle_t server;
static struct { httpd_req_t *r; uint32_t seq; bool fresh; } sse[MAX_SSE];
static SemaphoreHandle_t sse_mx;

extern const char setup_html[] asm("_binary_setup_html_start");

static esp_err_t status(httpd_req_t *r, const char *s) {
    httpd_resp_set_status(r, s);
    return httpd_resp_send(r, NULL, 0);
}

static int body(httpd_req_t *r, char *buf, int cap) {
    int n = 0, want = r->content_len < (size_t)cap - 1 ? r->content_len : cap - 1;
    while (n < want) {
        int k = httpd_req_recv(r, buf + n, want - n);
        if (k <= 0) return -1;
        n += k;
    }
    buf[n] = 0;
    return n;
}

static esp_err_t index_get(httpd_req_t *r) {
    if (portal_mode) {
        httpd_resp_set_status(r, "302 Found");
        httpd_resp_set_hdr(r, "Location", "/setup");
        return httpd_resp_send(r, NULL, 0);
    }
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    httpd_resp_set_hdr(r, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(r, "Cache-Control", "no-cache");
    return httpd_resp_send(r, (const char *)index_gz, index_gz_end - index_gz);
}

static esp_err_t events_get(httpd_req_t *r) {
    xSemaphoreTake(sse_mx, portMAX_DELAY);
    int i = 0;
    while (i < MAX_SSE && sse[i].r) i++;
    esp_err_t e = i < MAX_SSE ? httpd_req_async_handler_begin(r, &sse[i].r) : ESP_FAIL;
    if (e == ESP_OK) sse[i].fresh = true;
    else if (i < MAX_SSE) sse[i].r = NULL;
    xSemaphoreGive(sse_mx);
    return e == ESP_OK ? ESP_OK : status(r, "503 Service Unavailable");
}

static void sse_task(void *arg) {
    int64_t last_ping = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(40));
        bool ping = esp_timer_get_time() - last_ping > 15000000;
        if (ping) last_ping = esp_timer_get_time();
        xSemaphoreTake(sse_mx, portMAX_DELAY);
        for (int i = 0; i < MAX_SSE; i++) {
            httpd_req_t *r = sse[i].r;
            if (!r) continue;
            esp_err_t err = ESP_OK;
            if (sse[i].fresh) {
                sse[i].fresh = false;
                sse[i].seq = bus_head();
                httpd_resp_set_type(r, "text/event-stream");
                httpd_resp_set_hdr(r, "Cache-Control", "no-cache");
                char *m = radar_init_msg();
                err = m ? httpd_resp_send_chunk(r, m, strlen(m)) : ESP_FAIL;
                free(m);
            }
            for (char *m; err == ESP_OK && (m = bus_next(&sse[i].seq)); free(m)) err = httpd_resp_send_chunk(r, m, strlen(m));
            if (err == ESP_OK && ping) err = httpd_resp_send_chunk(r, ": ping\n\n", 8);
            if (err != ESP_OK) {
                httpd_sess_trigger_close(server, httpd_req_to_sockfd(r));
                httpd_req_async_handler_complete(r);
                sse[i].r = NULL;
            }
        }
        xSemaphoreGive(sse_mx);
    }
}

static esp_err_t cmd_post(httpd_req_t *r) {
    char b[64];
    if (body(r, b, sizeof b) < 0) return status(r, "400 Bad Request");
    int s = radar_cmd(b);
    return status(r, s == 204 ? "204 No Content" : s == 409 ? "409 Conflict" : "400 Bad Request");
}

static esp_err_t config_post(httpd_req_t *r) {
    char b[512];
    if (body(r, b, sizeof b) < 0) return status(r, "400 Bad Request");
    return status(r, radar_config(b) == 204 ? "204 No Content" : "400 Bad Request");
}

static esp_err_t setup_get(httpd_req_t *r) {
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    return httpd_resp_sendstr(r, setup_html);
}

static esp_err_t setup_state_get(httpd_req_t *r) {
    char b[768];
    snprintf(b, sizeof b, "{\"wifi\":%s,\"tunnel\":%s,\"token\":%s,\"relay\":\"%s\",\"sub\":\"%s\"}",
             wifi_status(), vorp_status(), cfg.token[0] ? "true" : "false", cfg.relay, cfg.sub);
    httpd_resp_set_type(r, "application/json");
    return httpd_resp_sendstr(r, b);
}

static esp_err_t scan_get(httpd_req_t *r) {
    wifi_scan_config_t sc = { 0 };
    uint16_t n = 20;
    wifi_ap_record_t *ap = calloc(n, sizeof *ap);
    if (!ap || esp_wifi_scan_start(&sc, true) != ESP_OK || esp_wifi_scan_get_ap_records(&n, ap) != ESP_OK) n = 0;
    cJSON *a = cJSON_CreateArray();
    for (int i = 0; i < n; i++) {
        if (!ap[i].ssid[0]) continue;
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "ssid", (char *)ap[i].ssid);
        cJSON_AddNumberToObject(o, "rssi", ap[i].rssi);
        cJSON_AddBoolToObject(o, "open", ap[i].authmode == WIFI_AUTH_OPEN);
        cJSON_AddItemToArray(a, o);
    }
    free(ap);
    char *s = cJSON_PrintUnformatted(a);
    cJSON_Delete(a);
    httpd_resp_set_type(r, "application/json");
    esp_err_t e = httpd_resp_sendstr(r, s ? s : "[]");
    free(s);
    return e;
}

static void reboot(void *arg) { esp_restart(); }

static void copy(char *dst, size_t n, cJSON *o, const char *k) {  // set only when given and non-empty
    cJSON *v = cJSON_GetObjectItem(o, k);
    if (cJSON_IsString(v) && v->valuestring[0] && strlen(v->valuestring) < n) strcpy(dst, v->valuestring);
}

static esp_err_t setup_post(httpd_req_t *r) {
    char b[512];
    cJSON *o = body(r, b, sizeof b) < 0 ? NULL : cJSON_Parse(b);
    if (!o) return status(r, "400 Bad Request");
    cJSON *ssid = cJSON_GetObjectItem(o, "ssid");
    if (cJSON_IsString(ssid) && ssid->valuestring[0] && strlen(ssid->valuestring) < sizeof cfg.ssid) {
        strcpy(cfg.ssid, ssid->valuestring);
        cJSON *pass = cJSON_GetObjectItem(o, "pass");  // a new network replaces the password, even with an empty one
        if (cJSON_IsString(pass) && strlen(pass->valuestring) < sizeof cfg.pass) strcpy(cfg.pass, pass->valuestring);
    }
    copy(cfg.token, sizeof cfg.token, o, "token");
    copy(cfg.relay, sizeof cfg.relay, o, "relay");
    cJSON *sub = cJSON_GetObjectItem(o, "sub");
    if (cJSON_IsString(sub) && strlen(sub->valuestring) < sizeof cfg.sub) strcpy(cfg.sub, sub->valuestring);  // empty = random name
    if (cJSON_IsTrue(cJSON_GetObjectItem(o, "clear_token"))) cfg.token[0] = 0;
    cJSON_Delete(o);
    cfg_save();
    ESP_LOGI(TAG, "settings saved, restarting");
    static esp_timer_handle_t t;
    esp_timer_create_args_t a = { .callback = reboot };
    if (!t) esp_timer_create(&a, &t);
    esp_timer_start_once(t, 1000000);
    return status(r, "204 No Content");
}

static esp_err_t fallback(httpd_req_t *r) {
    if (!portal_mode) return status(r, "404 Not Found");
    httpd_resp_set_status(r, "302 Found");  // captive portal: every OS probe URL lands on setup
    httpd_resp_set_hdr(r, "Location", "http://192.168.4.1/setup");
    return httpd_resp_send(r, NULL, 0);
}

// portal mode: answer every DNS question with 192.168.4.1
static void dns_task(void *arg) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(53), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(s, (struct sockaddr *)&me, sizeof me);
    uint8_t p[512];
    for (;;) {
        struct sockaddr_in from;
        socklen_t fl = sizeof from;
        int n = recvfrom(s, p, sizeof p - 16, 0, (struct sockaddr *)&from, &fl);
        if (n < 12 || (p[2] & 0x80) || p[4] != 0 || p[5] != 1) continue;  // queries with one question only
        int i = 12;
        while (i < n && p[i]) i += p[i] + 1;
        if ((i += 5) > n) continue;  // zero label + qtype + qclass
        p[2] = 0x81; p[3] = 0x80;                                   // response, recursion available
        p[6] = 0; p[7] = 1; p[8] = p[9] = p[10] = p[11] = 0;        // 1 answer
        static const uint8_t ans[] = { 0xc0, 12, 0, 1, 0, 1, 0, 0, 0, 60, 0, 4, 192, 168, 4, 1 };
        memcpy(p + i, ans, sizeof ans);
        sendto(s, p, i + sizeof ans, 0, (struct sockaddr *)&from, fl);
    }
}

void web_start(void) {
    sse_mx = xSemaphoreCreateMutex();
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.max_open_sockets = 10;
    c.max_uri_handlers = 12;
    c.lru_purge_enable = true;
    c.stack_size = 6144;
    c.uri_match_fn = httpd_uri_match_wildcard;
    ESP_ERROR_CHECK(httpd_start(&server, &c));
#define R(u, m, h) { .uri = u, .method = m, .handler = h }
    const httpd_uri_t routes[] = {
        R("/", HTTP_GET, index_get), R("/events", HTTP_GET, events_get),
        R("/cmd", HTTP_POST, cmd_post), R("/config", HTTP_POST, config_post),
        R("/setup", HTTP_GET, setup_get), R("/setup", HTTP_POST, setup_post),
        R("/setup/state", HTTP_GET, setup_state_get), R("/scan", HTTP_GET, scan_get),
        R("/*", HTTP_GET, fallback),  // last: wildcard
    };
    for (int i = 0; i < sizeof routes / sizeof routes[0]; i++) httpd_register_uri_handler(server, &routes[i]);
    xTaskCreate(sse_task, "sse", 4096, NULL, 4, NULL);
}

void web_portal(void) { xTaskCreate(dns_task, "dns", 3072, NULL, 4, NULL); }
