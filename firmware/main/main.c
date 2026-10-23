// Wi-Fi CSI motion radar: ping the Pi's AP at 100 Hz, read CSI off its replies,
// detect motion on-chip (moving variance of spatial turbulence), stream to the Pi over UDP.
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "ping/ping_sock.h"
#include "secrets.h"  // WIFI_SSID, WIFI_PASS, PI_HOST (gitignored)

#define PI_PORT   5005
#define CMD_PORT  5006
#define LED_GPIO  2
#define NSUB      52    // LLTF subcarriers -26..-1, 1..26
#define WIN       50    // moving-variance window, packets (~0.5 s)
#define CAL_N     1000  // calibration samples (~10 s, room should be still)
#define SEND_EVERY 10   // ~10 frames/s to the Pi
#define HOLD      50    // keep "motion" on this many packets after last trigger

static const char *TAG = "radar";

typedef struct { uint8_t amp[NSUB]; int8_t rssi; } sample_t;

static QueueHandle_t q;
static uint8_t ap_bssid[6];
static volatile bool cal_req = true;
static volatile float thr = 0;  // 0 = calibrating

static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    if (info->len < 128 || memcmp(info->mac, ap_bssid, 6)) return;
    sample_t s = { .rssi = info->rx_ctrl.rssi };
    // buf = (imag, real) int8 pairs; index k<32 is subcarrier k, k>=32 is k-64.
    // Keep -26..-1 (k=38..63) then 1..26 (k=1..26); skip DC and guard bands.
    int n = 0;
    for (int k = 38; k < 64; k++, n++) s.amp[n] = (uint8_t)hypotf(info->buf[2*k], info->buf[2*k+1]);
    for (int k = 1; k <= 26; k++, n++)  s.amp[n] = (uint8_t)hypotf(info->buf[2*k], info->buf[2*k+1]);
    xQueueSend(q, &s, 0);  // drop if the processor is behind
}

static int cmp_float(const void *a, const void *b) {
    float d = *(const float *)a - *(const float *)b;
    return (d > 0) - (d < 0);
}

static void process_task(void *arg) {
    int tx = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in pi = { .sin_family = AF_INET, .sin_port = htons(PI_PORT) };
    inet_pton(AF_INET, PI_HOST, &pi.sin_addr);

    static float turb[WIN], cal[CAL_N];
    static uint32_t acc[NSUB];
    static char json[700];
    int ti = 0, tn = 0, cn = 0, hold = 0, nacc = 0, pps = 0, pcount = 0;
    int64_t pps_t = esp_timer_get_time();
    sample_t s;

    for (;;) {
        if (!xQueueReceive(q, &s, portMAX_DELAY)) continue;
        if (cal_req) { cal_req = false; thr = 0; cn = 0; }

        // spatial turbulence: coefficient of variation across subcarriers (gain-invariant)
        float sum = 0, sq = 0;
        for (int i = 0; i < NSUB; i++) { sum += s.amp[i]; sq += (float)s.amp[i] * s.amp[i]; acc[i] += s.amp[i]; }
        float mean = sum / NSUB;
        float t = mean > 0 ? sqrtf(fmaxf(sq / NSUB - mean * mean, 0)) / mean : 0;
        turb[ti] = t; ti = (ti + 1) % WIN; if (tn < WIN) tn++;
        nacc++;

        float m = 0, v = 0;
        for (int i = 0; i < tn; i++) m += turb[i];
        m /= tn;
        for (int i = 0; i < tn; i++) v += (turb[i] - m) * (turb[i] - m);
        v /= tn;

        if (thr == 0 && tn == WIN) {
            cal[cn++] = v;
            if (cn == CAL_N) {
                // ponytail: p99 of a still room x2; the UI slider overrides it if your room needs more/less
                qsort(cal, CAL_N, sizeof(float), cmp_float);
                thr = fmaxf(cal[CAL_N * 99 / 100] * 2.0f, 1e-7f);
                ESP_LOGI(TAG, "calibrated thr=%g", thr);
            }
        }
        if (thr > 0 && v > thr) hold = HOLD;
        bool motion = hold > 0;
        if (hold) hold--;
        gpio_set_level(LED_GPIO, motion);

        pcount++;
        int64_t now = esp_timer_get_time();
        if (now - pps_t >= 1000000) { pps = pcount; pcount = 0; pps_t = now; }

        if (nacc >= SEND_EVERY) {
            int n = snprintf(json, sizeof json,
                "{\"ms\":%lld,\"var\":%.4g,\"thr\":%.4g,\"motion\":%d,\"cal\":%d,\"rssi\":%d,\"pps\":%d,\"amp\":[",
                now / 1000, v, thr, motion, thr == 0 ? cn * 100 / CAL_N : 100, s.rssi, pps);
            for (int i = 0; i < NSUB; i++) {
                n += snprintf(json + n, sizeof json - n, i ? ",%lu" : "%lu", (unsigned long)(acc[i] / nacc));
                acc[i] = 0;
            }
            n += snprintf(json + n, sizeof json - n, "]}");
            nacc = 0;
            sendto(tx, json, n, 0, (struct sockaddr *)&pi, sizeof pi);
        }
    }
}

// "cal" recalibrates, "thr <float>" sets the threshold.
static void cmd_task(void *arg) {
    int rx = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(CMD_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(rx, (struct sockaddr *)&me, sizeof me);
    char buf[64];
    for (;;) {
        int n = recv(rx, buf, sizeof buf - 1, 0);
        if (n <= 0) continue;
        buf[n] = 0;
        if (!strncmp(buf, "cal", 3)) cal_req = true;
        else if (!strncmp(buf, "thr ", 4)) { float f = strtof(buf + 4, NULL); if (f > 0) thr = f; }
        ESP_LOGI(TAG, "cmd: %s", buf);
    }
}

static void start_ping(esp_ip4_addr_t gw) {
    static esp_ping_handle_t ping;
    if (ping) return;
    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.target_addr.u_addr.ip4.addr = gw.addr;
    cfg.target_addr.type = IPADDR_TYPE_V4;
    cfg.count = ESP_PING_COUNT_INFINITE;
    cfg.interval_ms = 10;
    cfg.timeout_ms = 1000;
    cfg.data_size = 1;
    esp_ping_callbacks_t cbs = {0};
    ESP_ERROR_CHECK(esp_ping_new_session(&cfg, &cbs, &ping));
    esp_ping_start(ping);
}

static void start_csi(void) {
    wifi_csi_config_t csi = { .lltf_en = true, .htltf_en = false, .stbc_htltf2_en = false,
                              .ltf_merge_en = true, .channel_filter_en = false, .manu_scale = false };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) { ESP_LOGW(TAG, "wifi lost, retrying"); esp_wifi_connect(); }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) memcpy(ap_bssid, ((wifi_event_sta_connected_t *)data)->bssid, 6);
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "ip " IPSTR " gw " IPSTR, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
        start_csi();
        start_ping(e->ip_info.gw);
    }
}

void app_main(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    // static IP: the Pi AP has no DHCP server
    esp_netif_dhcpc_stop(sta);
    esp_netif_ip_info_t ip = {0};
    ip.ip.addr = ipaddr_addr(ESP_IP);
    ip.gw.addr = ipaddr_addr(PI_HOST);
    ip.netmask.addr = ipaddr_addr("255.255.255.0");
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta, &ip));
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL);

    wifi_config_t wc = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);  // power save would drop most CSI

    q = xQueueCreate(64, sizeof(sample_t));

    xTaskCreate(process_task, "process", 6144, NULL, 5, NULL);
    xTaskCreate(cmd_task, "cmd", 3072, NULL, 4, NULL);
}
