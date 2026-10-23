// Wi-Fi CSI capture: ping the Pi's AP at 100 Hz, read CSI off its replies, stream raw I/Q to the Pi.
// Detection runs on the Pi (easier to iterate); it drives the LED back over UDP.
#include <string.h>
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
#include "secrets.h"  // WIFI_SSID, WIFI_PASS, PI_HOST, ESP_IP (gitignored)

#define PI_PORT   5005
#define CMD_PORT  5006
#define LED_GPIO  2
#define NSUB      52    // LLTF subcarriers -26..-1, 1..26
#define BATCH     10    // packets per UDP datagram (~10 datagrams/s)

static const char *TAG = "radar";

// wire format (little-endian): "CS", version=2, count, then count x { u32 ms, i8 rssi, i8 iq[2*NSUB] (imag, real) }
typedef struct __attribute__((packed)) { uint32_t ms; int8_t rssi; int8_t iq[2 * NSUB]; } sample_t;

static QueueHandle_t q;
static uint8_t ap_bssid[6];

static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    if (info->len < 128 || memcmp(info->mac, ap_bssid, 6)) return;
    sample_t s = { .ms = esp_timer_get_time() / 1000, .rssi = info->rx_ctrl.rssi };
    // buf = (imag, real) int8 pairs; index k<32 is subcarrier k, k>=32 is k-64.
    // Keep -26..-1 (k=38..63) then 1..26 (k=1..26); skip DC and guard bands.
    memcpy(s.iq, info->buf + 2 * 38, 2 * 26);
    memcpy(s.iq + 2 * 26, info->buf + 2 * 1, 2 * 26);
    xQueueSend(q, &s, 0);  // drop if the sender is behind
}

static void send_task(void *arg) {
    int tx = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in pi = { .sin_family = AF_INET, .sin_port = htons(PI_PORT) };
    inet_pton(AF_INET, PI_HOST, &pi.sin_addr);
    static uint8_t pkt[4 + BATCH * sizeof(sample_t)] = { 'C', 'S', 2 };
    int n = 0;
    for (;;) {
        if (!xQueueReceive(q, pkt + 4 + n * sizeof(sample_t), portMAX_DELAY)) continue;
        if (++n < BATCH) continue;
        pkt[3] = n;
        sendto(tx, pkt, 4 + n * sizeof(sample_t), 0, (struct sockaddr *)&pi, sizeof pi);
        n = 0;
    }
}

// "led 0" / "led 1"
static void cmd_task(void *arg) {
    int rx = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in me = { .sin_family = AF_INET, .sin_port = htons(CMD_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(rx, (struct sockaddr *)&me, sizeof me);
    char buf[16];
    for (;;) {
        int n = recv(rx, buf, sizeof buf - 1, 0);
        if (n <= 0) continue;
        buf[n] = 0;
        if (!strncmp(buf, "led ", 4)) gpio_set_level(LED_GPIO, buf[4] == '1');
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
    cfg.timeout_ms = 15;  // esp_ping waits for each reply; a long timeout stalls CSI for that long on every lost packet
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

    xTaskCreate(send_task, "send", 4096, NULL, 5, NULL);
    xTaskCreate(cmd_task, "cmd", 3072, NULL, 4, NULL);
}
