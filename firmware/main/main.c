// CSI radar, all on the ESP32: joins the saved Wi-Fi, senses motion from the AP's ping replies (radar.c),
// serves the page locally (web.c) and through a vorp tunnel when a token is set (vorp.c).
// No saved Wi-Fi, or it fails 5+ times and for 45 s before ever connecting: open AP "csi-radar-setup" + captive setup page
// (the saved network is still retried every 2 min).
// Hold BOOT for 5 s to forget the Wi-Fi settings.
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "mdns.h"
#include "app.h"

#define BOOT_GPIO   0
#define LED_GPIO    2
#define SETUP_SSID  "csi-radar-setup"
#define MAX_FAILS   5
#define PORTAL_AFTER_US 45000000LL  // and at least 45 s without a connection

static const char *TAG = "main";
cfg_t cfg;  // all set on the setup page
bool portal_mode;
static esp_netif_t *sta;
static uint8_t bssid[6];
static bool ever_ip, have_ip;
static int fails;
static char status_buf[256];

static void load(nvs_handle_t h, const char *k, char *dst, size_t n) {
    size_t len = n;
    if (nvs_get_str(h, k, dst, &len) != ESP_OK) return;  // keep the default
}

static void cfg_load(void) {
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READONLY, &h) != ESP_OK) return;
    load(h, "ssid", cfg.ssid, sizeof cfg.ssid);
    load(h, "pass", cfg.pass, sizeof cfg.pass);
    load(h, "token", cfg.token, sizeof cfg.token);
    load(h, "relay", cfg.relay, sizeof cfg.relay);
    load(h, "sub", cfg.sub, sizeof cfg.sub);
    nvs_close(h);
}

void cfg_save(void) {
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "ssid", cfg.ssid);
    nvs_set_str(h, "pass", cfg.pass);
    nvs_set_str(h, "token", cfg.token);
    nvs_set_str(h, "relay", cfg.relay);
    nvs_set_str(h, "sub", cfg.sub);
    nvs_commit(h);
    nvs_close(h);
}

const char *wifi_status(void) {
    esp_netif_ip_info_t ip = { 0 };
    wifi_ap_record_t ap = { 0 };
    if (have_ip) { esp_netif_get_ip_info(sta, &ip); esp_wifi_sta_get_ap_info(&ap); }
    snprintf(status_buf, sizeof status_buf, "{\"portal\":%s,\"ssid\":\"%s\",\"connected\":%s,\"ip\":\"" IPSTR "\",\"rssi\":%d,"
             "\"uptime\":%lld,\"reset\":%d}", portal_mode ? "true" : "false", cfg.ssid, have_ip ? "true" : "false", IP2STR(&ip.ip),
             ap.rssi, esp_timer_get_time() / 1000000, esp_reset_reason());  // reset: esp_reset_reason_t, 1 = power-on, 4 = panic
    return status_buf;
}

static void start_portal(void) {
    if (portal_mode) return;
    portal_mode = true;
    ESP_LOGW(TAG, "no usable Wi-Fi, starting setup AP \"%s\"", SETUP_SSID);
    esp_wifi_disconnect();
    wifi_config_t ap = { .ap = { .ssid = SETUP_SSID, .ssid_len = strlen(SETUP_SSID), .channel = 1,
                                 .authmode = WIFI_AUTH_OPEN, .max_connection = 4 } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));  // STA side stays idle, for scanning
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    web_portal();
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (!portal_mode) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        memcpy(bssid, ((wifi_event_sta_connected_t *)data)->bssid, 6);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        have_ip = false;
        if (portal_mode) return;
        // once it has worked, keep retrying forever (the AP may just be rebooting)
        int reason = ((wifi_event_sta_disconnected_t *)data)->reason;
        if (!ever_ip && ++fails >= MAX_FAILS && esp_timer_get_time() > PORTAL_AFTER_US) start_portal();
        else { ESP_LOGW(TAG, "wifi lost (reason %d), retrying", reason); esp_wifi_connect(); }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        ESP_LOGI(TAG, "ip " IPSTR " gw " IPSTR, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
        ever_ip = have_ip = true;
        fails = 0;
        if (portal_mode) {  // the saved network came back while we were waiting in setup
            portal_mode = false;
            esp_wifi_set_mode(WIFI_MODE_STA);
            gpio_set_level(LED_GPIO, 0);
        }
        radar_link_up(bssid, e->ip_info.gw.addr);
        vorp_start();
    }
}

static void button_task(void *arg) {
    int held = 0;
    for (int tick = 0;; tick++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (portal_mode) gpio_set_level(LED_GPIO, tick / 5 % 2);  // slow blink: waiting for setup
        if (portal_mode && cfg.ssid[0] && tick % 1200 == 600) esp_wifi_connect();  // retry the saved network every 2 min
        held = gpio_get_level(BOOT_GPIO) ? 0 : held + 1;
        if (held == 50) {
            ESP_LOGW(TAG, "BOOT held 5 s: forgetting Wi-Fi");
            cfg.ssid[0] = cfg.pass[0] = 0;
            cfg_save();
            esp_restart();
        }
    }
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    cfg_load();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(sta, "csi-radar");
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL);

    radar_start();  // also sets up the LED
    gpio_reset_pin(BOOT_GPIO);
    gpio_set_direction(BOOT_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BOOT_GPIO);

    wifi_config_t wc = { 0 };
    strcpy((char *)wc.sta.ssid, cfg.ssid);
    strcpy((char *)wc.sta.password, cfg.pass);
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));  // our own NVS keys are the source of truth
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    if (!cfg.ssid[0]) start_portal();
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);  // power save would drop most CSI

    web_start();
    mdns_init();
    mdns_hostname_set("csi-radar");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    xTaskCreate(button_task, "button", 2560, NULL, 2, NULL);
}
