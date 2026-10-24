#pragma once
#include <stdbool.h>
#include <stdint.h>

// main.c: settings saved in NVS, entered on the setup page
typedef struct { char ssid[33], pass[65], token[128], relay[64], sub[64]; } cfg_t;
extern cfg_t cfg;
extern bool portal_mode;           // no usable Wi-Fi: own AP "csi-radar-setup" + captive portal
void cfg_save(void);
const char *wifi_status(void);     // JSON object for the setup page

// radar.c
void radar_start(void);
void radar_link_up(const uint8_t bssid[6], uint32_t gw);
int  radar_cmd(const char *s);     // -> HTTP status
int  radar_config(const char *json);
char *radar_init_msg(void);        // malloc'd SSE "init" message
uint32_t bus_head(void);
char *bus_next(uint32_t *seq);     // malloc'd SSE message after *seq, or NULL

// web.c
void web_start(void);
void web_portal(void);           // captive DNS
extern const uint8_t index_gz[] asm("_binary_index_html_gz_start");
extern const uint8_t index_gz_end[] asm("_binary_index_html_gz_end");

// vorp.c
void vorp_start(void);
const char *vorp_status(void);
