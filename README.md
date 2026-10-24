# csi-radar

Wi-Fi motion radar that runs entirely on one ESP32. It reads Channel State Information (CSI) from its access point's replies, detects motion (plus experimental breathing), and serves a live 3D dashboard. That's local at `http://csi-radar.local`, and public through a [vorp](https://github.com/themid6t/vorp) tunnel when a token is set.

```
AP (Pi hostapd, OFDM-only) ──ping replies ~100/s──> ESP32: CSI -> dsp.c -> SSE
                                                      ├─ http://csi-radar.local (esp_http_server)
                                                      └─ vorp agent (TLS + yamux) -> https://csi.themidst.xyz
```

## Firmware (`firmware/`, ESP-IDF v5.5.1)

| File | What |
|---|---|
| `main/main.c` | Settings in NVS. Wi-Fi with a setup portal fallback. BOOT held for 5 s forgets Wi-Fi. mDNS. |
| `main/radar.c` | Fixed-rate ICMP pinger, CSI capture, motion events, tuning jobs, SSE event bus. |
| `main/dsp.c` | Detection in plain C. Self-check: `cc main/test_dsp.c main/dsp.c -lm && ./a.out [recording.npy]`. |
| `main/web.c` | Page, `/events` (SSE), `/cmd`, `/config`. `/setup` and `/scan` are local only. Captive DNS. |
| `main/vorp.c` | vorp agent per `docs/protocol.md`: ALPN `vorp-agent/1`, yamux, SNTP-timestamped pings. |
| `main/www/` | `index.html` (served gzipped) and `setup.html`. |

```sh
. ~/esp/esp-idf/export.sh
cd firmware && idf.py set-target esp32 && idf.py build
idf.py -p /dev/ttyUSB0 -b 115200 flash   # this board's CP2102 fails at higher baud
```

**First boot:** with no saved Wi-Fi, the board opens the network `csi-radar-setup`, and a setup page pops up. Choose the network, enter its password, and optionally add a vorp agent token, subdomain and relay. It also falls back to setup if the saved network fails for 45 s before ever connecting, and it retries that network every 2 minutes. Later changes go through `http://<board-ip>/setup` from the local network. The tunnel never routes `/setup`.

## Detection

- **Motion:** WiDetect motion statistic, the mean lag-1 autocorrelation of normalized subcarrier amplitudes over 1 s. A quiet room measures ~0. The default threshold is 0.2, with a 1 s hold.
- **Interference:** narrow tones found in principal component 1 at the full rate (for example a ceiling fan) are notched out of the motion statistic (Q = 8). Multiples of 10 Hz come from the board's own page updates and aren't reported.
- **Breathing (experimental):** each subcarrier is low-passed at 2 Hz before decimating to 10 Hz, then PCA over 30 s plus FFT, looking for a 0.1–0.7 Hz peak at 12× the band median that holds for 5 s, with no motion in the last 30 s.
- **Tuning:** record *Empty room* and *Walking* (30 s each) in the UI and apply the suggested threshold.

Only the ping replies are used: the commonest frame length, with AP retransmissions dropped, because other traffic arrives in bursts. If CSI stays under 20/s for 15 s, the page says the AP isn't supported. That means the AP sends 802.11b, which carries no CSI.

## Pi access point (`pi/`)

The home router only sends the ESP32 802.11b, so the Pi's spare `wlan0` runs an OFDM-only AP. `pi/ap.sh` runs hostapd, dnsmasq DHCP (the ESP32 is pinned to 10.50.0.2) and NAT to eth0, so the agent can reach the relay. No sudo is needed:

```sh
mkdir -p ~/csi-radar && cp pi/ap.sh ~/csi-radar/ && cp pi/hostapd.conf.example ~/csi-radar/hostapd.conf  # set wpa_passphrase
docker run -d --name csi-ap --restart unless-stopped --net host --cap-add NET_ADMIN --cap-add NET_RAW \
  -v ~/csi-radar/hostapd.conf:/etc/hostapd.conf:ro -v ~/csi-radar/ap.sh:/ap.sh:ro alpine:3.20 sh /ap.sh
```

## Limits

One link detects motion; it can't localize. The 3D view shows the link's sensing zone (Fresnel ellipsoids). Positioning needs 4+ ESP32s exchanging ESP-NOW and radio tomographic imaging. The board's heap is tight (~47 KB free), so viewers are capped at 4 local plus a few through the tunnel.

## Undo

`docker rm -f csi-ap` on the Pi. To restore MicroPython: `esptool -b 115200 write-flash 0 backup/micropython-flash.bin` (kept locally, not in git).
