# csi-radar

Wi-Fi motion radar: an ESP32 captures Channel State Information (CSI) from Wi-Fi packets, and a Pi turns it into motion detection, experimental breathing detection, and a live 3D dashboard.

```
Pi wlan0 (hostapd AP "csi-radar", 10.50.0.1)  ⇄  ESP32 (10.50.0.2)
                                                   • pings the Pi ~100×/s, captures LLTF CSI (52 subcarriers, I/Q) of every reply
                                                   • streams raw I/Q over UDP → Pi:5005 (10 packets per datagram)
                                                   • GPIO2 LED driven by the Pi ("led 0|1" on :5006)
Pi: server.py + detect.py (numpy) → SSE → index.html on :8765 → vorp tunnel https://csi.themidst.xyz
```

## Detection (`pi/detect.py`)

- **Motion:** WiDetect-style motion statistic, the mean lag-1 autocorrelation of each subcarrier's (gain-normalized) amplitude over 1 s. A static channel is white noise (≈0); movement correlates consecutive packets (→1). A quiet room measures ~0.05. The default threshold is 0.2, with a 1 s hold.
- **Breathing (experimental):** top 3 principal components of 30 s of amplitudes at the full ~100 Hz, FFT'd, with the 0.1–0.7 Hz peak required to be ≥12× the band median and stable (±2/min) for 5 s. Only when there's been no motion for 30 s, and only for a person near the link.
- **Interference:** narrowband tones in 0.8–45 Hz are reported in the UI. The first recording here showed 1.78 / 5.33 Hz (3 blades × 107 rpm), a ceiling fan, which keeps the channel moving.

`python3 pi/detect.py` runs the self-check. It also runs during the Docker build.

## Tuning

In the UI, record **Empty room** and **Walking** (30 s each). The suggested threshold is the midpoint between the empty room's p99 and walking's p20, or 1.1× the empty p99 if they overlap. Click **Apply** to use it. Raw recordings are saved on the Pi in `~/csi-radar/data/rec/*.npy` (newest 30), so the detector can be improved offline. **Quick calibrate** sets 1.5× the p99 of 10 s, assuming the room is empty. The threshold, recordings summary and room geometry persist in `~/csi-radar/data/state.json`.

## Limits

One link can only detect motion. It can't localize, because the ESP32 has one antenna (no angle of arrival) and 20 MHz of bandwidth gives ~15 m range resolution. The 3D view shows the link's real sensing zone (Fresnel ellipsoids), lit by the motion statistic. Positioning needs 4+ ESP32 nodes and radio tomographic imaging.

## Why the Pi is the access point

The home router's 2.4 GHz radio sends the ESP32 only 802.11b (DSSS) frames, which carry no CSI. With hostapd on the Pi's otherwise unused `wlan0`, it's OFDM-only (`supported_rates`/`basic_rates`), which gives a steady ~100 CSI packets/s. AMPDU RX is also disabled (`sdkconfig.defaults`), because aggregated frames carry no CSI either. The ping timeout is 15 ms, because esp_ping waits for each reply and a lost reply would stall CSI for the full timeout.

## Firmware (ESP-IDF v5.5.1)

```sh
cp firmware/main/secrets.h.example firmware/main/secrets.h   # fill in the AP password
. ~/esp/esp-idf/export.sh
cd firmware && idf.py set-target esp32 && idf.py build
idf.py -p /dev/ttyUSB0 -b 115200 flash   # this board's CP2102 fails at higher baud
```

## Pi

No sudo needed; both pieces run as Docker containers with `--restart unless-stopped`:

```sh
mkdir -p ~/csi-radar/data && cp pi/* ~/csi-radar/
cp pi/hostapd.conf.example ~/csi-radar/hostapd.conf    # set wpa_passphrase

docker run -d --name csi-ap --restart unless-stopped --net host --cap-add NET_ADMIN --cap-add NET_RAW \
  -v ~/csi-radar/hostapd.conf:/etc/hostapd.conf:ro alpine:3.20 \
  sh -c 'apk add --no-cache hostapd iw >/dev/null && ip link set wlan0 up && exec hostapd /etc/hostapd.conf'

cd ~/csi-radar && docker build -t csi-ui . && \
docker run -d --name csi-ui --restart unless-stopped --net host -v ~/csi-radar/data:/data csi-ui
```

`wlan0` keeps its static `10.50.0.1/24`, and the ESP32 uses a static `10.50.0.2`, so no DHCP server is needed. Placement: put the ESP32 across the room from the Pi, about 1 m high, and enter both positions under **Room setup** in the UI.

## Undo

`docker rm -f csi-ap csi-ui` on the Pi. To restore the original MicroPython: `esptool -b 115200 write-flash 0 backup/micropython-flash.bin` (kept locally, not in git).
