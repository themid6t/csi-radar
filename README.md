# csi-radar

Wi-Fi motion radar: an ESP32 reads Channel State Information (CSI) from Wi-Fi packets and detects people moving between it and a transmitter. Detection runs on the chip, and a Pi serves a live UI.

```
Pi wlan0 (hostapd AP "csi-radar", 10.50.0.1)  ⇄  ESP32 (10.50.0.2)
                                                   • pings the Pi ~100×/s, reads CSI off every reply
                                                   • motion = moving variance of subcarrier turbulence > threshold
                                                   • GPIO2 LED on while motion
                                                   • UDP JSON ~10/s → Pi:5005, commands on :5006
Pi: server.py (stdlib) → SSE → index.html on :8765 → vorp tunnel https://csi.themidst.xyz
```

## Why the Pi is the access point

The home router's 2.4 GHz radio sends the ESP32 only 802.11b (DSSS 5.5 Mbps) frames. Those frames carry no CSI, so callbacks stopped after DHCP. With hostapd on the Pi's otherwise unused `wlan0`, the traffic is OFDM, which gives ~200–350 CSI packets/s. AMPDU RX is also disabled (`sdkconfig.defaults`), because aggregated frames carry no CSI either.

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
mkdir -p ~/csi-radar && cp pi/server.py pi/index.html ~/csi-radar/
cp pi/hostapd.conf.example ~/csi-radar/hostapd.conf    # set wpa_passphrase

docker run -d --name csi-ap --restart unless-stopped --net host --cap-add NET_ADMIN --cap-add NET_RAW \
  -v ~/csi-radar/hostapd.conf:/etc/hostapd.conf:ro alpine:3.20 \
  sh -c 'apk add --no-cache hostapd iw >/dev/null && ip link set wlan0 up && exec hostapd /etc/hostapd.conf'

docker run -d --name csi-ui --restart unless-stopped --net host -v ~/csi-radar:/app:ro \
  python:3.13-alpine python3 -u /app/server.py
```

`wlan0` keeps its static `10.50.0.1/24`, and the ESP32 uses a static `10.50.0.2`, so no DHCP server is needed.

## Tuning

- Calibration runs at boot and on **Recalibrate**. It takes about 1000 packets (~4 s) of a still room, and the threshold is set to 2× the p99 variance.
- **More / Less sensitive** scales the threshold by 1.5×. The threshold isn't persisted, so a reboot recalibrates.
- Placement: put the ESP32 across the room from the Pi. Motion is detected near the line between them.

## Undo

`docker rm -f csi-ap csi-ui` on the Pi. To restore the original MicroPython, write the flash backup back to the board with `esptool -b 115200 write-flash 0 backup/micropython-flash.bin` (kept locally, not in git).
