# csi-radar

A Wi-Fi motion radar that runs entirely on one ESP32. It reads the Channel State Information (CSI) of its access point's replies, detects motion (and, experimentally, a still person's breathing), and serves a live dashboard with a 3D view of the sensing zone. That's local at `http://csi-radar.local`, and optionally public through a [vorp](https://github.com/themid6t/vorp) tunnel.

```
Wi-Fi AP ──ping replies ~100/s──> ESP32: CSI -> detection -> live page
                                    ├─ http://csi-radar.local        (always)
                                    └─ https://<name>.<vorp relay>   (optional, with a vorp agent token)
```

No server, no cloud, no app: the board does the sensing, detection and web serving itself.

## What you need

- **An ESP32 with 4 MB flash.** This was built and tested on an ESP32-D0WD-V3 dev board (ESP32-WROOM-32). Other chips (S2/S3/C3) would need porting.
- **A USB cable and a Linux or macOS machine** to flash it. Windows works with ESP-IDF too.
- **A 2.4 GHz Wi-Fi network** whose AP sends 802.11g/n frames to the ESP32. Most 802.11n routers do. See [Access point](#access-point) if yours doesn't.
- Optional: **a vorp relay and an agent token** for a public link.

## 1. Install ESP-IDF v5.5

Follow Espressif's [getting started guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32/get-started/), or:

```sh
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.1 --recursive --depth 1 https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32
```

## 2. Build and flash

```sh
git clone https://github.com/themid6t/csi-radar.git && cd csi-radar/firmware
. ~/esp/esp-idf/export.sh
idf.py set-target esp32
idf.py build                                # fetches the mDNS component on the first build
idf.py -p /dev/ttyUSB0 -b 115200 flash      # macOS: /dev/cu.usbserial-*; Windows: COM3 etc.
```

The first build downloads the `espressif/mdns` component, so it needs internet. If flashing fails, see [Troubleshooting](#troubleshooting).

## 3. Connect it to Wi-Fi

1. Power the board. With no saved network, it opens its own Wi-Fi network, **`csi-radar-setup`** (open, no password), and the onboard LED blinks slowly.
2. Join that network from a phone or laptop. A setup page should pop up. If it doesn't, open `http://192.168.4.1/setup`.
3. Pick your network, enter its password, and press **Save and restart**.
4. Rejoin your usual Wi-Fi and open **`http://csi-radar.local`**. If your device doesn't resolve `.local` names (some Android versions), use the board's IP from your router's client list.

If the saved network is unavailable at boot for 45 s, the board goes back to setup mode, and it keeps retrying the saved network every 2 minutes. Holding the **BOOT** button for 5 s forgets the network. Settings can be changed later at `http://csi-radar.local/setup`, from your local network only.

If the page header says **"AP not supported (no CSI)"**, your AP is sending 802.11b frames. See [Access point](#access-point).

## 4. Optional: public link through vorp

[vorp](https://github.com/themid6t/vorp) is a self-hosted reverse tunnel. The board has a built-in vorp agent, so it can publish the page without any other machine.

1. Run a vorp relay, or use one you have access to, and create an agent token. If the token is restricted to certain names, include the subdomain you want.
2. On `http://csi-radar.local/setup`, open **Public link (vorp tunnel)** and enter:
   - **Agent token**
   - **Subdomain:** for example `radar`, or leave it empty for a random name if your token allows that.
   - **Relay host:** for example `vorp.example.com`.
3. Save. After the restart, the setup page shows **Public link: online** and the URL.

Through the tunnel, only the radar page, its live stream and its controls are reachable. The setup page is not. Anyone with the link can change the threshold, start recordings and edit the room layout, though only those whitelisted inputs are accepted.

## 5. Placement and tuning

- The ESP32 senses motion in the zone around the straight line between it and the AP, a stretched ellipsoid a few tens of centimetres thick. Put the two across the room from each other, about 1 m high. Then enter both positions and the room size under **Room setup**, so the 3D view matches.
- Press **Empty room** and leave the room for 30 s. Then press **Walking** and walk back and forth between the AP and the ESP32 for 30 s. Click **Apply** on the suggested threshold.
- Spinning things such as a ceiling fan create periodic interference. It's detected, filtered out and reported on the page, but switching them off still helps.
- **Breathing** only shows for someone sitting still close to the link, and is experimental.

## Access point

The ESP32 only gets CSI from 802.11g/n frames. Some older or b/g-only routers send 802.11b frames to a weak client, and those carry none. Your options:

- Set the router's 2.4 GHz mode to "g/n only" or "n only", or turn off 802.11b rates, if its settings have that.
- Use a separate AP. `pi/` turns any Linux box with a spare Wi-Fi interface, such as a Raspberry Pi's `wlan0`, into an OFDM-only AP with DHCP and NAT. It runs in Docker and needs no sudo:

```sh
mkdir -p ~/csi-radar && cp pi/ap.sh ~/csi-radar/
cp pi/hostapd.conf.example ~/csi-radar/hostapd.conf   # set ssid, wpa_passphrase, country_code (and interface)
docker run -d --name csi-ap --restart unless-stopped --net host --cap-add NET_ADMIN --cap-add NET_RAW \
  -e WLAN=wlan0 -e UPLINK=eth0 \
  -v ~/csi-radar/hostapd.conf:/etc/hostapd.conf:ro -v ~/csi-radar/ap.sh:/ap.sh:ro alpine:3.20 sh /ap.sh
```

`WLAN` is the Wi-Fi interface for the AP, and must match `interface=` in hostapd.conf. `UPLINK` is the interface with internet access, which the vorp agent needs. The AP's subnet is `10.50.0.0/24` (change it with `-e NET=10.60.0`). Then point the board at this network in step 3.

## How it works

| File | What |
|---|---|
| `firmware/main/main.c` | Settings in NVS, Wi-Fi, setup-portal fallback, BOOT-button reset, mDNS. |
| `firmware/main/radar.c` | Fixed-rate ICMP pinger, CSI capture (ping replies only, retransmissions dropped), motion events, tuning jobs, the event bus the page streams from. |
| `firmware/main/dsp.c` | Detection, in plain C so it also builds on a PC. |
| `firmware/main/web.c` | Page, `/events` (Server-Sent Events), `/cmd`, `/config`; local-only `/setup` and `/scan`; captive-portal DNS. |
| `firmware/main/vorp.c` | vorp agent: TLS (ALPN `vorp-agent/1`), yamux, SNTP-timestamped heartbeats, per [docs/protocol.md](https://github.com/themid6t/vorp/blob/main/docs/protocol.md). |
| `firmware/main/www/` | `index.html` (the dashboard, served gzipped) and `setup.html`. |

- **Motion:** WiDetect's motion statistic, the mean lag-1 autocorrelation of each subcarrier's normalized amplitude over 1 s. A still channel is white noise (≈0), and movement correlates consecutive packets (towards 1). The default threshold is 0.2.
- **Interference:** narrow tones in the first principal component (sampled at the full rate) are notched out before the motion statistic. Otherwise a spinning fan alone would push it well past the threshold.
- **Breathing:** each subcarrier is low-passed at 2 Hz before downsampling to 10 Hz, so fan harmonics can't alias into the breathing band. Then PCA over 30 s and an FFT. A 0.1–0.7 Hz peak must tower over the band and hold steady for 5 s.

Check the detection code on a PC: `cc firmware/main/test_dsp.c firmware/main/dsp.c -lm && ./a.out`.

**Limits:** one link detects motion but can't locate it, because the board has one antenna and 20 MHz of bandwidth. Locating motion would need 4 or more boards and radio tomographic imaging. The board's RAM allows about 4 local viewers plus a few through the tunnel.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Flashing fails or times out | Use `-b 115200`. Some boards' USB-serial chips fail at higher speeds. Hold BOOT while flashing starts if the board doesn't enter download mode. |
| No `csi-radar-setup` network | It only appears with no saved Wi-Fi, or 45 s after the saved one fails. Hold BOOT for 5 s to forget the saved network. |
| `csi-radar.local` doesn't resolve | Use the board's IP from your router. Android often lacks mDNS. |
| "AP not supported (no CSI)" | The AP sends 802.11b. See [Access point](#access-point). |
| Low "CSI packets / s" (normal is ~90–100) | Move the board closer to the AP, or change to a less busy channel. |
| CSI rate drops for a few seconds every ~15 s, on any channel | The AP's radio is going deaf on a schedule. On a Raspberry Pi AP, Wi-Fi shares its radio with Bluetooth, and a running Bluetooth scan (`bluetoothctl show` says `Discovering: yes`) causes exactly this. Stop it with `bluetoothctl power off` then `power on`, or disable Bluetooth if you don't use it. |
| Public link: `SUBDOMAIN_NOT_ALLOWED` / `SUBDOMAIN_TAKEN` | The token doesn't allow that name, or another agent holds it. The board retries a taken name every 60 s. |
| Public link stuck on "waiting for the clock" | The board needs NTP (UDP 123) to reach `time.cloudflare.com` or `pool.ntp.org`. |

To watch the logs: `idf.py -p /dev/ttyUSB0 monitor`. Note that opening the serial port resets most dev boards.

## License

MIT, see [LICENSE](LICENSE).
