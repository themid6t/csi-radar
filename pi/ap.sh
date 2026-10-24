#!/bin/sh
# Runs in the csi-ap container (alpine, --net host, NET_ADMIN): Wi-Fi AP on wlan0 for the ESP32, OFDM-only so
# every frame it sends carries CSI, plus DHCP and NAT to eth0 so the ESP32 can reach the vorp relay.
set -e
apk add --no-cache hostapd dnsmasq iptables >/dev/null
ip link set wlan0 up
ip addr show wlan0 | grep -q 10.50.0.1/ || ip addr add 10.50.0.1/24 dev wlan0
for r in "FORWARD -i wlan0 -o eth0 -j ACCEPT" "FORWARD -i eth0 -o wlan0 -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT"; do
  iptables -C $r 2>/dev/null || iptables -I $r
done
iptables -t nat -C POSTROUTING -s 10.50.0.0/24 -o eth0 -j MASQUERADE 2>/dev/null ||
  iptables -t nat -A POSTROUTING -s 10.50.0.0/24 -o eth0 -j MASQUERADE
# DHCP only (port=0: Pi-hole owns DNS on this host); the ESP32 keeps 10.50.0.2
dnsmasq --port=0 --interface=wlan0 --bind-interfaces --dhcp-range=10.50.0.10,10.50.0.50,12h \
  --dhcp-option=6,1.1.1.1,8.8.8.8 --dhcp-host=3c:8a:1f:a8:80:dc,10.50.0.2 --dhcp-leasefile=/tmp/leases
exec hostapd /etc/hostapd.conf
