#!/bin/sh
# Runs in the csi-ap container (alpine, --net host, NET_ADMIN): Wi-Fi AP for the ESP32, OFDM-only so every
# frame it sends carries CSI, plus DHCP and NAT to the uplink so the ESP32 can reach a vorp relay.
# WLAN must match `interface=` in hostapd.conf.
set -e
WLAN=${WLAN:-wlan0} UPLINK=${UPLINK:-eth0} NET=${NET:-10.50.0}
apk add --no-cache hostapd dnsmasq iptables >/dev/null
ip link set $WLAN up
ip addr show $WLAN | grep -q $NET.1/ || ip addr add $NET.1/24 dev $WLAN
for r in "FORWARD -i $WLAN -o $UPLINK -j ACCEPT" "FORWARD -i $UPLINK -o $WLAN -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT"; do
  iptables -C $r 2>/dev/null || iptables -I $r
done
iptables -t nat -C POSTROUTING -s $NET.0/24 -o $UPLINK -j MASQUERADE 2>/dev/null ||
  iptables -t nat -A POSTROUTING -s $NET.0/24 -o $UPLINK -j MASQUERADE
# DHCP only (port=0 leaves DNS to whatever already runs on the host); find the board as csi-radar.local
dnsmasq --port=0 --interface=$WLAN --bind-interfaces --dhcp-range=$NET.10,$NET.50,12h \
  --dhcp-option=6,1.1.1.1,8.8.8.8 --dhcp-leasefile=/tmp/leases
exec hostapd /etc/hostapd.conf
