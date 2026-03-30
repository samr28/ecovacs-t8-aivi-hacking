#!/bin/sh

# Configure wifi
wpa_cli -i wlan0 add_network
wpa_cli -i wlan0 set_network 0 ssid '"your_ssid"'
wpa_cli -i wlan0 set_network 0 psk '"your_pass"'
wpa_cli -i wlan0 set_network 0 key_mgmt WPA-PSK
wpa_cli -i wlan0 enable_network 0
wpa_cli -i wlan0 select_network 0

# Wait for association
sleep 8

# Get DHCP lease
dhcpcd wlan0

# Fix /etc/shells so dropbear accepts /bin/sh
echo -e "/bin/sh\n/bin/bash\n/bin/busybox" > /tmp/shells
mount --bind /tmp/shells /etc/shells

# Start dropbear
/data/dropbear -E -p 22 \
  -r /data/dropbear_keys/rsa_host_key \
  -r /data/dropbear_keys/ecdsa_host_key \
  -r /data/dropbear_keys/ed25519_host_key \
  -D /data/ssh_auth &