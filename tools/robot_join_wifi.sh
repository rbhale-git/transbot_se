#!/bin/bash
# =============================================================================
# robot_join_wifi.sh — run ON THE ROBOT to join a home Wi-Fi network.
#
# Saves the current hotspot connection name first, so the hotspot can be
# restored at any time with:
#     sudo nmcli con up "$(cat ~/hotspot_connection_name.txt)"
#
# The home profile gets a higher autoconnect priority, so the robot keeps
# rejoining home Wi-Fi across reboots while we develop. Credentials are
# passed as arguments and stored only in the robot's NetworkManager (normal
# for any Wi-Fi client) — never in this repo.
#
# Usage (on the robot, via SSH over the hotspot — the session WILL drop
# when the switch happens, that is expected):
#     sudo nohup bash ~/robot_join_wifi.sh "YourSSID" "YourWifiPassword" \
#         > ~/join_wifi.log 2>&1 &
#
# After ~30s, check the robot's OLED screen for its new IP on the home network.
# =============================================================================

SSID="$1"
PSK="$2"
if [ -z "$SSID" ] || [ -z "$PSK" ]; then
    echo "usage: sudo bash robot_join_wifi.sh \"SSID\" \"PASSWORD\""
    exit 1
fi

# Record the active wireless connection (the hotspot) for later restoration.
nmcli -t -f NAME,TYPE con show --active | grep -i wireless | cut -d: -f1 \
    > /home/jetson/hotspot_connection_name.txt
echo "Saved hotspot connection name: $(cat /home/jetson/hotspot_connection_name.txt)"

# (Re)create the home Wi-Fi profile.
nmcli con delete home_wifi >/dev/null 2>&1
nmcli con add type wifi ifname wlan0 con-name home_wifi \
    ssid "$SSID" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK" \
    connection.autoconnect yes connection.autoconnect-priority 10

echo "Switching to home Wi-Fi now - the hotspot (and this SSH session) will drop."
nmcli con up home_wifi
RC=$?

echo "nmcli con up exited with code $RC"
if [ $RC -ne 0 ]; then
    echo "Join FAILED - restoring hotspot..."
    nmcli con up "$(cat /home/jetson/hotspot_connection_name.txt)"
fi
