#!/bin/bash
# =============================================================================
# wifi_failover.sh — keep the robot reachable on SOME network, always.
#
# Run periodically by wifi-failover.timer. Logic:
#   - If any Wi-Fi connection is active -> do nothing. NEVER preempt an
#     active link (switching networks mid-teleop would cut the control link).
#   - Otherwise: rescan; if the home SSID is visible, try home_wifi first;
#     if not (or if that fails), start the robot's own hotspot AP so the
#     dashboard's HOTSPOT profile can reach it anywhere.
#
# The home SSID is read from the home_wifi profile — nothing hardcoded here.
# =============================================================================

HOME_CON="home_wifi"
AP_CON="Transbot"

# Any active Wi-Fi connection (client or AP) means we're fine.
if nmcli -t -f NAME,TYPE con show --active | grep -q 802-11-wireless; then
    exit 0
fi

logger -t wifi-failover "no active Wi-Fi - attempting recovery"

HOME_SSID=$(nmcli -g 802-11-wireless.ssid con show "$HOME_CON" 2>/dev/null)

nmcli device wifi rescan 2>/dev/null
sleep 6

if [ -n "$HOME_SSID" ] && nmcli -t -f SSID device wifi list | grep -qxF "$HOME_SSID"; then
    logger -t wifi-failover "home SSID visible - trying $HOME_CON"
    nmcli con up "$HOME_CON" && exit 0
    logger -t wifi-failover "$HOME_CON failed - falling back to hotspot"
fi

logger -t wifi-failover "starting hotspot AP ($AP_CON)"
nmcli con up "$AP_CON"
