#!/bin/bash
# Phase 1 discovery script — run ON THE JETSON, read-only.
#
# Prerequisite: the driver must be running. On the stock Yahboom image the
# Transbot stack AUTO-STARTS at boot, so normally nothing needs launching —
# just boot the robot and run this.
#
# Usage:
#   bash discovery.sh
#
# Writes a full interface dump to ~/transbot_discovery.txt. Copy that file
# back to the laptop (scp) to author FINDINGS.md from verified data.

# NOTE: no `set -u` — ROS setup scripts reference unset variables by design.
source /opt/ros/melodic/setup.bash
[ -f "$HOME/transbot_ws/devel/setup.bash" ] && source "$HOME/transbot_ws/devel/setup.bash"

OUT="$HOME/transbot_discovery.txt"
: > "$OUT"

# The factory stack starts via desktop autologin and can take a couple of
# minutes after power-on. Wait for the ROS master rather than failing fast.
echo "Waiting for ROS master (up to 180s)..."
for i in $(seq 1 36); do
    if rostopic list >/dev/null 2>&1; then
        echo "ROS master is up."
        break
    fi
    if [ "$i" -eq 36 ]; then
        echo "WARNING: ROS master never came up - topic sections will be empty." | tee -a "$OUT"
    fi
    sleep 5
done

section() {
    echo ""            >> "$OUT"
    echo "=== $1 ==="  >> "$OUT"
    echo "$1 ..."
}

section "ROS VERSION / ENV"
rosversion -d >> "$OUT" 2>&1
echo "ROS_MASTER_URI=$ROS_MASTER_URI" >> "$OUT"

section "NODES"
rosnode list >> "$OUT" 2>&1

section "TOPIC LIST"
rostopic list >> "$OUT" 2>&1

section "TOPIC INFO + MESSAGE DEFINITIONS"
for t in $(rostopic list 2>/dev/null); do
    echo "" >> "$OUT"
    echo "--- topic: $t ---" >> "$OUT"
    rostopic info "$t" >> "$OUT" 2>&1
    # Extract the message type and dump its full field layout
    type=$(rostopic info "$t" 2>/dev/null | awk '/^Type:/ {print $2}')
    if [ -n "$type" ]; then
        echo "fields of $type:" >> "$OUT"
        rosmsg show "$type" >> "$OUT" 2>&1
    fi
done

section "SERVICE LIST"
rosservice list >> "$OUT" 2>&1

section "SERVICE TYPES + DEFINITIONS"
for s in $(rosservice list 2>/dev/null); do
    echo "" >> "$OUT"
    echo "--- service: $s ---" >> "$OUT"
    stype=$(rosservice type "$s" 2>/dev/null)
    echo "Type: $stype" >> "$OUT"
    if [ -n "$stype" ]; then
        rossrv show "$stype" >> "$OUT" 2>&1
    fi
done

section "TOPIC SAMPLES (one message each, 5s timeout, telemetry only)"
# Sample only known-safe telemetry candidates; never publish anything.
for t in $(rostopic list 2>/dev/null | grep -iE 'vel|imu|volt|batt|edition|odom'); do
    echo "" >> "$OUT"
    echo "--- sample: $t ---" >> "$OUT"
    timeout 5 rostopic echo -n 1 "$t" >> "$OUT" 2>&1
done

section "CAMERA TOPIC CANDIDATES"
rostopic list 2>/dev/null | grep -iE 'image|camera|usb_cam|compressed' >> "$OUT"

section "REQUIRED PACKAGES INSTALLED?"
dpkg -l ros-melodic-rosbridge-suite ros-melodic-web-video-server >> "$OUT" 2>&1

section "AUTOSTART MECHANISM (factory stack starts at boot - find out how)"
echo "--- systemd units mentioning transbot/ros ---" >> "$OUT"
systemctl list-unit-files 2>/dev/null | grep -iE 'transbot|ros|yahboom' >> "$OUT"
echo "--- desktop autostart entries ---" >> "$OUT"
ls -la "$HOME/.config/autostart/" >> "$OUT" 2>&1
for f in "$HOME"/.config/autostart/*.desktop; do
    [ -f "$f" ] && { echo "-- $f:" >> "$OUT"; cat "$f" >> "$OUT"; }
done
echo "--- rc.local ---" >> "$OUT"
cat /etc/rc.local >> "$OUT" 2>&1
echo "--- crontab ---" >> "$OUT"
crontab -l >> "$OUT" 2>&1
echo "--- running ros-related processes ---" >> "$OUT"
ps aux | grep -iE 'ros|transbot' | grep -v grep >> "$OUT" 2>&1
echo "--- start_transbot.sh (what the factory autostart launches) ---" >> "$OUT"
cat "$HOME/Transbot/transbot/start_transbot.sh" >> "$OUT" 2>&1

section "DONE"
echo "Wrote $OUT"
echo ""
echo "Discovery complete. Output: $OUT"
