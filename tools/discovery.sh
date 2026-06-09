#!/bin/bash
# Phase 1 discovery script — run ON THE JETSON, read-only.
#
# Prerequisite: the robot bringup must already be running in another terminal:
#   roslaunch transbot_bringup bringup.launch
#
# Usage:
#   bash discovery.sh
#
# Writes a full interface dump to ~/transbot_discovery.txt. Copy that file
# back to the laptop (scp) to author FINDINGS.md from verified data.

set -u
source /opt/ros/melodic/setup.bash
[ -f "$HOME/transbot_ws/devel/setup.bash" ] && source "$HOME/transbot_ws/devel/setup.bash"

OUT="$HOME/transbot_discovery.txt"
: > "$OUT"

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

section "DONE"
echo "Wrote $OUT"
echo ""
echo "Discovery complete. Output: $OUT"
