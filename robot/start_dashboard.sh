#!/bin/bash
# One-command robot-side start for the Transbot SE dashboard stack:
# driver bringup + rosbridge (ws :9090) + web_video_server (http :8080).
#
# Phase 2 deploys this and transbot_dashboard.launch to ~/transbot_dashboard/
# on the Jetson. Run it there with:  bash ~/transbot_dashboard/start_dashboard.sh
set -e

source /opt/ros/melodic/setup.bash
source "$HOME/transbot_ws/devel/setup.bash"

exec roslaunch "$HOME/transbot_dashboard/transbot_dashboard.launch"
