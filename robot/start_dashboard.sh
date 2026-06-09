#!/bin/bash
# Robot-side start for the dashboard bridge. The factory stack (driver +
# web_video_server) autostarts at boot; this only adds rosbridge on :9090.
#
# Phase 2 deploys this and transbot_dashboard.launch to ~/transbot_dashboard/.
# Run on the Jetson with:  bash ~/transbot_dashboard/start_dashboard.sh
set -e

source /opt/ros/melodic/setup.bash
source "$HOME/transbot_ws/devel/setup.bash"

exec roslaunch "$HOME/transbot_dashboard/transbot_dashboard.launch"
