#!/bin/bash
# Robot-side start for the FULL dashboard stack: driver bringup + camera +
# web_video_server + rosbridge. Run by rosbridge-dashboard.service at boot;
# can also be run manually:  bash ~/transbot_dashboard/start_dashboard.sh
set -e

# Same workspace chain as the robot's .bashrc (bringup needs nodes from the
# transbot_library workspace: imu_calib, imu_filter_madgwick, EKF).
source /opt/ros/melodic/setup.bash
source /home/jetson/software/transbot_library/devel/setup.bash --extend
source /home/jetson/transbot_ws/devel/setup.bash --extend

exec roslaunch "$HOME/transbot_dashboard/transbot_dashboard.launch"
