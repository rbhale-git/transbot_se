#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""cmd_vel watchdog — stops the robot when motion commands go silent.

The Yahboom driver executes the last /cmd_vel forever; if the teleop link
degrades mid-drive (Wi-Fi dropout, VPN hiccup, browser crash) no zero-Twist
ever arrives and the robot runs away. This node subscribes to /cmd_vel and,
whenever the last command was non-zero and no fresh command has arrived
within `timeout`, publishes zero Twists itself.

Normal driving is unaffected: the dashboard republishes at 10 Hz while a key
is held, so the age of the last command stays ~100 ms. The watchdog's own
zero publications satisfy its zero-state check, so it fires once per event,
not repeatedly.

Runs on the robot as part of transbot_dashboard.launch (ROS Melodic, py2).
"""

import rospy
from geometry_msgs.msg import Twist

TIMEOUT_S = 0.5     # silence longer than this while moving => stop
CHECK_HZ = 20.0     # watchdog poll rate
STOP_BURST = 3      # zero Twists to publish when triggering


class CmdVelWatchdog(object):
    def __init__(self):
        self.last_msg_time = rospy.Time.now()
        self.last_was_motion = False
        self.timeout = rospy.Duration(rospy.get_param("~timeout", TIMEOUT_S))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / CHECK_HZ), self.check)
        rospy.loginfo("cmd_vel watchdog up: timeout %.2fs", self.timeout.to_sec())

    def on_cmd(self, msg):
        self.last_msg_time = rospy.Time.now()
        self.last_was_motion = (
            abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3
        )

    def check(self, _event):
        if not self.last_was_motion:
            return
        if rospy.Time.now() - self.last_msg_time > self.timeout:
            rospy.logwarn("cmd_vel silent %.2fs while moving - stopping robot",
                          (rospy.Time.now() - self.last_msg_time).to_sec())
            stop = Twist()  # all zeros
            for _ in range(STOP_BURST):
                self.pub.publish(stop)
            # Our own zero arrives via on_cmd and clears last_was_motion.


if __name__ == "__main__":
    rospy.init_node("cmd_vel_watchdog")
    CmdVelWatchdog()
    rospy.spin()
