#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""cmd_vel command-priority mux - manual always wins, AI is capped and gated.

Stage 2 of the AI roadmap puts an autonomous publisher on the chassis for the
first time. ROS does not arbitrate between publishers, so this node becomes
the ONLY publisher to /cmd_vel and arbitrates its two inputs:

  /manual/cmd_vel  dashboard (keyboard/gamepad/e-stop). Always forwarded;
                   suppresses AI for MANUAL_WINDOW_S after the last message,
                   so touching the joystick instantly takes over.
  /ai/cmd_vel      AI behaviors. Forwarded only while /ai/enabled (latched
                   Bool from the dashboard panel) is true and manual is
                   quiet, then clamped to the AI caps below - a buggy AI
                   process cannot drive fast, ever.

On disarm while AI was driving, and on AI silence > AI_TIMEOUT_S while AI
was driving, one zero Twist is published (halt, don't coast). /mux/status
(JSON String, 2 Hz) tells the dashboard who is driving.

The existing cmd_vel_watchdog is unchanged and remains the last line of
defense on /cmd_vel itself.

Runs on the robot under transbot_dashboard.launch (ROS Melodic, py2). The
MuxCore class is pure logic, py2/3 compatible, unit-tested on the laptop
(ai/tests/test_mux.py) before deployment.
"""

import json
import threading

MANUAL_WINDOW_S = 1.0   # manual suppresses AI for this long after last msg
AI_TIMEOUT_S = 0.5      # AI silence while driving => publish one zero
AI_CAP_FWD = 0.25       # m/s   (manual cap is 0.45 dashboard-side)
AI_CAP_REV = 0.12       # m/s   (reverse is blind - tighter cap)
AI_CAP_ANG = 1.2        # rad/s (manual cap is 2.0)
STATUS_HZ = 2.0
CHECK_HZ = 10.0


class MuxCore(object):
    """Pure arbitration logic. All methods take explicit `now` seconds."""

    def __init__(self, manual_window_s=MANUAL_WINDOW_S, ai_timeout_s=AI_TIMEOUT_S,
                 ai_cap_fwd=AI_CAP_FWD, ai_cap_rev=AI_CAP_REV, ai_cap_ang=AI_CAP_ANG):
        self.manual_window_s = manual_window_s
        self.ai_timeout_s = ai_timeout_s
        self.ai_cap_fwd = ai_cap_fwd
        self.ai_cap_rev = ai_cap_rev
        self.ai_cap_ang = ai_cap_ang
        self.armed = False
        self._last_manual = None
        self._last_ai = None
        self._ai_active = False   # AI is the current publisher to /cmd_vel (may be commanding zeros)

    def _manual_recent(self, now):
        return (self._last_manual is not None
                and (now - self._last_manual) < self.manual_window_s)

    def on_manual(self, lin, ang, now):
        """Manual input: always forwarded, becomes the active source."""
        self._last_manual = now
        self._ai_active = False
        return (lin, ang)

    def on_ai(self, lin, ang, now):
        """AI input: (lin, ang) clamped to caps, or None if blocked."""
        if not self.armed or self._manual_recent(now):
            return None
        self._last_ai = now
        self._ai_active = True
        lin = min(self.ai_cap_fwd, max(-self.ai_cap_rev, lin))
        ang = min(self.ai_cap_ang, max(-self.ai_cap_ang, ang))
        return (lin, ang)

    def set_armed(self, armed, now):
        """Returns True if a zero Twist must be published (halt AI motion)."""
        was_active = self._ai_active
        self.armed = bool(armed)
        if not self.armed and was_active:
            self._ai_active = False
            return True
        return False

    def check_timeout(self, now):
        """True once when AI goes silent while it was driving."""
        if self._ai_active and self._last_ai is not None:
            if now < self._last_ai:
                # Clock stepped backward (robot has no RTC; NTP can jump).
                # Restart the timeout window from now so the guard stays
                # fail-safe rather than never firing.
                self._last_ai = now
                return False
            if (now - self._last_ai) > self.ai_timeout_s:
                self._ai_active = False
                return True
        return False

    def status(self, now):
        """Status dict for /mux/status. Note: 'ai' persists until check_timeout() observes silence -- callers must drive check_timeout() periodically (the node's 10 Hz timer does)."""
        if self._manual_recent(now):
            source = "manual"
        elif self._ai_active:
            source = "ai"
        else:
            source = "none"
        return {"source": source, "armed": self.armed,
                "caps": {"fwd": self.ai_cap_fwd, "rev": self.ai_cap_rev,
                         "ang": self.ai_cap_ang}}


def run_node():
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String

    rospy.init_node("cmd_vel_mux")
    core = MuxCore()
    lock = threading.Lock()   # rospy callbacks arrive on separate threads
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    status_pub = rospy.Publisher("/mux/status", String, queue_size=1)

    def now_s():
        return rospy.Time.now().to_sec()

    def on_manual(msg):
        with lock:
            core.on_manual(msg.linear.x, msg.angular.z, now_s())
            pub.publish(msg)   # forwarded verbatim - manual is never modified

    def on_ai(msg):
        with lock:
            out = core.on_ai(msg.linear.x, msg.angular.z, now_s())
            if out is None:
                return
            fwd = Twist()
            fwd.linear.x, fwd.angular.z = out
            pub.publish(fwd)

    def on_enabled(msg):
        with lock:
            halt = core.set_armed(msg.data, now_s())
            rospy.loginfo("ai %s", "ARMED" if msg.data else "disarmed")
            if halt:
                pub.publish(Twist())   # all zeros

    def on_check(_event):
        with lock:
            halt = core.check_timeout(now_s())
            if halt:
                rospy.logwarn("ai input went silent while driving - stopping")
                pub.publish(Twist())

    def on_status(_event):
        with lock:
            s = core.status(now_s())
        status_pub.publish(String(json.dumps(s)))

    rospy.Subscriber("/manual/cmd_vel", Twist, on_manual, queue_size=5)
    rospy.Subscriber("/ai/cmd_vel", Twist, on_ai, queue_size=5)
    rospy.Subscriber("/ai/enabled", Bool, on_enabled, queue_size=5)
    rospy.Timer(rospy.Duration(1.0 / CHECK_HZ), on_check)
    rospy.Timer(rospy.Duration(1.0 / STATUS_HZ), on_status)
    rospy.loginfo("cmd_vel mux up: manual window %.1fs, ai caps %.2f/%.2f/%.1f",
                  core.manual_window_s, core.ai_cap_fwd, core.ai_cap_rev,
                  core.ai_cap_ang)
    rospy.spin()


if __name__ == "__main__":
    run_node()
