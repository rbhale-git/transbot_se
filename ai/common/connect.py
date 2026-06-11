"""Connect to rosbridge and PROVE commands reach the servos.

The robot's old rosbridge can half-fail its publisher registration and
silently drop every command for the session's lifetime (seen on two boots;
docs/AI_NOTEBOOK.md). No error reaches the websocket client, so the only
honest check is end-to-end: wiggle the tilt servo and verify the camera view
changed. A fresh session re-registers cleanly, so on failure we reconnect
and try again.

Shared by every behavior (extracted from face_tracking in stage 2). The
sink_factory must return an object with send(servo_id, angle) and
disconnect(); disconnect() is used between retries because close() would
kill the process-wide Twisted reactor for good.
"""

import time

from ai.common.video import frames_differ


def connect_with_actuation_check(sink_factory, src, tilt_cfg, attempts=3):
    for attempt in range(1, attempts + 1):
        sink = sink_factory()
        before = src.read(timeout_s=3.0)
        sink.send(tilt_cfg.servo_id, tilt_cfg.home_deg + 25)
        time.sleep(1.8)
        after = src.read(timeout_s=3.0)
        sink.send(tilt_cfg.servo_id, tilt_cfg.home_deg)
        time.sleep(1.0)
        if before is not None and after is not None and frames_differ(before, after):
            print(f"actuation check passed (attempt {attempt})")
            return sink
        print(f"actuation check FAILED on attempt {attempt}: commands are not "
              "reaching the gimbal (known rosbridge dropped-session bug) - "
              "reconnecting with a fresh session")
        sink.disconnect()
        time.sleep(2.0)
    raise RuntimeError(
        "gimbal did not respond after %d fresh rosbridge sessions - "
        "check the robot (journalctl -u rosbridge-dashboard.service)" % attempts)
