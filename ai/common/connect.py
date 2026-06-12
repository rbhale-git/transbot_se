"""Connect to rosbridge and PROVE commands reach the robot.

The robot's old rosbridge (0.11/Melodic) can half-fail publisher
registration PER TOPIC and silently drop every command on it for the
session's lifetime — a session can wiggle the gimbal fine while
/ai/cmd_vel is dead ("arm does nothing", seen live 2026-06-11; docs/
AI_NOTEBOOK.md). No error reaches the websocket client, so each fresh
session is checked in two phases:

1. Registration: ask rosapi (which runs inside the rosbridge process)
   whether /rosbridge_websocket is registered as a publisher of every
   topic behaviors publish — the same check the dashboard runs. If rosapi
   itself does not answer (mock server), the phase is UNVERIFIED and the
   wiggle phase decides alone; never treated as a fault.
2. Wiggle: tilt the gimbal and verify the camera view changed —
   end-to-end proof through the servo driver.

Escalation: fresh sessions re-register cleanly, so reconnect up to
`attempts` times; if still bad, spend the ONE `heal` per run (SSH restart
of rosbridge-dashboard.service — ai/common/heal.py) and try one final
fresh session. The heal restarts the whole robot stack, so the video
source is reopened after it.

Shared by every behavior. The sink_factory must return an object with
send(servo_id, angle), publishers_of(topic), and disconnect();
disconnect() is used between retries because close() would kill the
process-wide Twisted reactor for good.
"""

import time

from ai.common.video import frames_differ

# Every topic RosClient advertises, for any behavior. Mirrors the
# constants in ai/common/ros_client.py — NOT imported from there, because
# behaviors import this module on dry runs where roslibpy may be absent
# (RosClient is deliberately imported lazily inside the sinks).
REGISTRATION_TOPICS = ("/PWMServo", "/ai/cmd_vel", "/ai/status")
ROSBRIDGE_NODE = "/rosbridge_websocket"


def registration_failures(sink, topics=REGISTRATION_TOPICS):
    """Topics rosbridge dropped ([] = all good), or None if unverifiable."""
    dead = []
    for topic in topics:
        try:
            publishers = sink.publishers_of(topic)
        except Exception:
            return None
        if ROSBRIDGE_NODE not in publishers:
            dead.append(topic)
    return dead


def _wiggle_moves_camera(sink, src, tilt_cfg, sleep):
    before = src.read(timeout_s=3.0)
    sink.send(tilt_cfg.servo_id, tilt_cfg.home_deg + 25)
    sleep(1.8)
    after = src.read(timeout_s=3.0)
    sink.send(tilt_cfg.servo_id, tilt_cfg.home_deg)
    sleep(1.0)
    return before is not None and after is not None and frames_differ(before, after)


def connect_with_actuation_check(sink_factory, src, tilt_cfg, attempts=3,
                                 heal=None, sleep=time.sleep):
    """Return a verified sink, or raise RuntimeError naming what failed."""
    healed = False
    budget = attempts
    attempt = 0
    last_fail = "no attempt made"
    while attempt < budget:
        attempt += 1
        sink = sink_factory()
        dead = registration_failures(sink)
        if dead is None:
            print("registration check unverified (rosapi did not answer) - "
                  "relying on the wiggle check alone")
        if not dead:  # [] verified-ok, or None unverified: wiggle decides
            if _wiggle_moves_camera(sink, src, tilt_cfg, sleep):
                print(f"actuation check passed (attempt {attempt})")
                return sink
            last_fail = ("commands are not reaching the gimbal "
                         "(tilt wiggle did not change the camera view)")
        else:
            last_fail = ("rosbridge dropped publisher registration for "
                         + ", ".join(dead))
        print(f"actuation check FAILED on attempt {attempt}: {last_fail} - "
              "reconnecting with a fresh session")
        sink.disconnect()
        if attempt == budget and heal is not None and not healed:
            healed = True
            print("escalating: restarting rosbridge-dashboard.service on "
                  "the robot (drops camera + link ~15-30 s) ...")
            ok, detail = heal()
            if not ok:
                raise RuntimeError(f"auto-heal failed ({detail}); "
                                   f"last actuation failure: {last_fail}")
            print("rosbridge is back - reopening the video stream ...")
            src.reopen()
            budget += 1  # one post-heal session
        sleep(2.0)
    raise RuntimeError(
        f"actuation check failed after {attempt} rosbridge sessions"
        + (" including a service restart" if healed else "")
        + f"; last failure: {last_fail} - check the robot "
        "(journalctl -u rosbridge-dashboard.service)")
