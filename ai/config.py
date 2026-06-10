"""AI-side config — mirrors dashboard/js/config.js for everything the robot
verified (addresses, servo ids, ranges, homes, direction signs). The
dashboard config stays the source of truth; if a value changes there,
change it here too.
"""

from ai.face_tracking.tracker import AxisConfig

# ---- Network profiles (same as dashboard PROFILES) -------------------------
PROFILES = {
    "home": {
        "rosbridge_url": "ws://192.168.0.109:9090",
        "video_url": "http://192.168.0.109:8080/stream?topic=/usb_cam/image_raw&type=mjpeg",
    },
    "hotspot": {
        "rosbridge_url": "ws://192.168.1.11:9090",
        "video_url": "http://192.168.1.11:8080/stream?topic=/usb_cam/image_raw&type=mjpeg",
    },
}
DEFAULT_PROFILE = "home"

# ---- Gimbal axes ------------------------------------------------------------
# Servo ids/ranges/homes verified on the robot (FINDINGS.md / dashboard).
# sign: -1 on both axes, CONFIRMED live 2026-06-10.
# kp_deg: full-correction gain MEASURED live 2026-06-10 (command 8 deg, watch
# the face shift): pan 48.5, tilt 23.2 deg per unit error — set to ~0.85x so
# each move slightly undershoots and the next one finishes the job.
GIMBAL_PAN = AxisConfig(servo_id=1, min_deg=0, max_deg=180, home_deg=90,
                        sign=-1, kp_deg=41.0)
GIMBAL_TILT = AxisConfig(servo_id=2, min_deg=0, max_deg=180, home_deg=22,
                         sign=-1, kp_deg=20.0)

# ---- Tracker gains (retuned after first live run, 2026-06-10) ---------------
# kp 8 / step 4 overshot live: stream latency kept commanding motion after the
# face was centered, the camera blew past it and lost it. Lower slew + D-term
# damping + recenter-on-prolonged-loss fixed the failure mode.
# Move-and-settle, with MEASURED constants (calibration 2026-06-10): the
# command->servo->camera->stream->detection loop is ~0.8 s blind. Continuous
# stepping limit-cycles on that latency, and a settle window shorter than it
# double-fires corrections (that was the residual overshoot). So: command the
# full measured correction once, hold fire ~1.1 s, re-measure.
TRACKER = {
    "kp_deg": 20.0,       # global fallback gain; real gains are per-axis above
    "kd_deg": 0.0,        # unused in move-and-settle (no step-to-step memory)
    "deadband": 0.06,     # |error| below this is "centered" (~38 px at 1280)
    "max_step_deg": 20.0, # per-move clamp
    "control_rate_hz": 10.0,
    "smoothing": 0.5,     # EMA on face center while idle (reset on each move)
    "settle_updates": 11, # hold fire ~1.1 s after a move (measured 0.8 s + margin)
    "lost_recenter_after": 50,  # control steps (~5 s) lost -> drive home once
    "detect_width": 640,  # frames are downscaled to this width for detection
    "score_threshold": 0.7,
}
