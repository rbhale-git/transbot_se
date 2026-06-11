"""AI-side config — mirrors dashboard/js/config.js for everything the robot
verified (addresses, servo ids, ranges, homes, direction signs). The
dashboard config stays the source of truth; if a value changes there,
change it here too.
"""

from ai.face_tracking.tracker import AxisConfig

# ---- Network profiles (same as dashboard PROFILES) -------------------------
# AI streams are downscaled server-side (width/height/quality params): the
# Nano re-encodes per client and full 720p saturates it — stream latency then
# creeps up over a session (measured live 2026-06-11). Detectors letterbox to
# 640 px anyway, so 480p costs nothing. Override with --source for full res.
_DOWNSCALE = "&width=640&height=480&quality=50"
PROFILES = {
    "home": {
        "rosbridge_url": "ws://192.168.0.109:9090",
        "video_url": "http://192.168.0.109:8080/stream?topic=/usb_cam/image_raw&type=mjpeg" + _DOWNSCALE,
    },
    "hotspot": {
        "rosbridge_url": "ws://192.168.1.11:9090",
        "video_url": "http://192.168.1.11:8080/stream?topic=/usb_cam/image_raw&type=mjpeg" + _DOWNSCALE,
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
GIMBAL_TILT = AxisConfig(servo_id=2, min_deg=0, max_deg=115, home_deg=22,
                         sign=-1, kp_deg=20.0)  # max upward tilt per operator: 115

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

# ---- Person following (stage 2) ---------------------------------------------
# Caps MIRROR robot/cmd_vel_mux.py — the mux is the enforcement point, these
# are defense in depth. angular_sign/setpoints are starting values; the live
# tuning session adjusts them (same process as the gimbal tracker's).
FOLLOW = {
    "target_class": "person",   # any COCO class: "person", "dog", ...
    "score_threshold": 0.5,
    "input_size": 640,          # YOLO letterbox size (drop to 416 if <10 fps)
    "control_rate_hz": 10.0,
    "status_rate_hz": 2.0,
    # tracker
    "min_iou": 0.2,
    "lost_grace_s": 0.5,
    # controller — angular: P on horizontal offset (err_x in [-0.5, 0.5])
    # LIVE-TUNED 2026-06-11: kp 2.0 (and 1.0) limit-cycled — with the ~0.8 s
    # blind stream loop the chassis overshoots any deadband it can't see in
    # time. kp 0.6 + deadband 0.10 (~6.5 deg) lets the P ramp-down land
    # inside the band. Move-and-settle is the next rung if this still hunts.
    "kp_ang": 0.6,              # rad/s per unit err_x
    "deadband_x": 0.10,         # |err_x| below this is "centered"
    "angular_sign": 1,          # flip to -1 if the robot turns AWAY (live check)
    # controller — linear: P on bbox-height fraction vs setpoint
    # LIVE-TUNED 2026-06-11: operator wants ~1 m follow distance; 0.8 holds
    # roughly that indoors (0.55 was ~1.5-2 m). Note the bbox clips at frame
    # edges this close, so distance authority softens near the setpoint.
    "height_setpoint": 0.8,     # bbox h / frame h at the desired follow distance
    "deadband_h": 0.05,
    "kp_lin": 1.2,              # m/s per unit height error
    "smoothing": 0.5,           # EMA weight on previous (cx, h) estimate
    # caps (mirror the mux) + blind-reverse limit
    "cap_fwd": 0.25, "cap_rev": 0.12, "cap_ang": 1.2,
    "reverse_limit_s": 1.5,     # max continuous blind reverse, then hold
    # Follow pose: the gimbal tracker's HOME while following (and the park
    # pose under --fixed-gimbal). Same values as the axis-config home today,
    # but sourced here so they can diverge.
    "gimbal_follow": {"pan": 90, "tilt": 22},
    # Bearing fusion (spec 2026-06-11): chassis steers on total bearing =
    # image err_x + pan_sign * pan_offset / deg_per_errx. deg_per_errx
    # converts servo degrees to err_x units: measured pan calibration is
    # 48.5 deg per half-frame error unit (range +/-1); err_x spans +/-0.5,
    # so 2 x 48.5 = 97 deg per err_x unit.
    "deg_per_errx": 97.0,
    "pan_sign": GIMBAL_PAN.sign,
    # GimbalTracker params while following — stage-1 move-and-settle values
    # (TRACKER block above), rehomed to the follow pose by the runner.
    # (no kp here: per-axis gains live on the AxisConfigs above)
    "gimbal_track": {
        "kd_deg": 0.0,
        "deadband": 0.06,
        "max_step_deg": 20.0,
        "smoothing": 0.5,
        "settle_updates": 11,        # hold fire ~1.1 s after each move
        "lost_recenter_after": 50,   # ~5 s lost at 10 Hz -> back to follow pose
        # Aim point as a fraction of the bbox from its TOP (0.5 = center).
        # A close person's bbox clips at the frame edges so its center stays
        # mid-frame and tilt never engages; 0.2 aims at the face/upper chest.
        "aim_y": 0.2,
    },
}
