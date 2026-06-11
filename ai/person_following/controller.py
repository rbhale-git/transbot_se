"""Follow controller: bbox -> (linear, angular) chassis command.

Angular: P on the target's horizontal offset from frame center (the gimbal
is fixed during following — the chassis does the turning). Linear: P on
bbox-height fraction vs the follow-distance setpoint; height is the distance
proxy (bigger box = closer). Both have deadbands so a centered, at-distance
target commands exact zeros (no idle creep).

Safety (spec): outputs clamped to the same caps the mux enforces (defense in
depth); reverse is additionally time-limited — the robot has no rear
sensors, so after reverse_limit_s of continuous backing it holds still until
the demand goes non-negative (target stepped back / moved away).

Latency note: the command->stream loop is ~0.8 s blind (measured, stage 1).
Gains here start LOW; if the chassis limit-cycles live, the tuning ladder is
EMA smoothing -> lower gains -> stage-1 move-and-settle pattern.
"""


class FollowController:
    def __init__(self, cfg):
        self._cfg = cfg
        self._ema = None            # smoothed (cx_px, h_px)
        self._reverse_since = None  # when continuous reverse began

    def reset(self):
        self._ema = None
        self._reverse_since = None

    def update(self, target, frame_size, now):
        """target: Detection or None. Returns (linear m/s, angular rad/s)."""
        cfg = self._cfg
        if target is None:
            self.reset()
            return (0.0, 0.0)

        w, h = frame_size
        cx, bh = target.center[0], target.h
        a = cfg["smoothing"]
        if self._ema is not None and a > 0:
            cx = a * self._ema[0] + (1 - a) * cx
            bh = a * self._ema[1] + (1 - a) * bh
        self._ema = (cx, bh)

        # Angular: err_x in [-0.5, 0.5]; target right of center => negative z
        # (clockwise) for a forward-facing camera. angular_sign flips it if
        # the live check disagrees.
        err_x = cx / w - 0.5
        ang = 0.0
        if abs(err_x) >= cfg["deadband_x"]:
            ang = -cfg["kp_ang"] * err_x * cfg["angular_sign"]
        ang = min(cfg["cap_ang"], max(-cfg["cap_ang"], ang))

        # Linear: positive height error = too far = drive forward.
        err_h = cfg["height_setpoint"] - bh / h
        lin = 0.0
        if abs(err_h) >= cfg["deadband_h"]:
            lin = cfg["kp_lin"] * err_h
        lin = min(cfg["cap_fwd"], max(-cfg["cap_rev"], lin))

        # Blind-reverse time limit.
        if lin < 0:
            if self._reverse_since is None:
                self._reverse_since = now
            elif now - self._reverse_since > cfg["reverse_limit_s"]:
                lin = 0.0   # hold; _reverse_since stays set until demand >= 0
        else:
            self._reverse_since = None

        return (lin, ang)
