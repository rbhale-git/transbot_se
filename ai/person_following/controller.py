"""Follow controller: bbox -> (linear, angular) chassis command.

Angular: P on the target's total bearing — image offset plus the gimbal's
pan offset from its follow-pose home. The sum is invariant to gimbal moves
(a pan toward the person shrinks the image error by exactly what the pan
offset grows), so the chassis loop is decoupled from the gimbal's settle
cycle. With the gimbal parked (--fixed-gimbal) steering reduces to plain P on the image offset; the linear term keeps its cos(image-bearing) scaling. Linear: P on
bbox-height fraction vs the follow-distance setpoint; height is the distance
proxy (bigger box = closer), scaled by cos(bearing) so the robot never drives hard while the person is far off-axis. Both have deadbands so a centered, at-distance
target commands exact zeros (no idle creep).

Safety (spec): outputs clamped to the same caps the mux enforces (defense in
depth); reverse is additionally time-limited — the robot has no rear
sensors, so after reverse_limit_s of continuous backing it holds still until
the demand goes non-negative (target stepped back / moved away).

Latency note: the command->stream loop is ~0.8 s blind (measured, stage 1).
Gains here start LOW; if the chassis limit-cycles live, the tuning ladder is
EMA smoothing -> lower gains -> stage-1 move-and-settle pattern.
"""

import math


class FollowController:
    def __init__(self, cfg):
        self._cfg = cfg
        self._ema = None            # smoothed (cx_px, h_px)
        self._reverse_since = None  # when continuous reverse began

    def reset(self):
        self._ema = None
        self._reverse_since = None

    def update(self, target, frame_size, now, pan_offset_deg=0.0):
        """target: Detection or None. pan_offset_deg: gimbal pan minus its
        follow-pose home (0 when the gimbal is parked).
        Returns (linear m/s, angular rad/s)."""
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

        # Angular: err in [-0.5, 0.5]-ish units; target right of center =>
        # negative z (clockwise) for a forward-facing camera. pan_sign maps
        # the servo offset back into image-error convention (pan sign -1:
        # person right => pan below home => negative offset => positive err).
        # angular_sign flips the output if the live check disagrees.
        err_x = cx / w - 0.5
        err_total = err_x + cfg["pan_sign"] * pan_offset_deg / cfg["deg_per_errx"]
        ang = 0.0
        if abs(err_total) >= cfg["deadband_x"]:
            ang = -cfg["kp_ang"] * err_total * cfg["angular_sign"]
        ang = min(cfg["cap_ang"], max(-cfg["cap_ang"], ang))

        # Linear: positive height error = too far = drive forward. Scaled by
        # cos(bearing), floored at 0 — never drive forward/reverse at speed
        # while the person stands far off-axis (the robot has no side
        # sensors and isn't looking where it's going).
        err_h = cfg["height_setpoint"] - bh / h
        lin = 0.0
        if abs(err_h) >= cfg["deadband_h"]:
            lin = cfg["kp_lin"] * err_h
        lin *= max(0.0, math.cos(math.radians(err_total * cfg["deg_per_errx"])))
        lin = min(cfg["cap_fwd"], max(-cfg["cap_rev"], lin))
        lin = lin if lin else 0.0   # cos floor can leave -0.0; don't publish it

        # Blind-reverse time limit.
        if lin < 0:
            if self._reverse_since is None:
                self._reverse_since = now
            elif now - self._reverse_since > cfg["reverse_limit_s"]:
                lin = 0.0   # hold; _reverse_since stays set until demand >= 0
        else:
            self._reverse_since = None

        return (lin, ang)
