"""Pure P-control gimbal tracker — no hardware, no I/O.

update() maps a face center (or None) to a list of (servo_id, int_angle)
commands. Internal angle state is float so sub-degree corrections accumulate;
a command is emitted only when the rounded angle actually changes, so a
settled tracker is silent on the wire.

Safety envelope, in order: deadband (ignore small errors), per-update step
cap (no servo slewing), range clamp, hold position on lost target.
"""

from dataclasses import dataclass

from ai.common.safety import clamp


@dataclass
class AxisConfig:
    servo_id: int
    min_deg: float
    max_deg: float
    home_deg: float
    # Degrees of servo travel per unit of positive normalized image error.
    # -1 on both axes of the real robot: positive servo angle moves the view
    # toward negative image error (verified on hardware; dashboard config).
    sign: int
    # Per-axis gain override (deg per unit error). The camera's horizontal
    # and vertical FOV differ, so the axes need different full-correction
    # gains — measured live: pan ~48, tilt ~23. None = tracker's global kp.
    kp_deg: float = None


class _Axis:
    def __init__(self, cfg):
        self.cfg = cfg
        self.angle = float(cfg.home_deg)
        self.last_sent = round(self.angle)
        self.prev_error = None

    def step(self, error, kp_deg, kd_deg, deadband, max_step_deg):
        """Apply one PD-control step; return a command tuple or None.

        The D term damps the step while the error is already shrinking —
        without it, camera-stream latency makes the gimbal overshoot the
        face and lose it (observed on the first live run).
        """
        derr = 0.0 if self.prev_error is None else error - self.prev_error
        self.prev_error = error
        if abs(error) < deadband:
            return None
        kp = self.cfg.kp_deg if self.cfg.kp_deg is not None else kp_deg
        raw = self.cfg.sign * (kp * error + kd_deg * derr)
        delta = clamp(raw, -max_step_deg, max_step_deg)
        self.angle = clamp(self.angle + delta, self.cfg.min_deg, self.cfg.max_deg)
        target = round(self.angle)
        if target == self.last_sent:
            return None
        self.last_sent = target
        return (self.cfg.servo_id, target)


class GimbalTracker:
    def __init__(self, pan, tilt, kp_deg, deadband, max_step_deg,
                 kd_deg=0.0, lost_recenter_after=0, smoothing=0.0,
                 settle_updates=0):
        self._pan = _Axis(pan)
        self._tilt = _Axis(tilt)
        self._kp_deg = kp_deg
        self._kd_deg = kd_deg
        self._deadband = deadband
        self._max_step_deg = max_step_deg
        # After this many consecutive lost-target updates, drive back to home
        # once (recovery from overshoot-and-lose). 0 disables: hold forever.
        self._lost_recenter_after = lost_recenter_after
        # EMA on the face center: weight kept on the previous estimate.
        # Detector bbox wobble otherwise drives visible servo jitter.
        self._smoothing = smoothing
        self._smoothed_center = None
        # Move-and-settle mode (>0): after emitting a move, hold fire for
        # this many updates so the laggy camera stream catches up, then
        # measure fresh. Continuous control through ~0.4 s of stream latency
        # limit-cycles around the target (observed live); discrete
        # measure->move->wait cannot. In this mode kp_deg is the
        # full-correction gain (~degrees of half-FOV per unit error).
        self._settle_updates = settle_updates
        self._settle_remaining = 0
        self.frames_since_face = 0

    @property
    def pan_deg(self):
        return self._pan.angle

    @property
    def tilt_deg(self):
        return self._tilt.angle

    @property
    def settling(self):
        """True while holding fire for the stream to catch up after a move."""
        return self._settle_remaining > 0

    def start_commands(self):
        """Sync commands sent once at startup: drive both axes to home.

        The driver does not echo servo positions, so commanding home is the
        only way to make tracker state match reality.
        """
        return self.recenter()

    def recenter(self):
        """Reset both axes to home and return the commands to send."""
        commands = []
        for axis in (self._pan, self._tilt):
            axis.angle = float(axis.cfg.home_deg)
            axis.last_sent = round(axis.angle)
            axis.prev_error = None
            commands.append((axis.cfg.servo_id, axis.last_sent))
        self._smoothed_center = None
        # A recenter IS a move: hold fire while the laggy stream catches up,
        # or the first post-recenter update corrects against a frame still
        # showing the old pose.
        self._settle_remaining = self._settle_updates
        return commands

    def update(self, face_center, frame_size):
        """One control step. face_center=(x,y) px or None; frame_size=(w,h)."""
        if face_center is None:
            self.frames_since_face += 1
            self._pan.prev_error = None   # stale errors must not D-kick
            self._tilt.prev_error = None  # on reacquisition
            self._smoothed_center = None
            # Do NOT clear _settle_remaining here: recenter() starts a settle
            # window during a lost streak, and that window must survive into
            # reacquisition so the first post-recenter frame is not measured
            # against the old gimbal pose.
            if self.frames_since_face == self._lost_recenter_after:
                return self.recenter()
            return []  # lost target: hold position
        self.frames_since_face = 0

        if self._settle_remaining > 0:
            self._settle_remaining -= 1
            return []  # camera still catching up with the last move

        if self._smoothing > 0 and self._smoothed_center is not None:
            a = self._smoothing
            face_center = (
                a * self._smoothed_center[0] + (1 - a) * face_center[0],
                a * self._smoothed_center[1] + (1 - a) * face_center[1],
            )
        self._smoothed_center = face_center

        w, h = frame_size
        ex = (face_center[0] - w / 2.0) / (w / 2.0)
        ey = (face_center[1] - h / 2.0) / (h / 2.0)

        commands = []
        for axis, error in ((self._pan, ex), (self._tilt, ey)):
            cmd = axis.step(error, self._kp_deg, self._kd_deg,
                            self._deadband, self._max_step_deg)
            if cmd:
                commands.append(cmd)
        if commands and self._settle_updates > 0:
            self._settle_remaining = self._settle_updates
            # History describes face position in the OLD camera pose;
            # measure raw once the view has shifted.
            self._smoothed_center = None
        return commands
