"""FollowController: P-control with caps, deadbands, and the reverse limit."""

from ai.common.detection import Detection
from ai.person_following.controller import FollowController

FRAME = (1280, 720)


def cfg(**over):
    base = dict(kp_ang=2.0, deadband_x=0.05, angular_sign=1,
                height_setpoint=0.55, deadband_h=0.05, kp_lin=1.2,
                smoothing=0.0,   # EMA off in unit tests: pure P response
                cap_fwd=0.25, cap_rev=0.12, cap_ang=1.2, reverse_limit_s=1.5)
    base.update(over)
    return base


def target(cx_frac, h_frac):
    """A Detection centered at cx_frac of frame width with given height."""
    w, h = FRAME
    bw, bh = 100.0, h_frac * h
    return Detection(x=cx_frac * w - bw / 2, y=100.0, w=bw, h=bh,
                     score=0.9, label="person")


def test_centered_at_distance_is_still():
    c = FollowController(cfg())
    assert c.update(target(0.5, 0.55), FRAME, now=0.0) == (0.0, 0.0)


def test_no_target_is_zeros():
    c = FollowController(cfg())
    assert c.update(None, FRAME, now=0.0) == (0.0, 0.0)


def test_target_right_turns_right():
    c = FollowController(cfg())
    lin, ang = c.update(target(0.8, 0.55), FRAME, now=0.0)
    assert ang < 0           # right of center => clockwise (negative z)
    lin, ang = c.update(target(0.2, 0.55), FRAME, now=0.1)
    assert ang > 0


def test_angular_sign_flip():
    c = FollowController(cfg(angular_sign=-1))
    _, ang = c.update(target(0.8, 0.55), FRAME, now=0.0)
    assert ang > 0


def test_far_drives_forward_close_reverses_capped():
    c = FollowController(cfg())
    lin, _ = c.update(target(0.5, 0.20), FRAME, now=0.0)   # small box = far
    assert 0 < lin <= 0.25
    lin, _ = c.update(target(0.5, 0.95), FRAME, now=0.1)   # huge box = close
    assert -0.12 <= lin < 0


def test_reverse_time_limited_then_recovers():
    c = FollowController(cfg())
    too_close = target(0.5, 0.95)
    assert c.update(too_close, FRAME, now=0.0)[0] < 0
    assert c.update(too_close, FRAME, now=1.0)[0] < 0      # still within 1.5s
    assert c.update(too_close, FRAME, now=2.0)[0] == 0.0   # limit hit: hold
    assert c.update(too_close, FRAME, now=3.0)[0] == 0.0   # still holding
    far = target(0.5, 0.20)
    assert c.update(far, FRAME, now=4.0)[0] > 0            # forward resets it
    assert c.update(too_close, FRAME, now=5.0)[0] < 0      # reverse allowed again


def test_deadbands():
    c = FollowController(cfg())
    lin, ang = c.update(target(0.52, 0.57), FRAME, now=0.0)  # inside both bands
    assert (lin, ang) == (0.0, 0.0)
