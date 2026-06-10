"""MuxCore arbitration tests.

cmd_vel_mux.py runs on the robot under Python 2 / rospy, so its pure
arbitration core is loaded here by file path (robot/ is not a package) and
driven with explicit `now` timestamps - no rospy, no clocks.
"""

import importlib.util
from pathlib import Path

_PATH = Path(__file__).resolve().parents[2] / "robot" / "cmd_vel_mux.py"
_spec = importlib.util.spec_from_file_location("cmd_vel_mux", _PATH)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
MuxCore = _mod.MuxCore


def make():
    return MuxCore()  # defaults: window 1.0s, ai timeout 0.5s, caps 0.25/0.12/1.2


def test_manual_always_forwarded_unchanged():
    core = make()
    assert core.on_manual(0.45, -2.0, now=0.0) == (0.45, -2.0)
    core.armed = True  # arming must not affect manual
    assert core.on_manual(-0.3, 1.0, now=0.1) == (-0.3, 1.0)


def test_ai_blocked_when_disarmed():
    core = make()
    assert core.on_ai(0.1, 0.0, now=0.0) is None


def test_ai_blocked_during_manual_window():
    core = make()
    core.set_armed(True, now=0.0)
    core.on_manual(0.2, 0.0, now=1.0)
    assert core.on_ai(0.1, 0.0, now=1.5) is None      # 0.5s after manual: blocked
    assert core.on_ai(0.1, 0.0, now=2.1) == (0.1, 0.0)  # window expired: forwarded


def test_ai_clamped_to_caps():
    core = make()
    core.set_armed(True, now=0.0)
    assert core.on_ai(1.0, 5.0, now=0.0) == (0.25, 1.2)
    assert core.on_ai(-1.0, -5.0, now=0.1) == (-0.12, -1.2)


def test_disarm_zeroes_only_if_ai_was_driving():
    core = make()
    core.set_armed(True, now=0.0)
    assert core.set_armed(False, now=0.1) is False    # AI never drove: no zero
    core.set_armed(True, now=0.2)
    core.on_ai(0.1, 0.0, now=0.3)
    assert core.set_armed(False, now=0.4) is True     # AI was driving: halt
    assert core.on_ai(0.1, 0.0, now=0.5) is None      # and now disarmed


def test_manual_takeover_clears_ai_source():
    core = make()
    core.set_armed(True, now=0.0)
    core.on_ai(0.1, 0.0, now=0.0)
    assert core.status(now=0.1)["source"] == "ai"
    core.on_manual(0.2, 0.0, now=0.2)
    assert core.status(now=0.3)["source"] == "manual"
    # disarm right after a manual takeover must not zero (manual is driving)
    assert core.set_armed(False, now=0.4) is False


def test_ai_silence_timeout_fires_once():
    core = make()
    core.set_armed(True, now=0.0)
    core.on_ai(0.1, 0.0, now=0.0)
    assert core.check_timeout(now=0.3) is False   # still fresh
    assert core.check_timeout(now=0.6) is True    # >0.5s silent: zero once
    assert core.check_timeout(now=0.7) is False   # fired already


def test_status_shape():
    core = make()
    s = core.status(now=0.0)
    assert s == {
        "source": "none",
        "armed": False,
        "caps": {"fwd": 0.25, "rev": 0.12, "ang": 1.2},
    }
