"""TargetTracker: lock-at-arm / stop-on-loss state machine (no re-ID)."""

from ai.common.detection import Detection
from ai.person_following.tracker import TargetTracker, iou


def box(x, y, w=100, h=200, label="person"):
    return Detection(x=x, y=y, w=w, h=h, score=0.9, label=label)


def test_iou_identical_and_disjoint():
    assert iou(box(0, 0), box(0, 0)) == 1.0
    assert iou(box(0, 0), box(500, 500)) == 0.0


def test_locks_largest_on_first_sighting():
    t = TargetTracker()
    small, big = box(0, 0, w=50, h=100), box(300, 0, w=120, h=240)
    assert t.update([small, big], now=0.0) is big
    assert t.state == "FOLLOWING"


def test_sticks_to_locked_target_not_largest():
    t = TargetTracker()
    locked = box(100, 100)
    t.update([locked], now=0.0)
    moved = box(120, 105)                       # same person, slight motion
    intruder = box(600, 100, w=200, h=400)      # bigger box, elsewhere
    assert t.update([intruder, moved], now=0.1) is moved


def test_no_match_returns_none_within_grace():
    t = TargetTracker(lost_grace_s=0.5)
    t.update([box(100, 100)], now=0.0)
    assert t.update([], now=0.2) is None        # stopped, still FOLLOWING
    assert t.state == "FOLLOWING"


def test_lost_after_grace_then_relocks_largest():
    t = TargetTracker(lost_grace_s=0.5)
    t.update([box(100, 100)], now=0.0)
    t.update([], now=0.6)
    assert t.state == "LOST"
    newcomer = box(400, 50, w=80, h=160)
    assert t.update([newcomer], now=1.0) is newcomer
    assert t.state == "FOLLOWING"


def test_reappearance_within_grace_reassociates():
    t = TargetTracker(lost_grace_s=0.5)
    t.update([box(100, 100)], now=0.0)
    t.update([], now=0.2)                       # one missed frame
    back = box(110, 102)
    assert t.update([back], now=0.3) is back
