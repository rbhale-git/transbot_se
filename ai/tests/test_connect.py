"""Tests for ai.common.connect — the two-phase actuation preflight ladder.

Everything is driven through fakes: a FakeSink whose rosapi answers are
scripted, a FakeSrc whose frames either change on command (wiggle passes)
or never change (wiggle fails), and a FakeHeal that records calls. The
real rosapi/wiggle paths are exercised live, per suite convention.
"""

import types

import numpy as np
import pytest

from ai.common.connect import (
    REGISTRATION_TOPICS, connect_with_actuation_check, registration_failures,
)

FRAME_A = np.zeros((48, 64, 3), dtype=np.uint8)
FRAME_B = np.full((48, 64, 3), 200, dtype=np.uint8)
TILT = types.SimpleNamespace(servo_id=2, home_deg=22)
NOSLEEP = lambda s: None  # noqa: E731

ALL_OK = {t: ["/rosbridge_websocket"] for t in REGISTRATION_TOPICS}
DEAD_CMD_VEL = {**ALL_OK, "/ai/cmd_vel": []}


class FakeSink:
    """publishers_by_topic=None scripts a silent rosapi (raises)."""

    def __init__(self, publishers_by_topic):
        self._pubs = publishers_by_topic
        self.sent = []
        self.disconnected = False

    def publishers_of(self, topic, timeout_s=3.0):
        if self._pubs is None:
            raise RuntimeError("rosapi silent")
        return self._pubs.get(topic, [])

    def send(self, servo_id, angle):
        self.sent.append((servo_id, angle))

    def disconnect(self):
        self.disconnected = True


class FakeSrc:
    """Alternates frames when `moving` (wiggle check passes), repeats one
    frame when not (wiggle check fails)."""

    def __init__(self, moving=True):
        self.moving = moving
        self.reopened = 0
        self._flip = False

    def read(self, timeout_s=0.0):
        self._flip = not self._flip
        return FRAME_B if (self.moving and self._flip) else FRAME_A

    def reopen(self, timeout_s=60.0):
        self.reopened += 1


class FakeHeal:
    def __init__(self, ok=True):
        self.ok = ok
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return self.ok, "fake heal detail"


def factory_of(*sinks):
    it = iter(sinks)
    return lambda: next(it)


class TestRegistrationFailures:
    def test_all_registered(self):
        assert registration_failures(FakeSink(ALL_OK)) == []

    def test_names_dead_topics(self):
        assert registration_failures(FakeSink(DEAD_CMD_VEL)) == ["/ai/cmd_vel"]

    def test_silent_rosapi_is_unverifiable_not_fault(self):
        assert registration_failures(FakeSink(None)) is None


class TestLadder:
    def test_happy_path_first_attempt(self):
        sink = FakeSink(ALL_OK)
        heal = FakeHeal()
        got = connect_with_actuation_check(
            factory_of(sink), FakeSrc(), TILT, heal=heal, sleep=NOSLEEP)
        assert got is sink
        assert not sink.disconnected
        assert heal.calls == 0

    def test_dead_topic_fresh_sessions_then_heal_then_success(self):
        bad = [FakeSink(DEAD_CMD_VEL) for _ in range(3)]
        good = FakeSink(ALL_OK)
        heal = FakeHeal()
        src = FakeSrc()
        got = connect_with_actuation_check(
            factory_of(*bad, good), src, TILT, attempts=3, heal=heal,
            sleep=NOSLEEP)
        assert got is good
        assert heal.calls == 1
        assert src.reopened == 1
        assert all(s.disconnected for s in bad)

    def test_without_heal_raises_and_names_dead_topics(self):
        sinks = [FakeSink(DEAD_CMD_VEL) for _ in range(3)]
        with pytest.raises(RuntimeError, match="/ai/cmd_vel"):
            connect_with_actuation_check(
                factory_of(*sinks), FakeSrc(), TILT, attempts=3,
                sleep=NOSLEEP)

    def test_silent_rosapi_falls_back_to_wiggle(self):
        sink = FakeSink(None)  # unverifiable, but the wiggle passes
        got = connect_with_actuation_check(
            factory_of(sink), FakeSrc(moving=True), TILT, sleep=NOSLEEP)
        assert got is sink

    def test_wiggle_failure_spends_heal_then_aborts(self):
        sinks = [FakeSink(ALL_OK) for _ in range(4)]
        heal = FakeHeal()
        src = FakeSrc(moving=False)
        with pytest.raises(RuntimeError, match="service restart"):
            connect_with_actuation_check(
                factory_of(*sinks), src, TILT, attempts=3, heal=heal,
                sleep=NOSLEEP)
        assert heal.calls == 1          # never more than one heal per run
        assert src.reopened == 1

    def test_failed_heal_aborts_with_detail(self):
        sinks = [FakeSink(DEAD_CMD_VEL) for _ in range(3)]
        with pytest.raises(RuntimeError, match="fake heal detail"):
            connect_with_actuation_check(
                factory_of(*sinks), FakeSrc(), TILT, attempts=3,
                heal=FakeHeal(ok=False), sleep=NOSLEEP)

    def test_post_heal_wiggle_failure_reports_last_symptom(self):
        # Registration heals, but the wiggle then fails: the abort must
        # report the LAST symptom plus the fact a restart was already tried.
        bad = [FakeSink(DEAD_CMD_VEL) for _ in range(3)]
        good_reg = FakeSink(ALL_OK)
        heal = FakeHeal()
        src = FakeSrc(moving=False)  # wiggle never passes
        with pytest.raises(RuntimeError) as err:
            connect_with_actuation_check(
                factory_of(*bad, good_reg), src, TILT, attempts=3,
                heal=heal, sleep=NOSLEEP)
        assert "tilt wiggle" in str(err.value)
        assert "service restart" in str(err.value)
        assert heal.calls == 1

    def test_silent_rosapi_with_dead_wiggle_still_aborts(self):
        sinks = [FakeSink(None) for _ in range(3)]
        with pytest.raises(RuntimeError, match="tilt wiggle"):
            connect_with_actuation_check(
                factory_of(*sinks), FakeSrc(moving=False), TILT, attempts=3,
                sleep=NOSLEEP)
