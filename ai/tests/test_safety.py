"""Tests for ai.common.safety — clamping and command rate limiting.

Every value leaving the laptop goes through these; they are the AI-side
equivalent of the dashboard's clamp-before-publish rule.
"""

from ai.common.safety import RateLimiter, ServoPacer, clamp


class TestClamp:
    def test_passes_value_inside_range(self):
        assert clamp(90, 0, 180) == 90

    def test_clamps_below_min(self):
        assert clamp(-5, 0, 180) == 0

    def test_clamps_above_max(self):
        assert clamp(200, 0, 180) == 180


class TestRateLimiter:
    def test_first_call_is_allowed(self):
        rl = RateLimiter(min_interval_s=0.1, clock=lambda: 100.0)
        assert rl.ready() is True

    def test_blocks_within_interval(self):
        now = [100.0]
        rl = RateLimiter(min_interval_s=0.1, clock=lambda: now[0])
        assert rl.ready() is True
        now[0] = 100.05
        assert rl.ready() is False

    def test_allows_after_interval(self):
        now = [100.0]
        rl = RateLimiter(min_interval_s=0.1, clock=lambda: now[0])
        assert rl.ready() is True
        now[0] = 100.11
        assert rl.ready() is True

    def test_blocked_call_does_not_reset_window(self):
        now = [100.0]
        rl = RateLimiter(min_interval_s=0.1, clock=lambda: now[0])
        assert rl.ready() is True
        now[0] = 100.09
        assert rl.ready() is False
        now[0] = 100.11  # 0.11s after the *allowed* call, not the blocked one
        assert rl.ready() is True


class _FakeSink:
    """Records (servo_id, angle) pairs for assertion."""
    def __init__(self):
        self.sent = []

    def send(self, servo_id, angle):
        self.sent.append((servo_id, angle))


def _make_pacer(start_t=100.0):
    """Return (pacer, now_list, sleeps_list) using an injectable fake clock."""
    now = [start_t]
    sleeps = []
    pacer = ServoPacer(
        min_interval_s=0.15,
        clock=lambda: now[0],
        sleep=lambda s: (sleeps.append(s), now.__setitem__(0, now[0] + s)),
    )
    return pacer, now, sleeps


class TestServoPacer:
    def test_first_send_has_no_sleep(self):
        pacer, now, sleeps = _make_pacer()
        sink = _FakeSink()
        pacer.send(sink, [(1, 90)])
        assert sleeps == []
        assert sink.sent == [(1, 90)]

    def test_two_commands_in_one_call_sleeps_between_them(self):
        pacer, now, sleeps = _make_pacer()
        sink = _FakeSink()
        pacer.send(sink, [(1, 90), (2, 45)])
        # First command: no sleep. Second: must wait the full 0.15 s because
        # clock has not advanced (both commands sent "instantly").
        assert len(sleeps) == 1
        assert abs(sleeps[0] - 0.15) < 1e-9
        assert sink.sent == [(1, 90), (2, 45)]

    def test_second_call_after_short_gap_sleeps_remainder(self):
        pacer, now, sleeps = _make_pacer()
        sink = _FakeSink()
        pacer.send(sink, [(1, 90)])
        now[0] += 0.05          # only 0.05 s has elapsed
        pacer.send(sink, [(2, 45)])
        assert len(sleeps) == 1
        assert abs(sleeps[0] - 0.10) < 1e-9

    def test_second_call_after_full_interval_has_no_sleep(self):
        pacer, now, sleeps = _make_pacer()
        sink = _FakeSink()
        pacer.send(sink, [(1, 90)])
        now[0] += 0.20          # more than 0.15 s has elapsed
        pacer.send(sink, [(2, 45)])
        assert sleeps == []

    def test_empty_command_list_sends_nothing_and_no_sleep(self):
        pacer, now, sleeps = _make_pacer()
        sink = _FakeSink()
        pacer.send(sink, [])
        assert sleeps == []
        assert sink.sent == []
