"""Tests for ai.common.safety — clamping and command rate limiting.

Every value leaving the laptop goes through these; they are the AI-side
equivalent of the dashboard's clamp-before-publish rule.
"""

from ai.common.safety import RateLimiter, clamp


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
