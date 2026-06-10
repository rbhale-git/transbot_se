"""Safety primitives shared by every AI behavior.

Rule (same as the dashboard): every value leaving the laptop is clamped
first, and outbound command bursts are rate-limited.
"""

import time


def clamp(value, lo, hi):
    """Clamp value to [lo, hi]."""
    return min(hi, max(lo, value))


class RateLimiter:
    """Allows an action at most once per `min_interval_s`.

    ready() returns True and starts a new window only when the interval has
    elapsed since the last *allowed* call; blocked calls don't reset it.
    `clock` is injectable for tests.
    """

    def __init__(self, min_interval_s, clock=time.monotonic):
        self._min_interval_s = min_interval_s
        self._clock = clock
        self._last_allowed = None

    def ready(self):
        now = self._clock()
        if self._last_allowed is not None and now - self._last_allowed < self._min_interval_s:
            return False
        self._last_allowed = now
        return True
