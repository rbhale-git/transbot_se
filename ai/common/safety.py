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


class ServoPacer:
    """Paces /PWMServo sends ACROSS calls — the driver drops back-to-back
    commands, and a silently dropped command desyncs tracker state from the
    physical servo (which bearing fusion then propagates to the chassis)."""

    def __init__(self, min_interval_s, clock=time.monotonic, sleep=time.sleep):
        self._min_interval_s = min_interval_s
        self._clock = clock
        self._sleep = sleep
        self._last_send = None

    def send(self, sink, commands):
        for servo_id, angle in commands:
            if self._last_send is not None:
                wait = self._min_interval_s - (self._clock() - self._last_send)
                if wait > 0:
                    self._sleep(wait)
            sink.send(servo_id, angle)
            self._last_send = self._clock()
