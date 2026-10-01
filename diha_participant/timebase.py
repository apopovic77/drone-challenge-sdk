"""A wall-clock time base without steps, for streams this package stamps itself.

The system clock can be stepped while a flight runs (an NTP correction): in the hall test
of 30 Sep 2026 it jumped back by about 0.27 s mid-flight. Stamps taken from it then run
backwards, the forwarder must drop them, and the reference loses data exactly when it
matters. This clock is the wall time at start plus monotonic elapsed time: it never steps.
Clock measurements taken with the same clock stay consistent with the stamps, so the
server's clock model sees a smooth offset instead of a step.
"""

from __future__ import annotations

import time


class StableClock:
    def __init__(self) -> None:
        self._wall0 = time.time_ns()
        self._mono0 = time.monotonic_ns()

    def time_ns(self) -> int:
        return self._wall0 + (time.monotonic_ns() - self._mono0)

    def time(self) -> float:
        return self.time_ns() / 1e9
