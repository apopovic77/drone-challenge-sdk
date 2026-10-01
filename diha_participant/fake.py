"""Stand-in for a team estimate when no real drone software is flying.

Used for hall tests with a hand-carried marker body: every reference pose is copied
into a "team estimate" with a slow drift and a little noise, so the whole chain
(two streams, live view, evaluation) can be exercised. It is clearly a fake and is
never meant for scoring.
"""

from __future__ import annotations

import random

from .logger import Event, Pose
from .uploader import TelemetryUploader


class FakeTeam:
    def __init__(
        self,
        uploader: TelemetryUploader,
        *,
        drift_m_per_s: float = 0.003,
        noise_m: float = 0.005,
        switch_after_s: float = 5.0,
        seed: int = 1,
    ) -> None:
        self.uploader = uploader
        self.drift = drift_m_per_s
        self.noise = noise_m
        self.switch_after_s = switch_after_s
        self.random = random.Random(seed)
        self.start: float | None = None
        self.switched = False
        self.last: float | None = None

    def __call__(self, pose: Pose) -> None:
        if self.start is None:
            self.start = pose.stamp_s
            self.uploader.record_event(Event("system_start", pose.stamp_s))
        elapsed = pose.stamp_s - self.start
        if not self.switched and elapsed >= self.switch_after_s:
            self.switched = True
            self.uploader.record_event(Event("switch", pose.stamp_s))
        # Drift only after the switch, like an estimate that lost its external correction.
        self.last = pose.stamp_s
        drift = self.drift * max(elapsed - self.switch_after_s, 0.0)
        g = self.random.gauss
        self.uploader.publish(
            Pose(
                pose.stamp_s,
                pose.x + drift + g(0, self.noise),
                pose.y - 0.5 * drift + g(0, self.noise),
                pose.z + g(0, self.noise),
                pose.yaw,
            )
        )

    def finish(self) -> None:
        """Flight end at the last copied pose (called when the forwarder stops)."""
        if self.last is not None:
            self.uploader.record_event(Event("flight_end", self.last))
