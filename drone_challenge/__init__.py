"""Drone Challenge: send your position estimate, mark the flight events. That is all.

    from drone_challenge import Drone

    with Drone() as drone:
        drone.takeoff()
        while flying:
            drone.pose(x, y, z, yaw, timestamp)
            if gnss_disabled:
                drone.switch()

Access comes from ``drone-challenge pair`` (once per session) or the DRONE_LIVE_*
environment variables; without either, the same code records locally only.
Recording, upload in the background, retries, clock check and the files are handled
inside (see ``diha_participant`` for the advanced API).
"""

from __future__ import annotations

from diha_participant.live import DroneLive

from . import pairing
from .course import Course

__all__ = ["Course", "Drone", "__version__"]
__version__ = "0.6.0"


class Drone(DroneLive):
    """The participant interface: ``pose()``, ``takeoff()``, ``switch()``, ``stop()``.

    ``pose(x, y, z, yaw, timestamp)``: metres and radians in the hall frame; ``timestamp``
    (Unix seconds) is the time the estimate refers to and defaults to now. Never blocks,
    never raises; returns False for a rejected sample.
    """

    def __init__(self, course: str | None = None, trial_id: str | None = None, **options) -> None:
        """``course``: create a new session for this released course first (needs
        ``drone-challenge login``); otherwise use the current pairing or the environment."""
        if course:
            from . import team

            team.new_session(course, trial_id)
            env, stored = None, pairing.load()  # the session just created wins
        else:
            env = pairing.env_access()
            stored = None if env else pairing.load()
        source = env or stored or {}
        defaults = {}
        if stored:
            defaults = {
                "frame_id": stored.get("frame") or "hall",
                "team_id": stored.get("team_id") or "team",
                "trial_id": stored.get("trial_id"),
            }
        super().__init__(
            source.get("api"), source.get("session"), source.get("token"), **{**defaults, **options}
        )

    def course(self) -> Course | None:
        """The session's course: ``.waypoints`` / ``.route`` relative to the start pose,
        ``.hall_waypoints()`` once the measured start is bound. None when offline."""
        if self.client is None:
            return None
        return Course(self.client.course())

    def pose(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float = 0.0,
        timestamp: float | None = None,
        *,
        stamp: float | None = None,
    ) -> bool:
        return super().pose(x, y, z, yaw, stamp=timestamp if timestamp is not None else stamp)
