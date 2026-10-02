"""Drone Challenge: send your position estimate, mark the flight events. That is all.

    from drone_challenge import Drone

    with Drone() as drone:
        drone.takeoff()
        while flying:
            drone.pose(x, y, z, yaw, timestamp)
            if gnss_disabled:
                drone.switch()

After ``drone-challenge login`` every ``Drone()`` creates its own session for the active
challenge course the organiser set (``Drone(course=...)`` names another released course).
Otherwise access comes from ``drone-challenge pair`` or the DRONE_LIVE_* environment
variables; without any of them, the same code records locally only.
Recording, upload in the background, retries, clock check and the files are handled
inside (see ``diha_participant`` for the advanced API).
"""

from __future__ import annotations

import warnings

from diha_participant.live import DroneLive

from . import pairing
from .course import Course

__all__ = ["Course", "Drone", "__version__"]
__version__ = "0.7.0"


class Drone(DroneLive):
    """The participant interface: ``pose()``, ``takeoff()``, ``switch()``, ``stop()``.

    ``pose(x, y, z, yaw, timestamp)``: metres and radians in the hall frame; ``timestamp``
    (Unix seconds) is the time the estimate refers to and defaults to now. Never blocks,
    never raises; returns False for a rejected sample.
    """

    def __init__(
        self,
        course: str | None = None,
        trial_id: str | None = None,
        *,
        new_session: bool = True,
        **options,
    ) -> None:
        """``course``: create a new session for this released course first (needs
        ``drone-challenge login``). Without it a logged-in computer creates one for the
        active challenge course; ``new_session=False`` keeps the current pairing instead
        (e.g. a QR code from the organiser). Not logged in: the pairing, the environment,
        or offline recording."""
        from . import team

        env = None if course else pairing.env_access()
        if course or (new_session and not env and team.load()):
            try:
                team.new_session(course, trial_id)  # course None: the active challenge course
            except team.TeamError as exc:
                if course:
                    raise
                warnings.warn(
                    f"Keine Session für den Challenge-Kurs angelegt ({exc}); "
                    "es gilt die bisherige Kopplung, sonst nur lokale Aufzeichnung.",
                    stacklevel=2,
                )
        stored = None if env else pairing.load()  # a session just created wins
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
