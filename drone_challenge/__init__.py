"""Drone Challenge: send your position estimate, mark the flight events. That is all.

    from drone_challenge import Drone

    with Drone() as drone:
        drone.start()                 # already emitted by the context manager; idempotent
        drone.takeoff()
        while flying:
            drone.pose(x, y, z, yaw, timestamp)

Navigating relative to the start instead? ``drone.pose_relative(...)`` takes the start-
relative estimate and sends it in hall coordinates when the hall measured the start, else
in course coordinates (``wait_start()`` tells which). See ``startframe``.

After ``drone-challenge login`` every ``Drone()`` creates its own session for the active
challenge course the organiser set (``Drone(course=...)`` names another released course).
Otherwise access comes from ``drone-challenge pair`` or the DRONE_LIVE_* environment
variables; without any of them, the same code records locally only.
Recording, upload in the background, retries, clock check and the files are handled
inside (see ``diha_participant`` for the advanced API).
"""

from __future__ import annotations

import threading
import time
import warnings

from diha_participant.live import DroneLive

from . import pairing
from .course import Course
from .startframe import StartFrame, StartFrameError
from .vehicle import validate_size

__all__ = ["Course", "Drone", "StartFrameError", "__version__"]
__version__ = "0.8.1"


class Drone(DroneLive):
    """The participant interface: ``start()``, ``pose()``, ``takeoff()``, ``stop()``.

    ``switch()`` remains a recorded compatibility event, without changing live scoring.

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
        size_m: tuple[float, float, float] | None = None,
        simulation: bool = False,
        random: bool = False,
        seed: int | None = None,
        obstacles: int | None = None,
        targets: int | None = None,
        **options,
    ) -> None:
        """``course``: create a new session for this released course first (needs
        ``drone-challenge login``). Without it a logged-in computer creates one for the
        active challenge course; ``new_session=False`` keeps the current pairing instead
        (e.g. a QR code from the organiser). Not logged in: the pairing, the environment,
        or offline recording.

        Advanced training: ``random=True`` builds a fresh obstacle course from the course's
        hall and vehicle (``seed`` repeats one, ``obstacles`` boxes, ``targets`` moving
        targets 0-3); start and goal are given, the team plans the way."""
        from . import team

        size_m = validate_size(size_m)
        env = None if course else pairing.env_access()
        if size_m is not None and (not new_session or env or not team.load()):
            raise ValueError("size_m requires a new session with team login; existing pairings "
                             "and offline recordings cannot change a frozen session profile")
        if random and not team.load():
            raise ValueError("random courses need drone-challenge login")
        if course or random or (new_session and not env and team.load()):
            try:
                team.new_session(
                    course,
                    trial_id,
                    size_m=size_m,
                    simulation=simulation,
                    random=random,
                    seed=seed,
                    obstacles=obstacles,
                    targets=targets,
                )
            except team.TeamError as exc:
                if course or random or size_m is not None:
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
        self._frame: StartFrame | None = None
        self._relative: list[tuple[float, float, float, float, float]] = []
        self._relative_lock = threading.Lock()

    def course(self) -> Course | None:
        """The session's course: ``.waypoints`` / ``.route`` relative to the start pose,
        ``.hall_waypoints()`` once the measured start is bound. None when offline."""
        if self.client is None:
            return None
        return Course(self.client.course())

    def assist(self) -> dict | None:
        """Explicitly request optional assistance; None offline, pose=None when gated."""
        return self.client.assist() if self.client else None

    def watch_assist(self, callback, interval_s: float = 0.1):
        """Opt-in callback subscription; close it explicitly or use a with block."""
        from .assistance import AssistanceSubscription
        return AssistanceSubscription(self.assist, callback, interval_s)

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

    # Start-relative poses ---------------------------------------------------------------

    def wait_start(self, timeout_s: float = 3.5) -> bool:
        """Keep the drone still on the start mark until this returns. True: the hall
        measured the start (poses go out in hall coordinates, scored); False: no
        measurement (own test, course coordinates) or offline. Raises StartFrameError
        when the measurement is unusable, e.g. the drone moved: start again."""
        frame = self._start_frame(timeout_s)
        hall = frame.wait(timeout_s + 15.0)
        self._flush_relative()
        return hall

    def pose_relative(
        self, x: float, y: float, z: float, yaw: float = 0.0, timestamp: float | None = None
    ) -> bool:
        """Like ``pose()``, but relative to the start: x right, y forward, z up (metres),
        yaw 0 = heading at the start. Never blocks: until the start is decided the poses
        are kept with their own timestamps and sent afterwards."""
        stamp = time.time() if timestamp is None else float(timestamp)
        frame = self._start_frame()
        with self._relative_lock:  # keeps buffered and new poses in time order
            if not frame.resolved:
                self._relative.append((x, y, z, yaw, stamp))
                return True
            self._send_relative(frame)
            if frame.error:
                self.dropped += 1
                return False
            return self.pose(*frame.apply(x, y, z, yaw), stamp)

    def stop(self, end_stamp: float | None = None, timeout_s: float = 30.0):
        frame = self._frame
        if frame is not None and self._relative:
            try:
                frame.wait(timeout_s)
            except (TimeoutError, StartFrameError):
                pass
            self._flush_relative()
        return super().stop(end_stamp, timeout_s)

    def _start_frame(self, timeout_s: float = 3.5) -> StartFrame:
        with self._relative_lock:
            if self._frame is None:
                self._frame = StartFrame(self.course, timeout_s)
            return self._frame

    def _flush_relative(self) -> None:
        frame = self._frame
        if frame is not None and frame.resolved:
            with self._relative_lock:
                self._send_relative(frame)

    def _send_relative(self, frame: StartFrame) -> None:
        pending, self._relative = self._relative, []
        for x, y, z, yaw, stamp in pending:
            if frame.error:
                self.dropped += 1
            else:
                self.pose(*frame.apply(x, y, z, yaw), stamp)
