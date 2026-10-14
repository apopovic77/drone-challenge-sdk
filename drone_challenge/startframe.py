"""Start-relative poses: the team navigates relative to its start, the SDK sends them right.

After the start event the server knows whether the hall's camera system measured the
start (the course anchor is bound). Then poses go out in hall coordinates:

    p_hall = measured_start + Rz(rotation + start_yaw) * p_relative

Without a measurement (own test without OptiTrack) the same rule uses the course start, so
the practice report compares the flight with the course in its own coordinates. Offline
the relative pose is recorded unchanged. The same team code works in all three cases.
"""

from __future__ import annotations

import math
import threading
import time

__all__ = ["StartFrame", "StartFrameError"]

POLL_S = 0.2


class StartFrameError(RuntimeError):
    """The camera system saw the start, but the measurement is unusable (e.g. the drone
    moved). Start again: that creates a new session."""


class StartFrame:
    """Resolves once, in the background, where start-relative poses belong."""

    def __init__(self, fetch_course, timeout_s: float) -> None:
        self._fetch = fetch_course  # () -> Course | None
        self._timeout_s = timeout_s
        self._done = threading.Event()
        self.origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.rotation_rad = 0.0
        self.hall: bool | None = None  # True: hall coordinates, False: course/offline
        self.error: StartFrameError | None = None
        self.course = None
        threading.Thread(target=self._resolve, name="drone-start-frame", daemon=True).start()

    def wait(self, timeout_s: float | None = None) -> bool:
        if not self._done.wait(timeout_s):
            raise TimeoutError("Startmessung noch nicht entschieden")
        if self.error:
            raise self.error
        return bool(self.hall)

    @property
    def resolved(self) -> bool:
        return self._done.is_set()

    def apply(self, x: float, y: float, z: float, yaw: float) -> tuple[float, float, float, float]:
        c, s = math.cos(self.rotation_rad), math.sin(self.rotation_rad)
        ox, oy, oz = self.origin
        return ox + c * x - s * y, oy + s * x + c * y, oz + z, yaw + self.rotation_rad

    def _resolve(self) -> None:
        deadline = time.monotonic() + self._timeout_s
        try:
            while True:
                try:
                    course = self._fetch()
                except Exception as exc:  # network or server: retry until the deadline
                    if time.monotonic() >= deadline:
                        self.error = StartFrameError(f"Startmessung nicht abrufbar: {exc}")
                        return
                    time.sleep(POLL_S)
                    continue
                self.course = course
                if course is None:  # offline: record the relative pose unchanged
                    self.hall = False
                    return
                anchor = course.anchor or {}
                if course.anchored:
                    self.origin = tuple(map(float, anchor["measured_start_m"]))
                    self.rotation_rad = math.radians(anchor["rotation_deg"] + course.start_yaw_deg)
                    self.hall = True
                    return
                if time.monotonic() >= deadline:
                    if anchor.get("status") == "provisional":
                        self.error = StartFrameError(
                            f"Startmessung unbrauchbar ({anchor.get('reason')}): "
                            "Drohne beim Start ruhig halten und neu starten"
                        )
                        return
                    self.origin = tuple(map(float, course.course_start))
                    self.rotation_rad = math.radians(course.start_yaw_deg)
                    self.hall = False
                    return
                time.sleep(POLL_S)
        finally:
            self._done.set()
