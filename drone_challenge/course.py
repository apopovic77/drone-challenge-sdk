"""The course to fly: waypoints relative to the drone's start pose, and in the hall.

    from drone_challenge import Drone

    with Drone(course="Zufallskurs") as drone:
        course = drone.course()
        for w in course.waypoints:          # relative to the start pose, metres
            print(w["x"], w["y"], w["z"], w["segment"])

Relative frame: origin at the start point, +Y the direction the drone faces at the
start, +X to its right, +Z up. A course is drawn relative to its start; where the drone
really stands in the hall does not matter.

Hall frame: once the hall's motion capture has measured the drone at the start event,
the server binds the course anchor (contract: Content post #5127) and
``course.hall_waypoints()`` gives the same points in hall coordinates:

    p_hall = measured_start + Rz(rotation) * (p - course_start)

Before that (or in a practice session without motion capture) it returns None.
"""

from __future__ import annotations

import math

__all__ = ["Course", "from_response"]


def _rotate(x: float, y: float, angle_rad: float) -> tuple[float, float]:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return c * x - s * y, s * x + c * y


class Course:
    """A session's course: relative waypoints and route, plus the hall form once anchored."""

    def __init__(self, response: dict) -> None:
        self.raw = response
        course = response.get("course") or {}
        self.course_id: str | None = course.get("course_id")
        self.version: int | None = course.get("version")
        self.name: str | None = course.get("name")
        self.start_yaw_deg: float = float(course.get("start_yaw_deg") or 0.0)
        route = response.get("route") or []
        points = course.get("waypoints") or [
            {"x": p[0], "y": p[1], "z": p[2], "segment": "line"} for p in route
        ]
        self._hall_route = [tuple(map(float, p)) for p in route]
        self._hall_waypoints = [
            {"x": float(w["x"]), "y": float(w["y"]), "z": float(w["z"]), "segment": w["segment"]}
            for w in points
        ]
        first = self._hall_route[0] if self._hall_route else (0.0, 0.0, 0.0)
        if self._hall_waypoints:
            first = (
                self._hall_waypoints[0]["x"],
                self._hall_waypoints[0]["y"],
                self._hall_waypoints[0]["z"],
            )
        self.course_start: tuple[float, float, float] = first
        self.anchor: dict | None = response.get("anchor")

    # Relative to the start pose -------------------------------------------------------------

    def _relative(self, p: tuple[float, float, float]) -> tuple[float, float, float]:
        # Undo the course's own start direction: +Y becomes "forward at the start".
        x, y = _rotate(
            p[0] - self.course_start[0],
            p[1] - self.course_start[1],
            -math.radians(self.start_yaw_deg),
        )
        return (x, y, p[2] - self.course_start[2])

    @property
    def waypoints(self) -> list[dict]:
        """Waypoints relative to the start pose: x, y, z (m) and segment ("line" or
        "curve": the route bends smoothly through a curve point)."""
        out = []
        for w in self._hall_waypoints:
            x, y, z = self._relative((w["x"], w["y"], w["z"]))
            out.append({"x": x, "y": y, "z": z, "segment": w["segment"]})
        return out

    @property
    def route(self) -> list[tuple[float, float, float]]:
        """The planned route as a dense polyline (every 0.1 m), relative to the start pose.
        This is the line the flight is compared with."""
        return [self._relative(p) for p in self._hall_route]

    # Hall frame, once anchored ----------------------------------------------------------------

    @property
    def anchored(self) -> bool:
        """True once the server has bound the anchor from the measured start."""
        a = self.anchor or {}
        return bool(a.get("bound")) and a.get("status") == "measured"

    def _to_hall(self, p: tuple[float, float, float]) -> tuple[float, float, float]:
        a = self.anchor or {}
        cs, ms = a["course_start_m"], a["measured_start_m"]
        x, y = _rotate(p[0] - cs[0], p[1] - cs[1], math.radians(a["rotation_deg"]))
        return (ms[0] + x, ms[1] + y, ms[2] + p[2] - cs[2])

    def hall_waypoints(self) -> list[dict] | None:
        """Waypoints in hall coordinates (the measured start pose applied), or None
        before the anchor is bound."""
        if not self.anchored:
            return None
        out = []
        for w in self._hall_waypoints:
            x, y, z = self._to_hall((w["x"], w["y"], w["z"]))
            out.append({"x": x, "y": y, "z": z, "segment": w["segment"]})
        return out

    def hall_route(self) -> list[tuple[float, float, float]] | None:
        """The dense route in hall coordinates, or None before the anchor is bound."""
        if not self.anchored:
            return None
        return [self._to_hall(p) for p in self._hall_route]


def from_response(response: dict) -> Course:
    return Course(response)
