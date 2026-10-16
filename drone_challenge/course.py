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
the server binds the course anchor and
``course.hall_waypoints()`` gives the same points in hall coordinates:

    p_hall = measured_start + Rz(rotation) * (p - course_start)

Before that (or in a practice session without motion capture) it returns None.

Advanced training (course coordinates, i.e. the frame of ``obstacles`` and ``goal_course``;
a simulator that works in course coordinates gets a start anchor without rotation):

    course.collides(x, y, z)            # True if the vehicle at (x, y, z) touches a box or wall
    course.clearance(x, y, z)           # metres of free space left around the vehicle
    course.target_position(0, t)        # moving target 0, t seconds after the start event
"""

from __future__ import annotations

import math
from copy import deepcopy

__all__ = ["Course", "from_response"]


def _rotate(x: float, y: float, angle_rad: float) -> tuple[float, float]:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return c * x - s * y, s * x + c * y


class Course:
    """A session's course: relative waypoints and route, plus the hall form once anchored."""

    def __init__(self, response: dict) -> None:
        self.raw = response
        self.vehicle_profile: dict | None = deepcopy(response.get("vehicle_profile"))
        course = response.get("course") or {}
        self.course_id: str | None = course.get("course_id")
        self.version: int | None = course.get("version")
        self.name: str | None = course.get("name")
        self.assist_distance_m: float = float(response.get("assist_distance_m", 0.0))
        self.assist_max_s: float = float(response.get("assist_max_s", 10.0))
        # Older servers do not advertise a rule; do not infer their scoring policy.
        self.start_rule: str | None = response.get("start_rule")
        self.route_mode: str = course.get("route_mode", "prescribed")
        self.motion_model: str = course.get("motion_model", "flight")
        self.marker_height_m: float = float(course.get("marker_height_m", 0.2))
        self.environment: dict | None = deepcopy(course.get("environment"))
        self.obstacles: list[dict] = deepcopy(course.get("obstacles", []))
        self.hall: dict | None = deepcopy(course.get("hall"))
        self.drone_profile: dict | None = deepcopy(course.get("drone_profile"))
        self.geometry_anchored: bool = bool(course.get("geometry_anchored"))
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
        if self.route_mode == "environment" and self.environment:
            self.course_start = tuple(map(float, self.environment["start"]))
        self.anchor: dict | None = response.get("anchor")
        # Moving targets (advanced training): path in course coordinates, see target_position.
        self.targets: list[dict] = deepcopy(course.get("targets") or [])

    # Advanced training, course coordinates -------------------------------------------------

    @property
    def goal_course(self) -> tuple[float, float, float] | None:
        """Goal in course coordinates (the frame of ``obstacles``); None without a goal."""
        return tuple(map(float, self.environment["goal"])) if self.environment else None

    @property
    def start_course(self) -> tuple[float, float, float]:
        """Start in course coordinates (the frame of ``obstacles``)."""
        return self.course_start

    def _profile(self) -> dict | None:
        frozen = (self.vehicle_profile or {}).get("profile")
        return frozen or self.drone_profile

    @property
    def ground(self) -> bool:
        return self.motion_model == "ground"

    def vehicle_radius(self) -> float:
        """Radius the server checks contacts with (diagonal / 2), plus the safety margin."""
        p = self._profile()
        if not p:
            return 0.0
        height = 0.0 if self.ground else p["height"]
        return math.hypot(p["width"], p["depth"], height) / 2 + p.get("safety_margin", 0.0)

    def clearance(self, x: float, y: float, z: float) -> float:
        """Free distance (m) between the vehicle envelope at (x, y, z) and the nearest box or
        hall wall; negative means contact. Course coordinates, same rule as the server."""
        radius = self.vehicle_radius()
        p = self._profile() or {}
        h = self.hall or {}
        best = math.inf
        if h:
            best = min(x - h["x_min"], h["x_max"] - x, y - h["y_min"], h["y_max"] - y)
            if not self.ground:
                best = min(best, z, h["height"] - z)
        for box in self.obstacles:
            base = float(box.get("base_z", 0.0))
            if self.ground:
                if base >= p.get("height", 0.0):
                    continue  # passes under it
                dz = 0.0
            else:
                dz = max(base - z, 0.0, z - base - box["height"])
            dx = max(abs(x - box["x"]) - box["width"] / 2, 0.0)
            dy = max(abs(y - box["y"]) - box["depth"] / 2, 0.0)
            best = min(best, math.sqrt(dx * dx + dy * dy + dz * dz))
        return best - radius

    def collides(self, x: float, y: float, z: float, margin: float = 0.0) -> bool:
        """True if the vehicle at (x, y, z) touches a box or the hall (plus ``margin``)."""
        return self.clearance(x, y, z) < margin

    def target_position(self, target: int | str, t: float) -> tuple[float, float, float]:
        """Moving target position, course coordinates, ``t`` seconds after the start event.

        ``target`` is the index or the id. The target waits ``start_delay_s``, then travels
        its path at ``speed_mps``: ``loop`` closes the path, ``pingpong`` goes forth and
        back. Same function as the server evaluation.
        """
        if isinstance(target, str):
            found = [x for x in self.targets if x["id"] == target]
            if not found:
                raise KeyError(target)
            spec = found[0]
        else:
            spec = self.targets[target]
        points = [tuple(map(float, p)) for p in spec["path"]]
        if spec.get("mode", "loop") == "loop":
            points.append(points[0])
        lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
        total = sum(lengths)
        s = spec.get("speed_mps", 0.3) * max(0.0, t - spec.get("start_delay_s", 0.0))
        if total <= 0:
            return points[0]
        if spec.get("mode", "loop") == "loop":
            s %= total
        else:
            s %= 2 * total
            if s > total:
                s = 2 * total - s
        for (a, b), length in zip(zip(points, points[1:]), lengths):
            if s <= length:
                f = s / length if length else 0.0
                return tuple(a[k] + f * (b[k] - a[k]) for k in range(3))
            s -= length
        return points[-1]

    @property
    def goal(self) -> tuple[float, float, float] | None:
        """Advanced goal relative to the start pose; the team plans its own path."""
        return self._relative(tuple(self.environment["goal"])) if self.environment else None

    def hall_goal(self) -> tuple[float, float, float] | None:
        """Goal in the tracked hall frame only after a bound start anchor exists."""
        return (
            self._to_hall(tuple(self.environment["goal"]))
            if self.environment and self.anchored
            else None
        )

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
