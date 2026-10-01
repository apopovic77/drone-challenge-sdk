"""The course formula against the fixtures shared with the server and the apps."""

import json
from pathlib import Path

from drone_challenge.course import Course

HERE = Path(__file__).resolve().parent
# In the platform repository the fixtures live with the server tests; the public
# package repository carries a copy next to this file.
CANDIDATES = [
    HERE / "fixtures" / "route_anchor_cases.json",
    HERE.parents[1] / "backend" / "tests" / "fixtures" / "route_anchor_cases.json",
]


def test_course_formula_matches_the_shared_fixtures():
    fixtures = json.loads(next(p for p in CANDIDATES if p.exists()).read_text())
    for case in fixtures["cases"]:
        course = Course(
            {
                "route": case["route"],
                "course": None,
                "anchor": {**case["anchor"], "status": "measured", "bound": True},
            }
        )
        for got, want in zip(course.hall_route(), case["expected"], strict=True):
            assert all(abs(g - w) < fixtures["tolerance_m"] for g, w in zip(got, want)), case


def test_relative_waypoints_start_at_zero_facing_plus_y():
    # A course whose own start direction is turned 90 deg: relative +Y is still forward.
    course = Course(
        {
            "route": [[1.0, 1.0, 0.0], [0.0, 1.0, 1.0]],
            "course": {
                "course_id": "c",
                "version": 1,
                "name": "C",
                "start_yaw_deg": 90.0,
                "waypoints": [
                    {"x": 1.0, "y": 1.0, "z": 0.0, "segment": "line"},
                    {"x": 0.0, "y": 1.0, "z": 1.0, "segment": "line"},
                ],
            },
            "anchor": None,
        }
    )
    first, second = course.waypoints
    assert (first["x"], first["y"], first["z"]) == (0.0, 0.0, 0.0)
    assert abs(second["x"]) < 1e-12 and abs(second["y"] - 1.0) < 1e-12 and second["z"] == 1.0
    assert course.hall_waypoints() is None
