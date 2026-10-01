import csv
import json

import pytest

from diha_participant import Event, Pose, SessionLogger


def test_logger_writes_team_contract_and_events(tmp_path):
    logger = SessionLogger(tmp_path, frame_id="hall_world", reference_point="marker_origin", team_id="T", trial_id="F")
    logger.record_event("start", 1000)
    logger.publish_estimate(Pose(1000.2, 1, 2, 3, 0.4))
    logger.record_event("switch", 1006)
    files = logger.close()
    with open(files["team"], newline="", encoding="utf-8") as handle:
        assert list(csv.reader(handle)) == [["stamp", "x", "y", "z", "yaw"], ["1000.2", "1", "2", "3", "0.4"]]
    assert (tmp_path / "switch.txt").read_text() == "1006\n"
    assert json.loads((tmp_path / "events.jsonl").read_text().splitlines()[0])["kind"] == "start"


def test_ground_truth_rejected_in_competition_and_allowed_separately(tmp_path):
    logger = SessionLogger(tmp_path, frame_id="world", reference_point="origin", team_id="T", trial_id="F")
    with pytest.raises(PermissionError):
        logger.publish_ground_truth(Pose(1, 0, 0, 0, 0))
    dev = SessionLogger(tmp_path / "dev", frame_id="world", reference_point="origin", team_id="T", trial_id="F", development_mode=True)
    dev.publish_ground_truth(Pose(1, 0, 0, 0, 0))
    dev.publish_estimate(Pose(1, 0, 0, 0, 0))
    dev.close()
    assert not (tmp_path / "dev" / "team.csv").read_text().count("ground_truth")


def test_pose_timestamps_and_finite_values_are_checked(tmp_path):
    logger = SessionLogger(tmp_path, frame_id="world", reference_point="origin", team_id="T", trial_id="F")
    logger.publish_estimate(Pose(1, 0, 0, 0, 0))
    with pytest.raises(ValueError):
        logger.publish_estimate(Pose(1, 0, 0, 0, 0))
    with pytest.raises(ValueError):
        Pose(2, float("nan"), 0, 0, 0)



def test_clock_measurement_drops_exchanges_with_a_local_clock_step(monkeypatch):
    """A negative round trip only happens when the local clock steps during the exchange;
    such a sample must be dropped, not clipped to 0 (it would look like the best one)."""
    import pytest

    from diha_participant.client import ApiClient, ApiError

    # Local clock readings around three exchanges: the second has a -3 s step inside.
    readings = iter([1_000, 1_000 + 20_000_000, 5_000, 5_000 - 3_000_000_000, 9_000, 9_000 + 8_000_000])
    api = ApiClient("http://127.0.0.1:9", "token", "a" * 32, clock=lambda: next(readings))
    answers = iter(
        [
            {"server_receive_ns": 11_000_000, "server_send_ns": 11_000_000},
            {"server_receive_ns": 15_000_000, "server_send_ns": 15_000_000},
            {"server_receive_ns": 13_000_000, "server_send_ns": 13_000_000},
        ]
    )
    monkeypatch.setattr(api, "clock", lambda: next(answers))
    offset, rtt = api.measure_clock(samples=3)
    assert rtt == 8_000_000  # the stepped exchange (rtt < 0) was not chosen
    readings2 = iter([0, -1_000_000_000])
    api.clock_ns = lambda: next(readings2)
    monkeypatch.setattr(api, "clock", lambda: {"server_receive_ns": 5, "server_send_ns": 5})
    with pytest.raises(ApiError):
        api.measure_clock(samples=1)


def test_stable_clock_never_steps_with_the_system_clock(monkeypatch):
    from diha_participant import timebase

    wall = iter([1_000_000_000_000])
    mono = iter([5, 10, 20, 30])
    monkeypatch.setattr(timebase.time, "time_ns", lambda: next(wall))
    monkeypatch.setattr(timebase.time, "monotonic_ns", lambda: next(mono))
    clock = timebase.StableClock()  # wall 1e12 at monotonic 5
    # The system clock may now be stepped; only monotonic time advances this clock.
    assert [clock.time_ns(), clock.time_ns(), clock.time_ns()] == [
        1_000_000_000_005,
        1_000_000_000_015,
        1_000_000_000_025,
    ]
