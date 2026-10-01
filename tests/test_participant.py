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

