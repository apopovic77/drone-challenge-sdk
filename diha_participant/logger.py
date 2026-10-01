"""Deterministic team-pose and event recording for Challenge 1A."""

from __future__ import annotations

import csv
import json
import math
import os
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, TextIO

if TYPE_CHECKING:
    from .uploader import TelemetryUploader

# Event kinds understood by the live session API.
LIVE_EVENT_KINDS = ("system_start", "takeoff", "switch", "flight_end", "external_reference_used")


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


@dataclass(frozen=True)
class Pose:
    """A team-estimated pose in the declared global frame."""

    stamp_s: float
    x: float
    y: float
    z: float
    yaw: float
    # Exact decimal seconds (e.g. from a ROS2 header); a float loses ~100 ns at epoch scale.
    stamp_text: str | None = field(default=None, compare=False)

    def __post_init__(self) -> None:
        for name in ("stamp_s", "x", "y", "z", "yaw"):
            _finite(getattr(self, name), name)

    def csv_row(self) -> list[str]:
        stamp = self.stamp_text or str(self.stamp_s)
        return [stamp] + [str(v) for v in (self.x, self.y, self.z, self.yaw)]


@dataclass(frozen=True)
class Event:
    """A lifecycle event recorded alongside poses."""

    kind: str
    stamp_s: float
    stamp_text: str | None = field(default=None, compare=False)

    def __post_init__(self) -> None:
        if not self.kind or any(c in self.kind for c in "\r\n,"):
            raise ValueError("kind must be a non-empty single-line value")
        _finite(self.stamp_s, "stamp_s")


class SessionLogger:
    """Record only the participant estimate and lifecycle events.

    ``team.csv`` (the challenge's five columns) and ``events.jsonl`` are written
    continuously: every pose and event is appended and flushed at once, and synced
    to disk at least once per second, so a crash loses at most the last second.
    ``switch.txt`` and ``metadata.json`` are written on :meth:`close`. In
    competition mode an attempt to record a ground-truth pose is rejected rather
    than silently mixing sources.
    """

    SYNC_INTERVAL_S = 1.0

    def __init__(
        self,
        directory: str | os.PathLike[str],
        *,
        frame_id: str,
        reference_point: str,
        team_id: str,
        trial_id: str,
        development_mode: bool = False,
        uploader: "TelemetryUploader | None" = None,
        overwrite: bool = False,
    ) -> None:
        if not frame_id or not reference_point or not team_id or not trial_id:
            raise ValueError("frame_id, reference_point, team_id and trial_id are required")
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.frame_id = frame_id
        self.reference_point = reference_point
        self.team_id = team_id
        self.trial_id = trial_id
        self.development_mode = development_mode
        self.uploader = uploader
        self._last_stamp: float | None = None
        self._count = 0
        self._events: list[Event] = []
        self._closed = False
        team_csv = self.directory / "team.csv"
        if team_csv.exists() and team_csv.stat().st_size > 0 and not overwrite:
            raise FileExistsError(f"{team_csv} already holds a recording; use a new directory")
        self._team_file = team_csv.open("w", encoding="utf-8", newline="")
        self._writer = csv.writer(self._team_file)
        self._writer.writerow(("stamp", "x", "y", "z", "yaw"))
        self._events_file = (self.directory / "events.jsonl").open("w", encoding="utf-8")
        self._last_sync = time.monotonic()
        self._sync(force=True)

    def _sync(self, force: bool = False) -> None:
        self._team_file.flush()
        self._events_file.flush()
        if force or time.monotonic() - self._last_sync >= self.SYNC_INTERVAL_S:
            os.fsync(self._team_file.fileno())
            os.fsync(self._events_file.fileno())
            self._last_sync = time.monotonic()

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("session is closed")

    def publish_estimate(self, pose: Pose) -> None:
        """Append a monotonically timestamped team estimate."""
        self._ensure_open()
        if self._last_stamp is not None and pose.stamp_s <= self._last_stamp:
            raise ValueError("team pose timestamps must be strictly increasing")
        self._writer.writerow(pose.csv_row())
        self._last_stamp = pose.stamp_s
        self._count += 1
        self._sync()
        if self.uploader is not None:
            self.uploader.publish(pose)

    def publish_ground_truth(self, pose: Pose) -> None:
        """Record development-only reference data outside the team export."""
        self._ensure_open()
        if not self.development_mode:
            raise PermissionError("ground truth is unavailable in competition mode")
        # Keep it in a separate in-memory stream and never place it in team.csv.
        if not hasattr(self, "_ground_truth"):
            self._ground_truth: list[Pose] = []
        if self._ground_truth and pose.stamp_s <= self._ground_truth[-1].stamp_s:
            raise ValueError("ground-truth timestamps must be strictly increasing")
        self._ground_truth.append(pose)

    def record_event(self, kind: str, stamp_s: float) -> None:
        """Record start, takeoff, switch or end events."""
        self._ensure_open()
        event = Event(kind, stamp_s)
        if self.uploader is not None and kind not in LIVE_EVENT_KINDS:
            raise ValueError(f"live upload supports only {', '.join(LIVE_EVENT_KINDS)}")
        self._events.append(event)
        self._events_file.write(
            json.dumps({"kind": event.kind, "stamp_s": event.stamp_text or event.stamp_s}) + "\n"
        )
        self._sync(force=True)
        if self.uploader is not None:
            self.uploader.record_event(event)

    def close(self, allow_empty: bool = False) -> dict[str, str]:
        """Finish the streamed files, write switch.txt and metadata, return paths.

        ``allow_empty`` closes a session without any estimate (e.g. an aborted start)
        instead of raising; the files then only contain the header and the events.
        """
        self._ensure_open()
        if not self._count and not allow_empty:
            raise ValueError("at least one team estimate is required")
        self._sync(force=True)
        self._team_file.close()
        self._events_file.close()
        for event in self._events:
            if event.kind == "switch":
                self._atomic_text(
                    self.directory / "switch.txt", f"{event.stamp_text or event.stamp_s}\n"
                )
                break
        metadata = {
            "schema_version": "drone-live-participant-v1",
            "team_id": self.team_id,
            "trial_id": self.trial_id,
            "frame_id": self.frame_id,
            "reference_point": self.reference_point,
            "source": "team_estimate",
            "development_mode": self.development_mode,
            "poses": self._count,
        }
        self._atomic_text(self.directory / "metadata.json", json.dumps(metadata, indent=2) + "\n")
        self._closed = True
        return {"team": str(self.directory / "team.csv"), "events": str(self.directory / "events.jsonl")}

    @staticmethod
    def _atomic_text(path: Path, value: str) -> None:
        fd, temp_name = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
        try:
            with os.fdopen(fd, "w", encoding="utf-8", newline="") as handle:
                handle.write(value)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp_name, path)
        except BaseException:
            os.unlink(temp_name)
            raise
