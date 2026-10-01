"""Non-blocking live upload with a local spool, batching, retries and resume.

The flight process only appends to an in-memory queue and a local spool file; a
background thread sends batches. Every message carries a sequence number, so a
batch that is resent after a timeout is recognised by the server as a duplicate.
After a crash or restart the uploader re-reads the spool and resends everything
the server has not acknowledged.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

from .client import ApiClient, ApiError
from .logger import Event, Pose


@dataclass
class UploadStats:
    sent_poses: int = 0
    sent_events: int = 0
    duplicates: int = 0
    conflicts: list[int] = field(default_factory=list)
    retries: int = 0
    rejected_batches: int = 0
    last_error: str | None = None
    stopped: str | None = None
    # Hall stream only: which session the server bound it to, and poses it dropped.
    hall: dict | None = None
    hall_dropped: int = 0


class _Stream:
    """One ordered, spooled message stream (poses or events)."""

    def __init__(self, directory: Path, name: str) -> None:
        self.spool_path = directory / f"{name}.jsonl"
        self.ack_path = directory / f"{name}.acked"
        self.pending: deque[dict] = deque()
        self.acked = int(self.ack_path.read_text()) if self.ack_path.exists() else -1
        self.next_seq = 0
        if self.spool_path.exists():
            with self.spool_path.open(encoding="utf-8") as handle:
                for line in handle:
                    try:
                        item = json.loads(line)
                    except json.JSONDecodeError:
                        break  # a torn last line after a crash was never acknowledged
                    self.next_seq = max(self.next_seq, item["seq"] + 1)
                    if item["seq"] > self.acked:
                        self.pending.append(item)
        self.spool = self.spool_path.open("a", encoding="utf-8")

    def append(self, item: dict) -> None:
        self.spool.write(json.dumps(item, separators=(",", ":"), allow_nan=False) + "\n")
        self.spool.flush()
        self.pending.append(item)

    def acknowledge(self, seq: int) -> None:
        self.acked = seq
        temp = self.ack_path.with_suffix(".tmp")
        temp.write_text(str(seq))
        os.replace(temp, self.ack_path)

    def sync(self) -> None:
        os.fsync(self.spool.fileno())


class TelemetryUploader:
    """Forward poses and events for one source ('team' or 'reference') in the background."""

    def __init__(
        self,
        client: ApiClient,
        source: str,
        spool_dir: str | os.PathLike[str],
        *,
        batch_size: int = 200,
        max_pending: int = 200_000,
        retry_max_s: float = 5.0,
        idle_s: float = 0.05,
    ) -> None:
        if source not in ("team", "reference"):
            raise ValueError("source must be 'team' or 'reference'")
        self.client = client
        self.source = source
        self.batch_size = batch_size
        self.max_pending = max_pending
        self.retry_max_s = retry_max_s
        self.idle_s = idle_s
        directory = Path(spool_dir)
        directory.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closing = False
        self._poses = _Stream(directory, f"{source}-poses")
        self._events = _Stream(directory, f"{source}-events")
        self._in_flight = self._poses
        self.stats = UploadStats()
        self._thread = threading.Thread(target=self._run, name=f"telemetry-{source}", daemon=True)
        self._thread.start()

    # Producer side: never blocks on the network ----------------------------------------------

    def publish(self, pose: Pose) -> int:
        with self._lock:
            if len(self._poses.pending) >= self.max_pending:
                raise OverflowError("upload backlog full; local spool still holds all data")
            seq = self._poses.next_seq
            self._poses.next_seq += 1
            self._poses.append(
                {"seq": seq, "stamp_s": pose.stamp_text or pose.stamp_s, "x": pose.x, "y": pose.y, "z": pose.z, "yaw": pose.yaw}
            )
        self._wake.set()
        return seq

    def record_event(self, event: Event) -> int:
        with self._lock:
            seq = self._events.next_seq
            self._events.next_seq += 1
            self._events.append({"seq": seq, "kind": event.kind, "stamp_s": event.stamp_text or event.stamp_s})
        self._wake.set()
        return seq

    @property
    def pending(self) -> int:
        with self._lock:
            return len(self._poses.pending) + len(self._events.pending)

    def flush(self, timeout_s: float = 10.0) -> bool:
        """Wait until everything is acknowledged; False on timeout or permanent stop."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.pending == 0:
                return True
            if self.stats.stopped:
                return False
            self._wake.set()
            time.sleep(0.02)
        return self.pending == 0

    def close(self, timeout_s: float = 10.0) -> UploadStats:
        self.flush(timeout_s)
        self._closing = True
        self._wake.set()
        self._thread.join(timeout=2)
        for stream in (self._poses, self._events):
            stream.sync()
            stream.spool.close()
        return self.stats

    # Sender thread ------------------------------------------------------------------------

    def _run(self) -> None:
        delay = 0.0
        while not self._closing and not self.stats.stopped:
            self._wake.wait(timeout=self.idle_s if delay == 0 else delay)
            self._wake.clear()
            try:
                # Events first: they are few and decide the scoring window.
                sent = self._send(self._events, "events") or self._send(self._poses, "poses")
            except ApiError as exc:
                self.stats.last_error = str(exc)
                if exc.retryable:
                    self.stats.retries += 1
                    delay = min(max(delay * 2, 0.1), self.retry_max_s)
                    continue
                if exc.code in ("session_closed", "not_found", "authentication_required", "forbidden_source"):
                    self.stats.stopped = exc.code
                    return
                self._drop_head_batch()
                continue
            delay = 0.0
            if sent:
                self._wake.set()

    def _batch(self, stream: _Stream) -> list[dict]:
        with self._lock:
            size = min(len(stream.pending), self.batch_size if stream is self._poses else 100)
            batch = [stream.pending[i] for i in range(size)]
            if batch:
                stream.sync()  # anything the server acknowledges is also durable locally
            return batch

    def _send(self, stream: _Stream, kind: str) -> bool:
        batch = self._batch(stream)
        if not batch:
            return False
        self._in_flight = stream
        if kind == "events":
            result = self.client.post_events(batch)
        else:
            result = self.client.post_poses(self.source, batch)
        with self._lock:
            for _ in batch:
                stream.pending.popleft()
            stream.acknowledge(batch[-1]["seq"])
        if kind == "events":
            self.stats.sent_events += int(result.get("accepted", 0))
        else:
            self.stats.sent_poses += int(result.get("accepted", 0))
        self.stats.duplicates += int(result.get("duplicates", 0))
        self.stats.conflicts.extend(result.get("conflicts", []))
        if "dropped" in result or "hall" in result:
            self.stats.hall_dropped += int(result.get("dropped", 0))
            self.stats.hall = result.get("hall") or {"state": "bound", "session": result.get("session")}
        elif result.get("session"):
            self.stats.hall = {"state": "bound", "session": result["session"]}
        return True

    def _drop_head_batch(self) -> None:
        """A contract error would block the stream forever: skip that batch, keep it spooled."""
        self.stats.rejected_batches += 1
        stream = self._in_flight
        batch = self._batch(stream)
        with self._lock:
            for _ in batch:
                stream.pending.popleft()
            if batch:
                stream.acknowledge(batch[-1]["seq"])
