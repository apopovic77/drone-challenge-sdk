"""One-class interface for teams: configure, start, send poses, stop.

    from diha_participant import DroneLive

    with DroneLive() as live:          # URL, session and token from the environment
        live.takeoff()
        while flying:
            live.pose(x, y, z, yaw)    # timestamp = now, or pass stamp=...
        live.switch()                  # external positioning off
    # leaving the block sends flight_end, flushes the upload and writes the local files

Everything else is handled inside: clock synchronisation with the server, a local
spool with retries and resume, the challenge CSV files, and the lifecycle events.
``pose()`` never blocks on the network and never raises for a bad sample; it
returns False instead, so it is safe to call from a flight loop.

Without URL/session/token the same code only writes the local files (offline mode).
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path

from .client import ApiClient, ApiError
from .logger import Pose, SessionLogger
from .uploader import TelemetryUploader, UploadStats

ENV_URL = "DRONE_LIVE_URL"
ENV_SESSION = "DRONE_LIVE_SESSION"
ENV_TOKEN = "DRONE_LIVE_TOKEN"


class DroneLive:
    """Team-side live link for one flight (one session).

    Args:
        url: server base URL, default ``$DRONE_LIVE_URL``.
        session: session id, default ``$DRONE_LIVE_SESSION``.
        token: team token, default ``$DRONE_LIVE_TOKEN``. Keep it out of source code
            and command lines.
        out_dir: folder for ``team.csv``, ``events.jsonl``, ``switch.txt`` and the upload
            spool; default ``flight-<session or timestamp>``.
        frame_id, team_id, trial_id, reference_point: metadata for the local files.
        clock_sync_s: interval of the background clock measurement; 0 disables it.
    """

    def __init__(
        self,
        url: str | None = None,
        session: str | None = None,
        token: str | None = None,
        *,
        out_dir: str | os.PathLike[str] | None = None,
        frame_id: str = "hall",
        team_id: str = "team",
        trial_id: str | None = None,
        reference_point: str = "M0",
        clock_sync_s: float = 10.0,
    ) -> None:
        url = url or os.environ.get(ENV_URL)
        session = session or os.environ.get(ENV_SESSION)
        token = token or os.environ.get(ENV_TOKEN)
        given = [v for v in (url, session, token) if v]
        if given and len(given) < 3:
            raise ValueError(
                f"set all of {ENV_URL}, {ENV_SESSION}, {ENV_TOKEN} (or none for offline mode)"
            )
        self.online = bool(given)
        name = session or time.strftime("%Y%m%d-%H%M%S")
        base = Path(out_dir or f"flight-{name}")
        # A restart never overwrites an earlier flight's files: flight-X, flight-X-2, ...
        self.directory = base
        n = 1
        while (self.directory / "team.csv").exists():
            n += 1
            self.directory = base.with_name(f"{base.name}-{n}")
        self.client = ApiClient(url, token, session) if self.online else None
        # The upload spool belongs to the session, so a restarted program resends what the
        # server has not confirmed yet and continues the sequence numbers.
        self.uploader = (
            TelemetryUploader(self.client, "team", base.with_name(f".spool-{name}"))
            if self.client
            else None
        )
        self._logger = SessionLogger(
            self.directory,
            frame_id=frame_id,
            reference_point=reference_point,
            team_id=team_id,
            trial_id=trial_id or name,
            uploader=self.uploader,
        )
        self.clock_sync_s = clock_sync_s
        self.clock_offset_ns: int | None = None
        self.dropped = 0
        self._last_stamp: float | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._started = False
        self._stopped = False

    @classmethod
    def from_pairing(cls, pairing: str | dict, **options) -> "DroneLive":
        """Configure from the pairing QR code content (JSON text or parsed dict)."""
        data = json.loads(pairing) if isinstance(pairing, str) else dict(pairing)
        if data.get("type") != "drone-challenge-pairing":
            raise ValueError("not a drone-challenge pairing code")
        defaults = {
            "frame_id": data.get("frame") or "hall",
            "team_id": data.get("team_id") or "team",
            "trial_id": data.get("trial_id"),
        }
        return cls(data["api"], data["session"], data["token"], **{**defaults, **options})

    # Lifecycle ---------------------------------------------------------------------------

    def start(self, stamp: float | None = None) -> "DroneLive":
        """Synchronise the clock (best effort, unless ``clock_sync_s=0``) and record
        ``system_start`` (now, or ``stamp``). Call it when your system really starts."""
        if self._started:
            return self
        self._started = True
        if self.client is not None and self.clock_sync_s > 0:
            self._sync_clock()
            threading.Thread(target=self._clock_loop, name="drone-live-clock", daemon=True).start()
        self._event("system_start", stamp)
        return self

    def takeoff(self, stamp: float | None = None) -> None:
        self._event("takeoff", stamp)

    def switch(self, stamp: float | None = None) -> None:
        """External positioning is switched off from here on (start of the scored part)."""
        self._event("switch", stamp)

    def pose(
        self, x: float, y: float, z: float, yaw: float = 0.0, stamp: float | None = None
    ) -> bool:
        """Queue one team estimate (metres, radians, hall frame). Never blocks.

        Returns False (and counts it in ``dropped``) for non-finite values or a
        timestamp that does not increase; the flight loop is never interrupted.
        """
        stamp = time.time() if stamp is None else float(stamp)
        with self._lock:
            if self._stopped or not self._started:
                self.dropped += 1
                return False
            values = (stamp, x, y, z, yaw)
            if not all(math.isfinite(float(v)) for v in values) or (
                self._last_stamp is not None and stamp <= self._last_stamp
            ):
                self.dropped += 1
                return False
            try:
                self._logger.publish_estimate(Pose(stamp, float(x), float(y), float(z), float(yaw)))
            except (ValueError, OverflowError):
                self.dropped += 1
                return False
            self._last_stamp = stamp
        return True

    def stop(self, end_stamp: float | None = None, timeout_s: float = 30.0) -> UploadStats | None:
        """Record ``flight_end``, write the local files and wait for the upload.

        ``flight_end`` gets ``end_stamp`` if given, otherwise the stamp of the last pose, so
        the end of the scored window always has data even when ``stop()`` runs later.
        """
        if self._stopped:
            return self.uploader.stats if self.uploader else None
        if self._started:
            self._event("flight_end", end_stamp if end_stamp is not None else self._last_stamp)
        with self._lock:
            self._stopped = True
        self._stop.set()
        self._logger.close(allow_empty=True)
        return self.uploader.close(timeout_s=timeout_s) if self.uploader else None

    def __enter__(self) -> "DroneLive":
        return self.start()

    def __exit__(self, *_exc) -> None:
        self.stop()

    # Status ------------------------------------------------------------------------------

    @property
    def pending(self) -> int:
        """Poses and events not yet confirmed by the server (0 offline)."""
        return self.uploader.pending if self.uploader else 0

    # Internals ---------------------------------------------------------------------------

    def _event(self, kind: str, stamp: float | None = None) -> None:
        with self._lock:
            if self._stopped:
                return
            self._logger.record_event(kind, time.time() if stamp is None else float(stamp))

    def _sync_clock(self) -> None:
        try:
            self.clock_offset_ns, _ = self.client.sync_clock()
        except ApiError:
            pass  # documented by the server when it works; never blocks the flight

    def _clock_loop(self) -> None:
        while not self._stop.wait(self.clock_sync_s):
            self._sync_clock()
