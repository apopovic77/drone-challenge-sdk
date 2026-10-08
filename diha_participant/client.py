"""Standard-library HTTP client for the live session API."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Callable

LOCAL_HOSTS = {"127.0.0.1", "localhost", "::1"}


class ApiError(RuntimeError):
    """Raised when the evaluation API cannot be reached or rejects a request."""

    def __init__(self, message: str, status: int | None = None, code: str | None = None) -> None:
        super().__init__(message)
        self.status = status
        self.code = code

    @property
    def retryable(self) -> bool:
        """Network errors, timeouts, 429 and 5xx are worth retrying; contract errors are not."""
        return self.status is None or self.status == 429 or self.status >= 500


class ApiClient:
    """Minimal client for one session and one device token; no flight-control logic."""

    def __init__(
        self,
        base_url: str,
        token: str,
        session_id: str,
        *,
        timeout_s: float = 3.0,
        clock: Callable[[], int] = time.time_ns,
    ) -> None:
        parsed = urllib.parse.urlparse(base_url)
        if parsed.scheme != "https" and not (parsed.scheme == "http" and parsed.hostname in LOCAL_HOSTS):
            raise ValueError("API base_url must use HTTPS (plain HTTP only for localhost tests)")
        if not token or not session_id:
            raise ValueError("token and session_id are required")
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.session_id = session_id
        self.timeout_s = timeout_s
        # The clock the device stamps with; clock measurements must use the same one.
        self.clock_ns = clock

    def post_poses(self, source: str, items: list[dict]) -> dict:
        if source not in ("team", "reference"):
            raise ValueError("source must be 'team' or 'reference'")
        return self._request("POST", f"{source}-poses", {"items": items})

    def post_events(self, items: list[dict]) -> dict:
        return self._request("POST", "events", {"items": items})

    def clock(self) -> dict:
        return self._request("GET", "clock")

    def course(self) -> dict:
        """Planned route, course geometry and anchor of this session (raw, hall frame)."""
        return self._request("GET", "course")

    def assist(self) -> dict:
        """One optional M0 fix in hall coordinates, or pose=None. Never cached.

        RTT uses a monotonic clock. Conservatively include the whole RTT in pose age
        and remaining time; no local/server clock assumption is needed.
        """
        sent = time.monotonic()
        value = self._request("GET", "assist")
        rtt = time.monotonic() - sent
        value["request_rtt_s"] = rtt
        pose = value.get("pose")
        if pose is not None:
            age = pose["age_s"] + rtt
            remaining = (int(value["deadline_ns"]) - int(value["server_time_ns"])) / 1e9
            if age > value["max_pose_age_s"] or remaining <= rtt:
                value.update(pose=None, status="unavailable", reason="transport_expired")
            else:
                value["pose_age_upper_bound_s"] = age
        return value

    def clock_report(self, offset_ns: int, rtt_ns: int) -> dict:
        return self._request("POST", "clock-reports", {"offset_ns": offset_ns, "rtt_ns": rtt_ns})

    def measure_clock(self, samples: int = 8) -> tuple[int, int]:
        """NTP-style offset (server minus local clock) from the lowest-latency exchange."""
        best: tuple[int, int] | None = None
        failure: ApiError | None = None
        for _ in range(samples):
            sent = self.clock_ns()
            try:
                answer = self.clock()
            except ApiError as exc:
                if not exc.retryable:
                    raise
                failure = exc  # a lost exchange only costs one sample
                continue
            received = self.clock_ns()
            server_in, server_out = int(answer["server_receive_ns"]), int(answer["server_send_ns"])
            rtt = (received - sent) - (server_out - server_in)
            if rtt < 0:
                # Only a local clock step during the exchange makes the round trip negative;
                # such a sample says nothing about the offset (and must never look "best").
                failure = ApiError("local clock changed during the clock measurement")
                continue
            offset = ((server_in - sent) + (server_out - received)) // 2
            if best is None or rtt < best[1]:
                best = (offset, rtt)
        if best is None:
            raise failure or ApiError("no clock sample")
        return best

    def sync_clock(self, samples: int = 8) -> tuple[int, int]:
        """Measure the clock offset and report it so the server can document it."""
        offset, rtt = self.measure_clock(samples)
        self.clock_report(offset, rtt)
        return offset, rtt

    def _request(self, method: str, resource: str, body: dict | None = None) -> dict:
        request = urllib.request.Request(
            f"{self.base_url}/api/v1/sessions/{self.session_id}/{resource}",
            data=json.dumps(body, allow_nan=False).encode("utf-8") if body is not None else None,
            headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"},
            method=method,
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            code = None
            try:
                code = json.loads(exc.read().decode("utf-8"))["error"]["code"]
            except (ValueError, KeyError, TypeError):
                pass
            raise ApiError(f"API rejected request: HTTP {exc.code} {code or ''}".strip(), exc.code, code) from exc
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            raise ApiError(f"API request failed: {exc}") from exc
        if not isinstance(payload, dict):
            raise ApiError("API response must be a JSON object")
        return payload
