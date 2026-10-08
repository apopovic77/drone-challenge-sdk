"""Optional assistance polling. Nothing subscribes until the team explicitly asks."""

from __future__ import annotations

import threading

from diha_participant.client import ApiError


class AssistanceSubscription:
    """Callback receives every status, including pose=None on loss/end. No replay/cache.

    The callback runs on this subscription's thread. ``close`` interrupts the interval;
    an in-flight HTTP call is bounded by ApiClient.timeout_s. Errors contain no pose.
    """

    def __init__(self, fetch, callback, interval_s: float = 0.1):
        if not 0.05 <= interval_s <= 1:
            raise ValueError("interval_s must be between 0.05 and 1")
        self._stop = threading.Event()
        self._fetch, self._callback, self._interval = fetch, callback, interval_s
        self._thread = threading.Thread(target=self._run, daemon=True, name="challenge-assist")
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                value = self._fetch()
            except ApiError as exc:
                value = {"status": "unavailable", "reason": exc.code or "network", "pose": None}
                terminal = not exc.retryable
            else:
                terminal = value is None or value.get("status") in ("ended", "disabled")
            if not self._stop.is_set():
                self._callback(value)
            if terminal or self._stop.wait(self._interval):
                return

    def close(self):
        self._stop.set()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=4)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
