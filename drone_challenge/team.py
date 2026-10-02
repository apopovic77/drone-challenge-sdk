"""Team access for creating sessions: one login per computer, then sessions by course.

``drone-challenge login`` exchanges the team password once for a team key, stored in
``~/.config/drone-challenge/team.json`` (0600). With it the computer lists the released
courses and creates its own session before each flight, without anyone handing out codes.
"""

from __future__ import annotations

import json
import os
import tempfile
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from . import pairing

DEFAULT_API = "https://drone-challenge.arkturian.com"


class TeamError(RuntimeError):
    pass


def path() -> Path:
    return pairing.config_path().with_name("team.json")


def load() -> dict | None:
    try:
        data = json.loads(path().read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) and data.get("key") and data.get("api") else None


def _save(data: dict) -> Path:
    target = path()
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(dir=target.parent, prefix=".team.")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)
    os.chmod(temp, 0o600)
    os.replace(temp, target)
    return target


def _call(api: str, method: str, route: str, body: dict | None = None, key: str | None = None):
    parsed = urllib.parse.urlparse(api)
    if parsed.scheme != "https" and parsed.hostname not in ("127.0.0.1", "localhost", "::1"):
        raise TeamError("Server-Adresse muss mit https:// beginnen")
    headers = {"Content-Type": "application/json"}
    if key:
        headers["Authorization"] = f"Bearer {key}"
    request = urllib.request.Request(
        f"{api.rstrip('/')}/api/v1/sdk/{route}",
        data=json.dumps(body).encode() if body is not None else None,
        headers=headers,
        method=method,
    )
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return json.loads(response.read().decode())
    except urllib.error.HTTPError as exc:
        try:
            error = json.loads(exc.read().decode()).get("error")
        except ValueError:
            error = None
        message = error.get("message") if isinstance(error, dict) else error
        raise TeamError(message or f"Server antwortet mit HTTP {exc.code}") from exc
    except (urllib.error.URLError, OSError) as exc:
        raise TeamError(f"Server nicht erreichbar: {exc}") from exc


def login(username: str, password: str, api: str = DEFAULT_API, device: str = "sdk") -> dict:
    answer = _call(
        api, "POST", "key", {"username": username, "password": password, "device": device}
    )
    _save(
        {
            "api": api,
            "team": answer["team"]["id"],
            "key": answer["key"],
            "expires_at": answer["expires_at"],
        }
    )
    return answer["team"]


def _require() -> dict:
    data = load()
    if not data:
        raise TeamError("nicht angemeldet: zuerst `drone-challenge login <team>`")
    return data


def courses() -> list[dict]:
    data = _require()
    return _call(data["api"], "GET", "courses", key=data["key"])["items"]


def new_session(
    course: str | None = None, trial_id: str | None = None, start: str = "system_start"
) -> dict:
    """Create a session for a released course (None: the active challenge course the
    organiser set); it becomes the current pairing."""
    data = _require()
    body = {"start_event_kind": start}
    if course:
        body["course"] = course
    if trial_id:
        body["trial_id"] = trial_id
    answer = _call(data["api"], "POST", "sessions", body, key=data["key"])
    pairing.save(pairing.parse(json.dumps(answer["pairing"])))
    return answer
