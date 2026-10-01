"""Pairing: the session access (server, session, team token) stored once per computer.

``drone-challenge pair`` saves the pairing code (the JSON inside the session's QR code) to
``~/.config/drone-challenge/pairing.json`` with permissions 0600. ``Drone()`` and every
command use it, so neither code nor command lines ever contain the token.
Environment variables (DRONE_LIVE_URL, DRONE_LIVE_SESSION, DRONE_LIVE_TOKEN) still win.
"""

from __future__ import annotations

import json
import os
import tempfile
import urllib.parse
from pathlib import Path

ENV_CONFIG = "DRONE_CHALLENGE_CONFIG"
REQUIRED = ("api", "session", "token")


def config_path() -> Path:
    if os.environ.get(ENV_CONFIG):
        return Path(os.environ[ENV_CONFIG])
    base = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    return base / "drone-challenge" / "pairing.json"


def parse(text: str) -> dict:
    """Validate the pairing code from the QR code / web page."""
    try:
        data = json.loads(text)
    except ValueError as exc:
        raise ValueError(
            "Pairing-Code ist kein gültiges JSON (Inhalt des QR-Codes einfügen)"
        ) from exc
    if not isinstance(data, dict) or data.get("type") != "drone-challenge-pairing":
        raise ValueError("kein Drone-Challenge-Pairing-Code")
    missing = [k for k in REQUIRED if not data.get(k)]
    if missing:
        raise ValueError(f"Pairing-Code unvollständig: {', '.join(missing)} fehlt")
    api = urllib.parse.urlparse(str(data["api"]))
    local = api.scheme == "http" and api.hostname in ("127.0.0.1", "localhost", "::1")
    if api.scheme != "https" and not local:
        raise ValueError("Server-Adresse muss mit https:// beginnen")
    keep = ("type", "v", "api", "session", "token", "team_id", "trial_id", "frame")
    return {k: data[k] for k in keep if k in data}


def save(pairing: dict) -> Path:
    path = config_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(dir=path.parent, prefix=".pairing.")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        json.dump(pairing, handle, indent=2)
    os.chmod(temp, 0o600)
    os.replace(temp, path)
    return path


def load() -> dict | None:
    path = config_path()
    if not path.exists():
        return None
    try:
        return parse(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def forget() -> bool:
    path = config_path()
    if path.exists():
        path.unlink()
        return True
    return False


def env_access() -> dict | None:
    """Server, session and token from DRONE_LIVE_* environment variables, if set."""
    env = {
        "api": os.environ.get("DRONE_LIVE_URL"),
        "session": os.environ.get("DRONE_LIVE_SESSION"),
        "token": os.environ.get("DRONE_LIVE_TOKEN"),
    }
    if not any(env.values()):
        return None
    if not all(env.values()):
        raise ValueError("DRONE_LIVE_URL, DRONE_LIVE_SESSION und DRONE_LIVE_TOKEN zusammen setzen")
    return env


def access() -> dict | None:
    """Environment variables first, then the stored pairing; None = offline."""
    return env_access() or load()
