"""``drone-challenge``: pair once, then send, upload or mark events.

    drone-challenge login team03               # once per computer: team password -> team key
    drone-challenge courses                    # released courses
    drone-challenge session new [--course NAME] # own session (default: active challenge course)
    drone-challenge pair                       # or: paste a pairing code (from QR / web page)
    drone-challenge status                     # what is paired, is the access valid
    drone-challenge ros2 /navigation/pose      # forward a ROS2 pose topic live
    drone-challenge upload flight-<session>    # hand in a recorded flight afterwards
    drone-challenge event switch               # mark an event now (start|takeoff|switch|stop)

The token never appears on a command line: it comes from the stored pairing or the
DRONE_LIVE_* environment variables.
"""

from __future__ import annotations

import argparse
import getpass
import json
import os
import sys
import time
from pathlib import Path

from diha_participant.client import ApiClient, ApiError

from . import __version__, pairing
from .ros2 import DEFAULT_TOPIC

KINDS = {"start": "system_start", "takeoff": "takeoff", "switch": "switch", "stop": "flight_end"}
TOKEN_ENV = "_DRONE_CHALLENGE_TOKEN"  # in-process only, handed to the forwarder


def _access(parser) -> dict:
    try:
        access = pairing.access()
    except ValueError as exc:
        parser.error(str(exc))
    if not access:
        parser.error("nicht gekoppelt: zuerst `drone-challenge pair` (oder DRONE_LIVE_* setzen)")
    return access


def _forward(access: dict, extra: list[str]) -> int:
    from diha_participant.forward import main as forward_main

    os.environ[TOKEN_ENV] = access["token"]
    return forward_main(
        ["--api", access["api"], "--session", access["session"], "--token-env", TOKEN_ENV, *extra]
    )


def cmd_pair(args, parser) -> int:
    if args.forget:
        print("Kopplung entfernt." if pairing.forget() else "Es war nichts gekoppelt.")
        return 0
    if args.file:
        text = Path(args.file).read_text(encoding="utf-8")
    elif not sys.stdin.isatty():
        text = sys.stdin.read()
    else:
        text = getpass.getpass("Pairing-Code einfügen (wird nicht angezeigt): ")
    try:
        data = pairing.parse(text.strip())
    except ValueError as exc:
        parser.error(str(exc))
    path = pairing.save(data)
    print(f"Gekoppelt: Session {data['session']} auf {data['api']}")
    if data.get("team_id"):
        print(
            f"Team {data['team_id']}, Versuch {data.get('trial_id', '-')}, Frame {data.get('frame', '-')}"
        )
    print(f"Gespeichert in {path} (nur für dich lesbar)")
    return 0


def cmd_login(args, parser) -> int:
    from . import team

    password = getpass.getpass(f"Passwort für {args.username}: ")
    try:
        account = team.login(args.username, password, args.api, args.device)
    except team.TeamError as exc:
        print(f"Anmeldung fehlgeschlagen: {exc}", file=sys.stderr)
        return 1
    print(f"Angemeldet als {account['name']} ({account['id']}) auf {args.api}")
    print(f"Team-Schlüssel gespeichert in {team.path()} (nur für dich lesbar)")
    return 0


def cmd_courses(args, parser) -> int:
    from . import team

    try:
        items = team.courses()
    except team.TeamError as exc:
        parser.error(str(exc))
    if not items:
        print("Keine freigegebenen Kurse.")
    for c in items:
        mark = "  ← aktiver Challenge-Kurs" if c.get("active") else ""
        print(
            f"{c['course_id']:<32} {c['name']} (Version {c['version']}, "
            f"{c['length_m']:.1f} m){mark}"
        )
    return 0


def cmd_session(args, parser) -> int:
    from . import team

    try:
        answer = team.new_session(args.course, args.trial, args.start)
    except team.TeamError as exc:
        print(f"Session nicht angelegt: {exc}", file=sys.stderr)
        return 1
    config = answer["session"]["config"]
    print(
        f"Session {answer['session']['id']} angelegt: Team {config['team_id']}, "
        f"Versuch {config['trial_id']}, Kurs {config['course_id']} v{config['course_version']}"
    )
    print("Ist jetzt die aktuelle Kopplung: Drone(), ros2, upload und event verwenden sie.")
    return 0


def cmd_course(args, parser) -> int:
    from diha_participant.client import ApiClient, ApiError

    from .course import Course

    access = _access(parser)
    try:
        course = Course(ApiClient(access["api"], access["token"], access["session"]).course())
    except ApiError as exc:
        print(f"Kurs nicht abrufbar: {exc}", file=sys.stderr)
        return 1
    hall = course.hall_waypoints()
    if args.hall and hall is None:
        status = (course.anchor or {}).get("status", "unbekannt")
        print(
            f"Noch nicht in Hallenkoordinaten (Anker: {status}). Verankert wird, sobald das "
            "Kamerasystem die Drohne beim Start-Ereignis gemessen hat.",
            file=sys.stderr,
        )
        return 1
    points = hall if args.hall else course.waypoints
    if args.json:
        print(json.dumps({"course_id": course.course_id, "version": course.version,
                          "frame": "hall" if args.hall else "start", "waypoints": points}))
        return 0
    name = course.name or "eigene Sollroute"
    frame = "Hallenkoordinaten" if args.hall else "relativ zur Startpose (y = vorne beim Start)"
    print(f"{name} (Version {course.version}), {frame}, Meter:")
    for i, w in enumerate(points):
        print(f"{i:>3}  x {w['x']:7.3f}  y {w['y']:7.3f}  z {w['z']:6.3f}  {w['segment']}")
    if not args.hall:
        print("Hallenkoordinaten: " + ("verfügbar (--hall)" if hall else "nach der Startmessung"))
    return 0


def cmd_status(args, parser) -> int:
    try:
        access = pairing.access()
    except ValueError as exc:
        parser.error(str(exc))
    if not access:
        print(
            "Nicht gekoppelt: Drone() zeichnet nur lokal auf. Koppeln mit `drone-challenge pair`."
        )
        return 1
    source = "Umgebungsvariablen" if pairing.env_access() else str(pairing.config_path())
    print(f"Session {access['session']} auf {access['api']} (aus {source})")
    try:
        offset, rtt = ApiClient(access["api"], access["token"], access["session"]).measure_clock(4)
    except ApiError as exc:
        print(f"Zugang NICHT gültig oder Server nicht erreichbar: {exc}")
        return 1
    print(
        f"Zugang gültig. Uhrversatz zum Server {offset / 1e6:+.1f} ms (Laufzeit {rtt / 1e6:.1f} ms)"
    )
    return 0


def cmd_ros2(args, parser) -> int:
    extra = ["--ros2-pose-topic", args.topic, "--ros2-event-topic", args.events]
    if args.frame:
        extra += ["--expected-frame", args.frame]
    if args.no_clock:
        extra += ["--clock-sync-interval", "0"]
    return _forward(_access(parser), extra)


def cmd_upload(args, parser) -> int:
    folder = Path(args.folder)
    csv_path = folder / "team.csv" if folder.is_dir() else folder
    if not csv_path.exists():
        parser.error(f"{csv_path} nicht gefunden (Ordner einer Aufzeichnung angeben)")
    return _forward(_access(parser), ["--csv", str(csv_path)])


def cmd_event(args, parser) -> int:
    access = _access(parser)
    stamp = args.at if args.at is not None else f"{time.time():.6f}"
    # A sequence range of its own, so it never collides with a running uploader's numbers.
    seq = 2**61 + time.time_ns() // 1000
    client = ApiClient(access["api"], access["token"], access["session"])
    try:
        answer = client.post_events([{"seq": seq, "kind": KINDS[args.kind], "stamp_s": stamp}])
    except ApiError as exc:
        print(f"Ereignis nicht übertragen: {exc}", file=sys.stderr)
        return 1
    print(f"{KINDS[args.kind]} um {stamp} gemeldet ({answer.get('accepted', 0)} angenommen)")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="drone-challenge",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--version", action="version", version=f"drone-challenge {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("pair", help="Pairing-Code einmal übernehmen")
    p.add_argument("--file", help="Pairing-Code aus Datei statt Eingabe")
    p.add_argument("--forget", action="store_true", help="gespeicherte Kopplung löschen")
    p.set_defaults(run=cmd_pair)
    p = sub.add_parser("login", help="einmal pro Rechner mit dem Team-Konto anmelden")
    p.add_argument("username", help="Team-Benutzername, z. B. team03")
    p.add_argument("--api", default="https://drone-challenge.arkturian.com")
    p.add_argument("--device", default="sdk", help="Name dieses Rechners")
    p.set_defaults(run=cmd_login)
    p = sub.add_parser("courses", help="freigegebene Kurse anzeigen")
    p.set_defaults(run=cmd_courses)
    p = sub.add_parser("session", help="eigene Session anlegen")
    p.add_argument("action", choices=["new"])
    p.add_argument("--course", help="Kurs-ID oder Kursname (Standard: der aktive Challenge-Kurs)")
    p.add_argument("--trial", help="Versuchsname (Standard: FLUG-<Datum-Uhrzeit>)")
    p.add_argument("--start", choices=["system_start", "takeoff"], default="system_start")
    p.set_defaults(run=cmd_session)
    p = sub.add_parser("course", help="Kurs der aktuellen Session: Wegpunkte ab Start")
    p.add_argument("--hall", action="store_true", help="in Hallenkoordinaten (nach Startmessung)")
    p.add_argument("--json", action="store_true", help="als JSON ausgeben")
    p.set_defaults(run=cmd_course)
    p = sub.add_parser("status", help="Kopplung und Zugang prüfen")
    p.set_defaults(run=cmd_status)
    p = sub.add_parser("ros2", help="ROS2-Posen-Topic live übertragen")
    p.add_argument("topic")
    p.add_argument(
        "--events", default=DEFAULT_TOPIC, help=f"Ereignis-Topic (Standard {DEFAULT_TOPIC})"
    )
    p.add_argument("--frame", help="nur Posen mit diesem frame_id annehmen")
    p.add_argument(
        "--no-clock", action="store_true", help="kein Uhrabgleich (anderer Rechner stempelt)"
    )
    p.set_defaults(run=cmd_ros2)
    p = sub.add_parser("upload", help="aufgezeichneten Flug nachreichen")
    p.add_argument("folder", help="Ordner der Aufzeichnung (flight-…) oder team.csv")
    p.set_defaults(run=cmd_upload)
    p = sub.add_parser("event", help="Ereignis jetzt melden")
    p.add_argument("kind", choices=sorted(KINDS))
    p.add_argument("--at", help="Zeitpunkt in Unix-Sekunden statt jetzt")
    p.set_defaults(run=cmd_event)
    args = parser.parse_args(argv)
    return args.run(args, parser)


if __name__ == "__main__":
    raise SystemExit(main())
