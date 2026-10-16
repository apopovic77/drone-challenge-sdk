"""``drone-challenge``: pair once, then send, upload or mark events.

    drone-challenge login team03               # once per computer: team password -> team key
    drone-challenge courses                    # released courses
    drone-challenge session new [--course NAME] # own session (default: active challenge course)
    drone-challenge session new --simulation --random [--targets 1]  # random obstacle course
    drone-challenge pair                       # or: paste a pairing code (from QR / web page)
    drone-challenge status                     # what is paired, is the access valid
    drone-challenge ros2 /navigation/pose      # forward a ROS2 pose topic live
    drone-challenge upload flight-<session>    # hand in a recorded flight afterwards
    drone-challenge event start                # before take-off; scoring begins here

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

KINDS = {
    "start": "system_start",
    "takeoff": "takeoff",
    "switch": "switch",
    "stop": "flight_end",
    "external_reference": "external_reference_used",
    # The server's own event names work too (16.10.: teams typed "event flight_end").
    "system_start": "system_start",
    "flight_end": "flight_end",
    "end": "flight_end",
    "external_reference_used": "external_reference_used",
}
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
        kind = (
            "Advanced: Start und Ziel, Weg selbst planen"
            if c.get("route_mode") == "environment"
            else f"{c['length_m']:.1f} m"
        )
        print(f"{c['course_id']:<32} {c['name']} (Version {c['version']}, {kind}){mark}")
    return 0


def cmd_session(args, parser) -> int:
    from . import team

    try:
        answer = team.new_session(
            args.course,
            args.trial,
            args.start,
            size_m=args.size_m,
            simulation=args.simulation,
            random=args.random,
            seed=args.seed,
            obstacles=args.obstacles,
            targets=args.targets,
        )
    except (team.TeamError, ValueError) as exc:
        print(f"Session nicht angelegt: {exc}", file=sys.stderr)
        return 1
    config = answer["session"]["config"]
    print(
        f"Session {answer['session']['id']} angelegt: Team {config['team_id']}, "
        f"Versuch {config['trial_id']}, Kurs {config['course_id']} v{config['course_version']}"
    )
    print("Ist jetzt die aktuelle Kopplung: Drone(), ros2, upload und event verwenden sie.")
    return 0


def cmd_finish(args, parser) -> int:
    from . import pairing, team

    session = args.session or (pairing.load() or {}).get("session")
    if not session:
        print("Keine Session angegeben und keine aktuelle Kopplung.", file=sys.stderr)
        return 1
    try:
        finished = team.finish(session)
    except team.TeamError as exc:
        print(f"Nicht ausgewertet: {exc}", file=sys.stderr)
        return 1
    result = (finished.get("finish") or {}).get("practice") or {}
    if not result:
        print(f"Session {session} abgeschlossen ({(finished.get('finish') or {}).get('mode')}).")
        return 0
    print(
        f"Weg {result['path_length_m']:.2f} m in {result['duration_s']:.1f} s, "
        f"{result['samples']} Posen; Abstand Ende–Start {result['closure_error_m']:.2f} m"
    )
    route = result.get("route_comparison") or {}
    if route.get("status") == "computed":
        print(
            f"Kurs: Soll {route['planned_length_m']:.2f} m, geflogen {route['flown_length_m']:.2f} m, "
            f"seitliche Abweichung mittel {route['cross_track_mean_m']:.3f} m, "
            f"max {route['cross_track_max_m']:.3f} m"
        )
    else:
        anchor = (result.get("route_anchor") or {}).get("reason") or "kein Startanker"
        print(f"Kein Kursvergleich ({anchor}).")
    print("Daten laden: drone-challenge data")
    return 0


def cmd_data(args, parser) -> int:
    import csv
    from pathlib import Path

    from . import pairing, team

    try:
        if args.list:
            for s in team.sessions():
                created = time.strftime("%d.%m. %H:%M", time.localtime(s["created_at_ns"] / 1e9))
                state = "offen" if s["state"] == "open" else "abgeschlossen"
                print(f"{s['id']}  {created}  {s['config']['trial_id']:<28} {state}")
            return 0
        session = args.session or (pairing.load() or {}).get("session")
        if not session:
            print("Keine Session angegeben und keine aktuelle Kopplung.", file=sys.stderr)
            return 1
        data = team.session_data(session)
    except team.TeamError as exc:
        print(f"Daten nicht geladen: {exc}", file=sys.stderr)
        return 1
    folder = Path(args.out or f"daten-{data['session']['config']['trial_id']}")
    folder.mkdir(parents=True, exist_ok=True)
    for name in ("team", "reference"):
        with (folder / f"{name}.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(data["columns"])
            writer.writerows(data[name])
    with (folder / "events.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["source", "kind", "stamp_s"])
        writer.writerows([e["source"], e["kind"], e["stamp_s"]] for e in data["events"])
    for name in ("course", "finish", "session"):
        (folder / f"{name}.json").write_text(
            json.dumps(data[name], indent=2, ensure_ascii=False), encoding="utf-8"
        )
    print(
        f"{folder}: {len(data['team'])} eigene Posen, {len(data['reference'])} OptiTrack-Posen, "
        f"{len(data['events'])} Ereignisse, Kurs und Ergebnis"
    )
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
                          "frame": "hall" if args.hall else "start", "waypoints": points,
                          "start_rule": course.start_rule, "assist_distance_m": course.assist_distance_m,
                          "motion_model": course.motion_model, "marker_height_m": course.marker_height_m, "vehicle_profile": course.vehicle_profile,
                          # Advanced training, course coordinates (the frame of the obstacles).
                          "route_mode": course.route_mode, "hall": course.hall,
                          "start_course": course.start_course, "goal_course": course.goal_course,
                          "goal_radius_m": (course.environment or {}).get("goal_radius"),
                          "obstacles": course.obstacles, "targets": course.targets}))
        return 0
    if course.route_mode == "environment":
        print(f"{course.name} (Version {course.version}), Kurskoordinaten, Meter:")
        print("Start  " + "  ".join(f"{v:7.3f}" for v in course.start_course))
        print("Ziel   " + "  ".join(f"{v:7.3f}" for v in course.goal_course)
              + f"   Radius {(course.environment or {}).get('goal_radius')}")
        for o in course.obstacles:
            print(f"Box {o['id']:<8} Mitte x {o['x']:6.2f} y {o['y']:6.2f}  "
                  f"{o['width']:.2f} × {o['depth']:.2f} × {o['height']:.2f} ab z {o.get('base_z', 0):.2f}")
        for tgt in course.targets:
            print(f"Bewegliches Ziel {tgt['id']}: {len(tgt['path'])} Punkte, {tgt['speed_mps']} m/s, "
                  f"{tgt['mode']}, Radius {tgt['radius_m']}, ab {tgt['start_delay_s']} s")
        print("Den Weg plant ihr selbst; alles als JSON: drone-challenge course --json")
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
    p.add_argument("--size-m", type=float, nargs=3, metavar=("WIDTH", "DEPTH", "HEIGHT"))
    p.add_argument(
        "--simulation",
        action="store_true",
        help="Simulator-Lauf: nie Hallendaten, Kurs am simulierten Start verankert",
    )
    p.add_argument(
        "--random",
        action="store_true",
        help="Zufallskurs mit Hindernissen (Advanced-Training): Start und Ziel vorgegeben",
    )
    p.add_argument("--seed", type=int, help="Zufallskurs wiederholen (gleicher Seed, gleicher Kurs)")
    p.add_argument("--obstacles", type=int, help="Anzahl Hindernisse im Zufallskurs (Standard 8)")
    p.add_argument("--targets", type=int, help="bewegliche Ziele im Zufallskurs, 0 bis 3")
    p.set_defaults(run=cmd_session)
    p = sub.add_parser("finish", help="eigene Test- oder Simulations-Session auswerten")
    p.add_argument("session", nargs="?", help="Session-ID (Standard: die aktuelle Kopplung)")
    p.set_defaults(run=cmd_finish)
    p = sub.add_parser("course", help="Kurs der aktuellen Session: Wegpunkte ab Start")
    p.add_argument("--hall", action="store_true", help="in Hallenkoordinaten (nach Startmessung)")
    p.add_argument("--json", action="store_true", help="als JSON ausgeben")
    p.set_defaults(run=cmd_course)
    p = sub.add_parser(
        "data", help="Daten einer abgeschlossenen Session laden (eigene Posen, OptiTrack, Ergebnis)"
    )
    p.add_argument("session", nargs="?", help="Session-ID (Standard: die aktuelle Kopplung)")
    p.add_argument("--list", action="store_true", help="eigene Sessions auflisten")
    p.add_argument("--out", help="Zielordner (Standard: daten-<Versuch>)")
    p.set_defaults(run=cmd_data)
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
