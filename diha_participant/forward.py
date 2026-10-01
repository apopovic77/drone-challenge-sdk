"""Command line forwarder for team estimates or the hall reference stream.

Examples::

    export LIVE_TOKEN=...   # token from session creation; never pass it as an argument
    python -m diha_participant.forward --api https://drone-challenge.arkturian.com \\
        --session <id> --source team --ros2-pose-topic /team/pose --ros2-event-topic /team/events

    python -m diha_participant.forward --api ... --session <id> --source reference \\
        --csv optitrack.csv --realtime
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

from .client import ApiClient, ApiError
from .timebase import StableClock
from .logger import Event, Pose
from .uploader import TelemetryUploader


def replay_csv(uploader: TelemetryUploader, path: Path, realtime: bool, on_pose=None) -> int:
    """Send a stamp,x,y,z,yaw file; with realtime the original pacing is kept."""
    count, first_stamp, started = 0, None, time.monotonic()
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.reader(handle):
            if not row or row[0] == "stamp":
                continue
            pose = Pose(*(float(v) for v in row[:5]), stamp_text=row[0].strip())
            if realtime:
                first_stamp = pose.stamp_s if first_stamp is None else first_stamp
                wait = (pose.stamp_s - first_stamp) - (time.monotonic() - started)
                if wait > 0:
                    time.sleep(wait)
            uploader.publish(pose)
            if on_pose is not None:
                on_pose(pose)
            count += 1
    return count


def send_events(uploader: TelemetryUploader, path: Path) -> int:
    """Send events recorded by SessionLogger (events.jsonl: {"kind", "stamp_s"} per line)."""
    count = 0
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                row = json.loads(line)
                stamp = row["stamp_s"]
                uploader.record_event(
                    Event(
                        row["kind"],
                        float(stamp),
                        stamp_text=stamp if isinstance(stamp, str) else None,
                    )
                )
                count += 1
    return count


def read_event_kinds(path: Path) -> set[str]:
    with path.open(encoding="utf-8") as handle:
        return {json.loads(line)["kind"] for line in handle if line.strip()}


def clock_loop(client: ApiClient, interval_s: float, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            offset, rtt = client.sync_clock()
            print(f"clock offset {offset / 1e6:+.3f} ms, rtt {rtt / 1e6:.3f} ms", file=sys.stderr)
        except ApiError as exc:
            print(f"clock sync failed: {exc}", file=sys.stderr)
        stop.wait(interval_s)


HALL_TEXT = {
    "bound": "hall stream -> session {session}",
    "unbound": "hall stream NOT stored: no session has started (takeoff/system_start) "
    "or been bound by the operator",
    "ambiguous": "hall stream NOT stored: several sessions are active ({sessions}); "
    "operator must bind one",
}


def hall_watch(uploader, stop: threading.Event) -> None:
    """Print whenever the server's binding of the hall stream changes."""
    last = None
    while not stop.wait(1.0):
        state = uploader.stats.hall
        if state and state != last:
            text = HALL_TEXT.get(state.get("state"), "hall stream: {state}")
            fields = {**state, "sessions": ", ".join(state.get("sessions", []))}
            print(text.format_map(fields), file=sys.stderr)
            last = state


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--api",
        default=os.environ.get("DRONE_LIVE_URL"),
        help="server base URL (default $DRONE_LIVE_URL)",
    )
    parser.add_argument(
        "--session",
        default=os.environ.get("DRONE_LIVE_SESSION"),
        help="session id (default $DRONE_LIVE_SESSION)",
    )
    parser.add_argument("--source", choices=("team", "reference"), default="team")
    parser.add_argument(
        "--hall",
        action="store_true",
        help="hall reference stream with the hall key ($DRONE_HALL_KEY): no session needed, "
        "the server feeds the one session that has started or that the operator bound",
    )
    parser.add_argument(
        "--token-env",
        default="DRONE_LIVE_TOKEN" if "DRONE_LIVE_TOKEN" in os.environ else "LIVE_TOKEN",
        help="environment variable holding the token (default DRONE_LIVE_TOKEN, else LIVE_TOKEN)",
    )
    parser.add_argument("--spool", type=Path, default=None, help="local spool directory")
    parser.add_argument(
        "--clock-sync-interval",
        type=float,
        default=None,
        help="seconds; 0 disables. Default 10 for live sources; 0 for --csv, because the "
        "uploading computer's clock says nothing about a recording made earlier or elsewhere",
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--ros2-pose-topic")
    group.add_argument("--csv", type=Path)
    group.add_argument(
        "--vrpn", metavar="HOST[:PORT]", help="read a VRPN tracker directly, without ROS2"
    )
    parser.add_argument("--vrpn-tracker", help="tracker / rigid body name for --vrpn")
    parser.add_argument("--vrpn-stamps", choices=("receive", "vrpn"), default="receive")
    parser.add_argument("--ros2-event-topic")
    parser.add_argument("--expected-frame")
    parser.add_argument("--realtime", action="store_true", help="CSV replay with original pacing")
    parser.add_argument(
        "--events",
        type=Path,
        help="events.jsonl for a CSV replay (default: events.jsonl next to the CSV; "
        "switch.txt next to the CSV is used when no switch event is present)",
    )
    parser.add_argument(
        "--fake-team",
        action="store_true",
        help="hall test without drone software: also send a drifting copy of the reference as team "
        "estimate; with --hall into the team session from `drone-challenge session new`, "
        "otherwise into --session with the team token in LIVE_TEAM_TOKEN",
    )
    parser.add_argument(
        "--fake-drift", type=float, default=0.003, help="fake team drift in m/s after switch"
    )
    args = parser.parse_args(argv)
    if args.hall:
        args.session, args.source = "hall", "reference"
        if "--token-env" not in (argv if argv is not None else sys.argv):
            args.token_env = "DRONE_HALL_KEY"
    if not args.api or not args.session:
        parser.error(
            "server URL and session are required (--api/--session or DRONE_LIVE_URL/DRONE_LIVE_SESSION)"
        )

    token = os.environ.get(args.token_env)
    if not token:
        parser.error(f"environment variable {args.token_env} is not set")
    # Streams stamped here on receipt (VRPN) use a time base without steps, and the clock
    # measurement uses the same one (see timebase.StableClock).
    stable = StableClock() if args.vrpn and args.vrpn_stamps == "receive" else None
    client = ApiClient(
        args.api, token, args.session, **({"clock": stable.time_ns} if stable else {})
    )
    spool = args.spool or Path(f"spool-{args.session}-{args.source}")
    uploader = TelemetryUploader(client, args.source, spool)
    fake_uploader, on_pose = None, None
    if args.fake_team:
        team_api, team_session = args.api, args.session
        team_token = os.environ.get("LIVE_TEAM_TOKEN")
        if args.hall:
            # The fake drone flies in the team's own session: the one `drone-challenge
            # session new` or `pair` stored (or DRONE_LIVE_* for the team).
            from drone_challenge import pairing

            team = pairing.load()
            if not team:
                parser.error("--hall --fake-team needs a team session: drone-challenge session new")
            team_api, team_session, team_token = team["api"], team["session"], team["token"]
        if args.source != "reference" or not team_token:
            parser.error("--fake-team needs --source reference and LIVE_TEAM_TOKEN")
        from .fake import FakeTeam

        fake_uploader = TelemetryUploader(
            ApiClient(team_api, team_token, team_session), "team", spool / "fake-team"
        )
        on_pose = FakeTeam(fake_uploader, drift_m_per_s=args.fake_drift)
        print("FAKE TEAM ACTIVE: team stream is a drifting copy of the reference", file=sys.stderr)
    stop = threading.Event()
    if args.clock_sync_interval is None:
        args.clock_sync_interval = 0.0 if args.csv else 10.0
    if args.clock_sync_interval > 0:
        threading.Thread(
            target=clock_loop, args=(client, args.clock_sync_interval, stop), daemon=True
        ).start()
    if args.hall:
        threading.Thread(target=hall_watch, args=(uploader, stop), daemon=True).start()
    try:
        if args.vrpn:
            from .vrpn import VrpnTrackerClient, run_forever

            if not args.vrpn_tracker:
                parser.error("--vrpn needs --vrpn-tracker")
            host, _, port = args.vrpn.partition(":")
            last = [None]

            def on_report(report) -> None:
                stamp = report.vrpn_time_s if args.vrpn_stamps == "vrpn" else report.received_s
                if last[0] is not None and stamp <= last[0]:
                    return
                last[0] = stamp
                qx, qy, qz, qw = report.quaternion
                pose = Pose(
                    stamp,
                    *report.position,
                    math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)),
                )
                uploader.publish(pose)
                if on_pose is not None:
                    on_pose(pose)

            client_vrpn = VrpnTrackerClient(
                host,
                args.vrpn_tracker,
                int(port or 3883),
                on_hint=lambda m: print(m, file=sys.stderr),
                **({"clock": stable.time} if stable else {}),
            )
            try:
                run_forever(client_vrpn, on_report, log=lambda m: print(m, file=sys.stderr))
            except KeyboardInterrupt:
                pass
        elif args.csv:
            # Pick up the files SessionLogger writes next to team.csv, so a plain
            # `--csv flight/team.csv` submits the whole flight.
            events_path = args.events or args.csv.with_name("events.jsonl")
            kinds: set[str] = set()
            if events_path.exists():
                kinds = read_event_kinds(events_path)
                print(f"queued {send_events(uploader, events_path)} events", file=sys.stderr)
            switch_path = args.csv.with_name("switch.txt")
            if "switch" not in kinds and switch_path.exists():
                stamp = switch_path.read_text(encoding="utf-8-sig").strip()
                uploader.record_event(Event("switch", float(stamp), stamp_text=stamp))
                print("queued switch from switch.txt", file=sys.stderr)
            sent = replay_csv(uploader, args.csv, args.realtime, on_pose)
            print(f"queued {sent} poses", file=sys.stderr)
        else:
            from .ros2 import run_forwarder

            run_forwarder(
                uploader,
                args.ros2_pose_topic,
                args.ros2_event_topic,
                expected_frame=args.expected_frame,
                on_pose=on_pose,
            )
    finally:
        stop.set()
        stats = uploader.close(timeout_s=30)
        if fake_uploader is not None:
            on_pose.finish()
            print(f"fake team stats: {fake_uploader.close(timeout_s=30)}", file=sys.stderr)
        print(f"upload stats: {stats}", file=sys.stderr)
    return 0 if uploader.pending == 0 and not stats.stopped else 1


if __name__ == "__main__":
    raise SystemExit(main())
