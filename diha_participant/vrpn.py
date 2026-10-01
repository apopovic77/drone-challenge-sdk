"""Minimal pure-Python VRPN tracker client (standard library only).

Reads ``vrpn_Tracker Pos_Quat`` reports of one tracker (rigid body) from a VRPN server
such as Vicon Tracker or OptiTrack Motive. No UDP channel is offered, so the server
sends everything over the TCP connection. Protocol reference: VRPN
``Format_Of_Protocol.txt`` and ``vrpn_Connection.C`` (``marshall_message``) and
``vrpn_Tracker.C`` (``encode_to``).

Wire format: after both sides exchange a 24-byte ASCII cookie, every message is a
24-byte header (length, time sec, time usec, sender, type, sequence; big-endian int32)
followed by the payload padded to 8 bytes. ``length`` is header plus unpadded payload.
"""

from __future__ import annotations

import socket
import struct
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass

COOKIE_SIZE = 24
HEADER = struct.Struct("!iiiiii")
SENDER_DESCRIPTION = -1
TYPE_DESCRIPTION = -2
POS_QUAT = "vrpn_Tracker Pos_Quat"
POS_QUAT_BODY = struct.Struct("!ii3d4d")


class VrpnError(RuntimeError):
    pass


@dataclass(frozen=True)
class TrackerReport:
    vrpn_time_s: float
    received_s: float
    sensor: int
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]  # x, y, z, w


def _pad(length: int) -> int:
    return length + (-length) % 8


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise VrpnError("VRPN server closed the connection")
        data += chunk
    return bytes(data)


def _description_name(payload: bytes) -> str:
    (length,) = struct.unpack_from("!i", payload)
    return payload[4 : 4 + length].split(b"\x00", 1)[0].decode("ascii", "replace")


class VrpnTrackerClient:
    """Iterate over tracker reports of one named tracker on a VRPN server."""

    def __init__(
        self,
        host: str,
        tracker: str,
        port: int = 3883,
        *,
        sensor: int | None = 0,
        timeout_s: float = 5.0,
        on_hint: Callable[[str], None] | None = None,
        clock: Callable[[], float] = time.time,
    ):
        self.host, self.port, self.tracker, self.sensor = host, port, tracker, sensor
        self.clock = clock  # receive stamps; see timebase.StableClock
        self.timeout_s = timeout_s
        self.server_version = ""
        self.senders: dict[int, str] = {}
        self.types: dict[int, str] = {}
        self.on_hint = on_hint

    def reports(self) -> Iterator[TrackerReport]:
        with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            cookie = _recv_exact(sock, COOKIE_SIZE)
            if not cookie.startswith(b"vrpn: ver. 07."):
                raise VrpnError(f"not a compatible VRPN server: {cookie!r}")
            self.server_version = cookie[:16].decode("ascii")
            # Mirror the server's version; log mode 0 (no remote logging).
            sock.sendall(cookie[:16] + b"  0" + b"\x00" * (COOKIE_SIZE - 19))
            sock.settimeout(None)
            last_match = time.monotonic()
            while True:
                if self.on_hint and time.monotonic() - last_match > 5:
                    self.on_hint(f"no reports for {self.tracker!r} yet; trackers seen: {self.known_trackers()}")
                    last_match = time.monotonic()
                length, sec, usec, sender, msg_type, _seq = HEADER.unpack(_recv_exact(sock, HEADER.size))
                payload_len = length - HEADER.size
                if payload_len < 0:
                    raise VrpnError(f"corrupt VRPN header (length {length})")
                body = _recv_exact(sock, _pad(payload_len))[:payload_len]
                received = self.clock()
                if msg_type == SENDER_DESCRIPTION:
                    self.senders[sender] = _description_name(body)
                elif msg_type == TYPE_DESCRIPTION:
                    self.types[sender] = _description_name(body)
                elif (
                    msg_type >= 0
                    and self.types.get(msg_type) == POS_QUAT
                    and self.senders.get(sender) == self.tracker
                    and len(body) >= POS_QUAT_BODY.size
                ):
                    values = POS_QUAT_BODY.unpack_from(body)
                    if self.sensor is not None and values[0] != self.sensor:
                        continue
                    last_match = time.monotonic()
                    yield TrackerReport(sec + usec * 1e-6, received, values[0], values[2:5], values[5:9])

    def known_trackers(self) -> list[str]:
        return sorted(set(self.senders.values()))


def run_forever(
    client: VrpnTrackerClient,
    on_report: Callable[[TrackerReport], None],
    *,
    log: Callable[[str], None] = print,
    retry_s: float = 2.0,
) -> None:
    """Keep reading; reconnect after errors. Warns if the tracker name never appears."""
    while True:
        count = 0
        try:
            log(f"connecting to VRPN {client.host}:{client.port}, tracker {client.tracker!r}")
            for report in client.reports():
                count += 1
                if count == 1:
                    log(f"receiving {client.tracker!r} from {client.server_version}")
                on_report(report)
                if count % 1000 == 0:
                    log(f"{count} reports, last position {tuple(round(v, 3) for v in report.position)}")
        except (OSError, VrpnError) as exc:
            log(f"VRPN connection lost: {exc}")
        time.sleep(retry_s)


def main(argv: list[str] | None = None) -> int:
    """Quick check: ``python -m diha_participant.vrpn <OptiTrack-IP> <rigid-body>`` prints 5 reports."""
    import argparse

    parser = argparse.ArgumentParser(description="Print a few VRPN tracker reports")
    parser.add_argument("server", help="HOST or HOST:PORT")
    parser.add_argument("tracker")
    parser.add_argument("--count", type=int, default=5)
    args = parser.parse_args(argv)
    host, _, port = args.server.partition(":")
    client = VrpnTrackerClient(host, args.tracker, int(port or 3883), on_hint=print)
    for i, report in enumerate(client.reports(), 1):
        print(
            f"{args.tracker}: pos {tuple(round(v, 4) for v in report.position)} "
            f"quat {tuple(round(v, 4) for v in report.quaternion)} "
            f"server-time {report.vrpn_time_s:.3f} receive-time {report.received_s:.3f}"
        )
        if i >= args.count:
            break
    print(f"server {client.server_version}, trackers: {client.known_trackers()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
