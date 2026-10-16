"""VRPN client against an in-process server speaking the VRPN wire format."""

import socket
import struct
import threading

from diha_participant.vrpn import HEADER, POS_QUAT_BODY, VrpnTrackerClient


def message(sender, msg_type, payload, sec=1790000000, usec=500000):
    padded = payload + b"\x00" * ((-len(payload)) % 8)
    return HEADER.pack(HEADER.size + len(payload), sec, usec, sender, msg_type, 0) + padded


def description(name):
    raw = name.encode() + b"\x00"
    return struct.pack("!i", len(raw)) + raw


def serve(listener, reports):
    conn, _ = listener.accept()
    with conn:
        conn.sendall(b"vrpn: ver. 07.33  0".ljust(24, b"\x00"))
        assert conn.recv(24).startswith(b"vrpn: ver. 07.33")
        conn.sendall(message(3, -1, description("other")) + message(5, -1, description("dummy1")))
        conn.sendall(message(7, -2, description("vrpn_Tracker Pos_Quat")))
        conn.sendall(message(3, 7, POS_QUAT_BODY.pack(0, 0, 9, 9, 9, 0, 0, 0, 1)))  # other tracker
        for i in range(reports):
            body = POS_QUAT_BODY.pack(0, 0, 0.1 * i, -0.2, 1.5, 0, 0, 0.7071067811865476, 0.7071067811865476)
            conn.sendall(message(5, 7, body, usec=1000 * i))


def test_client_decodes_named_tracker_only():
    listener = socket.create_server(("127.0.0.1", 0))
    port = listener.getsockname()[1]
    threading.Thread(target=serve, args=(listener, 3), daemon=True).start()
    client = VrpnTrackerClient("127.0.0.1", "dummy1", port)
    reports = []
    for report in client.reports():
        reports.append(report)
        if len(reports) == 3:
            break
    assert [r.position for r in reports] == [(0.0, -0.2, 1.5), (0.1, -0.2, 1.5), (0.2, -0.2, 1.5)]
    assert reports[0].quaternion[3] == 0.7071067811865476
    assert reports[1].vrpn_time_s == 1790000000.001
    assert client.server_version == "vrpn: ver. 07.33"
    assert client.known_trackers() == ["dummy1", "other"]
    listener.close()


def test_tracker_can_switch_while_connected_and_matches_team_ids():
    """--vrpn-tracker auto: the bridge switches rigid bodies without reconnecting."""
    listener = socket.create_server(("127.0.0.1", 0))
    port = listener.getsockname()[1]

    def two_bodies(listener):
        conn, _ = listener.accept()
        with conn:
            conn.sendall(b"vrpn: ver. 07.33  0".ljust(24, b"\x00"))
            conn.recv(24)
            conn.sendall(message(3, -1, description("Team01")) + message(5, -1, description("Team03")))
            conn.sendall(message(7, -2, description("vrpn_Tracker Pos_Quat")))
            for i in range(40):
                conn.sendall(message(3, 7, POS_QUAT_BODY.pack(0, 0, 1, 1, 1, 0, 0, 0, 1), usec=i))
                conn.sendall(message(5, 7, POS_QUAT_BODY.pack(0, 0, 3, 3, 3, 0, 0, 0, 1), usec=i))

    threading.Thread(target=two_bodies, args=(listener,), daemon=True).start()
    client = VrpnTrackerClient("127.0.0.1", "team01", port)
    seen = []
    for report in client.reports():
        seen.append(report.position[0])
        if len(seen) == 2:
            client.tracker = "team03"  # hall now bound to team03
        if len(seen) == 4:
            break
    assert seen == [1.0, 1.0, 3.0, 3.0]
    listener.close()
