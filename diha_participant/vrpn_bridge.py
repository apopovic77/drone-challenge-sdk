"""VRPN -> ROS2 bridge: publish one VRPN tracker as geometry_msgs/PoseStamped.

For halls whose motion capture system (Vicon Tracker, OptiTrack Motive) streams VRPN
but offers no ROS2 topics. Runs on the organiser computer::

    python -m diha_participant.vrpn_bridge --server <OptiTrack-IP> --tracker <rigid-body>

publishes ``/vrpn/dummy1/pose``. By default messages are stamped with this computer's
receive time (like vrpn_mocap); ``--stamps vrpn`` uses the server's report time.
Requires a sourced ROS2 environment.
"""

from __future__ import annotations

import argparse
import threading


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - needs ROS2
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server", required=True, help="VRPN server host or IP")
    parser.add_argument("--port", type=int, default=3883)
    parser.add_argument("--tracker", required=True, help="rigid body / tracker name on the VRPN server")
    parser.add_argument("--sensor", type=int, default=0)
    parser.add_argument("--topic", help="default /vrpn/<tracker>/pose")
    parser.add_argument("--frame-id", default="vicon")
    parser.add_argument("--stamps", choices=("receive", "vrpn"), default="receive")
    args = parser.parse_args(argv)
    topic = args.topic or f"/vrpn/{args.tracker}/pose"

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    from .vrpn import VrpnTrackerClient, run_forever

    rclpy.init()
    node = Node("vrpn_bridge")
    qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(PoseStamped, topic, qos)
    log = node.get_logger()

    def publish(report) -> None:
        stamp = report.vrpn_time_s if args.stamps == "vrpn" else report.received_s
        msg = PoseStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(stamp * 1e9), 1_000_000_000)
        msg.header.frame_id = args.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = report.position
        q = report.quaternion
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        publisher.publish(msg)

    client = VrpnTrackerClient(args.server, args.tracker, args.port, sensor=args.sensor, on_hint=log.warning)
    threading.Thread(target=run_forever, args=(client, publish), kwargs={"log": log.info}, daemon=True).start()
    log.info(f"publishing VRPN {args.tracker!r} from {args.server}:{args.port} on {topic} ({args.stamps} stamps)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
