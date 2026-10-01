"""ROS2 stand-in for a drone's navigation software, for hall tests without a real drone.

It subscribes to the OptiTrack pose of a hand-carried marker body and republishes it
as the "team estimate" on its own topic, with a slow drift after a simulated GNSS
switch-off, plus lifecycle events on an event topic. The live forwarder then reads
these topics exactly as it would read a real team's topics::

    python -m diha_participant.fake_drone --optitrack-topic /optitrack/body
    python -m diha_participant.forward ... --source team \\
        --ros2-pose-topic /team/pose --ros2-event-topic /team/events

Requires a sourced ROS2 environment. It never commands any vehicle.
"""

from __future__ import annotations

import argparse
import random


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - needs ROS2
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--optitrack-topic", required=True, help="pose topic of the marker body (PoseStamped, TransformStamped, Odometry)"
    )
    parser.add_argument("--pose-topic", default="/team/pose")
    parser.add_argument("--event-topic", default="/team/events")
    parser.add_argument("--switch-after", type=float, default=5.0, help="seconds until simulated GNSS switch-off")
    parser.add_argument("--drift", type=float, default=0.003, help="drift in m/s after the switch")
    parser.add_argument("--noise", type=float, default=0.005, help="position noise in m")
    parser.add_argument("--frame-id", default=None, help="override frame_id (default: keep OptiTrack frame)")
    args = parser.parse_args(argv)

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from .ros2 import pose_parts, resolve_pose_type

    class FakeDrone(Node):
        def __init__(self) -> None:
            super().__init__("fake_drone_estimator")
            listen = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
            publish = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
            source_type = resolve_pose_type(self, args.optitrack_topic)
            self.create_subscription(source_type, args.optitrack_topic, self.on_pose, listen)
            self.pose_pub = self.create_publisher(PoseStamped, args.pose_topic, publish)
            self.event_pub = self.create_publisher(String, args.event_topic, publish)
            self.start: float | None = None
            self.switched = False
            self.random = random.Random(1)
            self.sent_events: list[str] = []
            # Repeat stamped events so a forwarder that subscribes late still receives them;
            # forwarders drop repeats with the same kind and stamp.
            self.create_timer(2.0, self.repeat_events)
            self.get_logger().info(f"listening on {args.optitrack_topic}, publishing {args.pose_topic}")

        def event(self, kind: str, stamp_s: float) -> None:
            # kind@stamp keeps the event on the mocap clock, like the poses.
            data = f"{kind}@{stamp_s:.9f}"
            self.sent_events.append(data)
            self.event_pub.publish(String(data=data))
            self.get_logger().info(f"event {kind} at {stamp_s:.3f}")

        def repeat_events(self) -> None:
            for data in self.sent_events:
                self.event_pub.publish(String(data=data))

        def on_pose(self, msg) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self.start is None:
                self.start = stamp
                self.event("system_start", stamp)
            elapsed = stamp - self.start
            if not self.switched and elapsed >= args.switch_after:
                self.switched = True
                self.event("switch", stamp)
            drift = args.drift * max(elapsed - args.switch_after, 0.0)
            out = PoseStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = args.frame_id or msg.header.frame_id
            position, orientation = pose_parts(msg)
            out.pose.orientation.x = orientation.x
            out.pose.orientation.y = orientation.y
            out.pose.orientation.z = orientation.z
            out.pose.orientation.w = orientation.w
            g = self.random.gauss
            out.pose.position.x = position.x + drift + g(0, args.noise)
            out.pose.position.y = position.y - 0.5 * drift + g(0, args.noise)
            out.pose.position.z = position.z + g(0, args.noise)
            self.pose_pub.publish(out)

    rclpy.init()
    node = FakeDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # mark the flight end in the Live view; Ctrl+C may already have stopped ROS2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
