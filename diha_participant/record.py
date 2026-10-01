"""Record a pose topic to the Challenge 1A files, like a team without a live link.

Writes ``team.csv`` (stamp,x,y,z,yaw), ``events.jsonl`` and ``switch.txt`` into a
directory when stopped with Ctrl+C. The files can then be submitted after the flight::

    python -m diha_participant.record --pose-topic /team/pose --event-topic /team/events --out flight-01
    python -m diha_participant.forward ... --source team --csv flight-01/team.csv --events flight-01/events.jsonl

Requires a sourced ROS2 environment.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main(argv: list[str] | None = None) -> int:  # pragma: no cover - needs ROS2
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pose-topic", required=True)
    parser.add_argument("--event-topic")
    parser.add_argument("--out", type=Path, required=True, help="output directory")
    parser.add_argument("--frame-id", default="vicon")
    parser.add_argument("--team-id", default="OFFLINE-TEAM")
    parser.add_argument("--trial-id", default="TRIAL-01")
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    from .logger import SessionLogger
    from .ros2 import event_from_msg, pose_from_msg, resolve_pose_type

    log = SessionLogger(
        args.out, frame_id=args.frame_id, reference_point="body_origin", team_id=args.team_id, trial_id=args.trial_id
    )

    class Recorder(Node):
        def __init__(self) -> None:
            super().__init__("offline_recorder")
            qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.seen: set[tuple[str, float]] = set()
            if args.event_topic:
                self.create_subscription(String, args.event_topic, self.on_event, qos)
            self.create_subscription(resolve_pose_type(self, args.pose_topic), args.pose_topic, self.on_pose, qos)
            self.count = 0

        def on_pose(self, msg) -> None:
            try:
                log.publish_estimate(pose_from_msg(msg))
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                return
            self.count += 1
            if self.count == 1 or self.count % 500 == 0:
                self.get_logger().info(f"{self.count} poses recorded")

        def on_event(self, msg) -> None:
            try:
                event = event_from_msg(msg, self.get_clock().now().nanoseconds * 1e-9)
                if (event.kind, event.stamp_s) in self.seen:
                    return  # repeated stamped event
                self.seen.add((event.kind, event.stamp_s))
                log.record_event(event.kind, event.stamp_s)
                self.get_logger().info(f"event {event.kind}")
            except ValueError as exc:
                self.get_logger().warning(str(exc))

    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        files = log.close()
        print(f"written: {files}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
