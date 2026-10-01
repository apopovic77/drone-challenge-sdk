"""Optional ROS2 adapter: forward pose messages and event strings to the live API.

Supported pose types: geometry_msgs/PoseStamped, geometry_msgs/TransformStamped,
geometry_msgs/PoseWithCovarianceStamped and nav_msgs/Odometry. The type is read from
the ROS2 graph, so the forwarder can point at whatever topic the mocap driver offers.

Requires a sourced ROS2 environment (``rclpy``, ``geometry_msgs``, ``std_msgs``).
The conversion functions work without ROS2 so they can be tested anywhere.
Topic names, QoS and the frame of the published pose must match what the organiser
and the hall operator confirm; nothing here guesses them.
"""

from __future__ import annotations

import importlib
import math
import re
import time

from .logger import LIVE_EVENT_KINDS, Event, Pose
from .uploader import TelemetryUploader


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stamp_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# Decimal seconds as accepted by the API: keeps the exact value instead of a float.
STAMP_TEXT = re.compile(r"[+-]?[0-9]{1,10}(?:\.[0-9]{1,9})?")

# ROS2 type name -> (python module, class)
POSE_TYPES = {
    "geometry_msgs/msg/PoseStamped": ("geometry_msgs.msg", "PoseStamped"),
    "geometry_msgs/msg/TransformStamped": ("geometry_msgs.msg", "TransformStamped"),
    "geometry_msgs/msg/PoseWithCovarianceStamped": (
        "geometry_msgs.msg",
        "PoseWithCovarianceStamped",
    ),
    "nav_msgs/msg/Odometry": ("nav_msgs.msg", "Odometry"),
}


def pose_parts(msg):
    """(position, orientation) of any supported pose message."""
    if hasattr(msg, "transform"):
        return msg.transform.translation, msg.transform.rotation
    pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
    return pose.position, pose.orientation


def pose_from_msg(msg, stamp_s: float | None = None) -> Pose:
    """Any supported pose message -> Pose; header time (measurement time) unless given."""
    position, q = pose_parts(msg)
    stamp = msg.header.stamp
    return Pose(
        stamp_seconds(stamp) if stamp_s is None else stamp_s,
        position.x,
        position.y,
        position.z,
        yaw_from_quaternion(q.x, q.y, q.z, q.w),
        stamp_text=f"{stamp.sec}.{stamp.nanosec:09d}" if stamp_s is None else None,
    )


def resolve_pose_type(node, topic: str, timeout_s: float = 600.0):  # pragma: no cover - needs ROS2
    """Wait for the topic in the ROS2 graph and return the message class it carries.

    Mocap bridges are sometimes restarted during calibration, so this waits patiently.
    Once subscribed, DDS reconnects by itself when the publisher comes back.
    """
    import rclpy

    deadline = time.monotonic() + timeout_s
    next_note = time.monotonic() + 10
    while time.monotonic() < deadline:
        if time.monotonic() >= next_note:
            node.get_logger().info(f"waiting for {topic} ...")
            next_note += 10
        types = dict(node.get_topic_names_and_types()).get(topic)
        if types:
            for name in types:
                if name in POSE_TYPES:
                    module, cls = POSE_TYPES[name]
                    node.get_logger().info(f"{topic}: using {name}")
                    return getattr(importlib.import_module(module), cls)
            raise SystemExit(
                f"{topic} has unsupported type(s) {types}; supported: {sorted(POSE_TYPES)}"
            )
        rclpy.spin_once(node, timeout_sec=0.2)
    raise SystemExit(
        f"topic {topic} not found within {timeout_s:.0f} s; check ROS_DOMAIN_ID and `ros2 topic list`"
    )


def event_from_msg(msg, now_s: float) -> Event:
    """std_msgs/String ``kind`` or ``kind@stamp_s``; without a stamp the node clock is used.

    Sending the stamp keeps events on the same clock as the poses (e.g. the mocap clock)
    even when the forwarding computer's clock differs.
    """
    kind, _, stamp = msg.data.strip().partition("@")
    if kind not in LIVE_EVENT_KINDS:
        raise ValueError(f"unknown event kind {kind!r}")
    try:
        if not stamp:
            return Event(kind, now_s)
        return Event(kind, float(stamp), stamp_text=stamp if STAMP_TEXT.fullmatch(stamp) else None)
    except ValueError as exc:
        raise ValueError(f"invalid event stamp {stamp!r}") from exc


def run_forwarder(
    uploader: TelemetryUploader,
    pose_topic: str,
    event_topic: str | None = None,
    *,
    expected_frame: str | None = None,
    depth: int = 50,
    on_pose=None,
) -> None:  # pragma: no cover - needs a ROS2 installation
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class Forwarder(Node):
        def __init__(self) -> None:
            super().__init__(f"live_forward_{uploader.source}")
            # BEST_EFFORT subscribers receive from both best-effort and reliable publishers;
            # mocap drivers usually publish best-effort (sensor data QoS).
            qos = QoSProfile(
                depth=depth,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            # Subscribe to events first: resolving the pose type can take a while, and an
            # event published meanwhile (e.g. system_start) would otherwise be missed.
            if event_topic:
                self.create_subscription(String, event_topic, self.on_event, qos)
            self.seen_events: set[tuple[str, float]] = set()
            self.create_subscription(
                resolve_pose_type(self, pose_topic), pose_topic, self.on_pose, qos
            )
            self.last_stamp: float | None = None
            self.count = 0
            self.warned_zero_stamp = False

        def on_pose(self, msg) -> None:
            if expected_frame and msg.header.frame_id != expected_frame:
                self.get_logger().warning(f"ignored pose in frame {msg.header.frame_id!r}")
                return
            stamp = None
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                # Some drivers leave the header empty; fall back to the receive time.
                if not self.warned_zero_stamp:
                    self.get_logger().warning("header stamp is 0: using receive time instead")
                    self.warned_zero_stamp = True
                stamp = self.get_clock().now().nanoseconds * 1e-9
            pose = pose_from_msg(msg, stamp)
            if self.last_stamp is not None and pose.stamp_s <= self.last_stamp:
                self.get_logger().warning("ignored non-increasing pose stamp")
                return
            self.last_stamp = pose.stamp_s
            self.count += 1
            if self.count == 1 or self.count % 500 == 0:
                self.get_logger().info(
                    f"{self.count} poses, last ({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}) frame {msg.header.frame_id!r}"
                )
            uploader.publish(pose)
            if on_pose is not None:
                on_pose(pose)

        def on_event(self, msg) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            try:
                event = event_from_msg(msg, now)
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                return
            # Publishers may repeat stamped events so late subscribers still get them.
            if "@" in msg.data:
                key = (event.kind, event.stamp_s)
                if key in self.seen_events:
                    return
                self.seen_events.add(key)
            self.get_logger().info(f"event {event.kind} at {event.stamp_s:.3f}")
            uploader.record_event(event)

    rclpy.init()
    node = Forwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def event_text(kind: str, sec: int, nanosec: int) -> str:
    """The ``kind@stamp`` string the forwarder expects, with exact nanoseconds."""
    if kind not in LIVE_EVENT_KINDS:
        raise ValueError(f"unknown event kind {kind!r}")
    return f"{kind}@{int(sec)}.{int(nanosec):09d}"


class EventPublisher:  # pragma: no cover - needs a ROS2 installation
    """Lifecycle events for a team's ROS2 node in two lines::

        events = EventPublisher(self, "/drone/events")
        events.emit("switch")

    Each event is stamped once with the node clock (the same clock as the poses) and
    repeated unchanged every ``repeat_s`` seconds, so a forwarder that starts late still
    receives it; the forwarder drops the repeats.
    """

    def __init__(self, node, topic: str = "/drone/events", repeat_s: float = 2.0) -> None:
        from std_msgs.msg import String

        self._string = String
        self._node = node
        self._publisher = node.create_publisher(String, topic, 10)
        self._sent: list[str] = []
        node.create_timer(repeat_s, self._repeat)

    def emit(self, kind: str, stamp=None) -> str:
        """kind: system_start | takeoff | switch | flight_end.

        ``stamp``: when the event really happened, as ``builtin_interfaces/Time`` (e.g. a
        message header stamp) or ``rclpy.time.Time``; default is now on the node clock.
        Pass it whenever the event is reported later than it happened.
        """
        if stamp is None:
            stamp = self._node.get_clock().now()
        if hasattr(stamp, "to_msg"):
            stamp = stamp.to_msg()
        data = event_text(kind, stamp.sec, stamp.nanosec)
        self._sent.append(data)
        self._publisher.publish(self._string(data=data))
        return data

    def _repeat(self) -> None:
        for data in self._sent:
            self._publisher.publish(self._string(data=data))
