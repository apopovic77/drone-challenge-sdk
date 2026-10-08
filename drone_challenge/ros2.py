"""ROS2 helper for the flight events, in the team's own node::

    from drone_challenge.ros2 import Challenge

    challenge = Challenge(self)      # in __init__ of your node
    challenge.start()                # system_start: without it the flight is not scored
    challenge.takeoff()              # optional
    challenge.stop()                 # end of flight

Each call is stamped once with the node clock (or ``stamp=msg.header.stamp``) and
repeated, so ``drone-challenge ros2 <topic>`` receives it even when started later.
"""

from __future__ import annotations

from diha_participant.ros2 import EventPublisher

DEFAULT_TOPIC = "/drone_challenge/events"


class Challenge:  # pragma: no cover - needs a ROS2 installation
    def __init__(self, node, topic: str = DEFAULT_TOPIC) -> None:
        self._events = EventPublisher(node, topic)

    def start(self, stamp=None) -> str:
        return self._events.emit("system_start", stamp)

    def takeoff(self, stamp=None) -> str:
        return self._events.emit("takeoff", stamp)

    def switch(self, stamp=None) -> str:
        """Record a compatibility event; live scoring still begins at start()."""
        return self._events.emit("switch", stamp)

    def stop(self, stamp=None) -> str:
        return self._events.emit("flight_end", stamp)


def assistance_pose(value, message_type):
    """Exact raw reference stamp and hall frame, yaw-only orientation; no stale replay."""
    import math

    pose = (value or {}).get("pose")
    if pose is None:
        return None
    message = message_type()
    stamp = int(pose["stamp_ns"])
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.header.frame_id = value["frame"]
    message.pose.position.x = pose["x"]
    message.pose.position.y = pose["y"]
    message.pose.position.z = pose["z"]
    message.pose.orientation.z = math.sin(pose["yaw"] / 2)
    message.pose.orientation.w = math.cos(pose["yaw"] / 2)
    return message


class AssistancePublisher:  # pragma: no cover - requires ROS2 runtime
    """Explicit opt-in, volatile PoseStamped topic. Consumers must check stamp age.

    Use after creating Drone(new_session=False) with the same pairing as the
    telemetry forwarder. close() stops polling; no flight-control command is sent.
    """

    def __init__(self, node, drone, topic="/drone_challenge/assist"):
        from geometry_msgs.msg import PoseStamped
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, QoSProfile

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                         lifespan=Duration(seconds=0.5))
        self._node = node
        self._publisher = node.create_publisher(PoseStamped, topic, qos)

        def publish(value):
            message = assistance_pose(value, PoseStamped)
            if message is not None:
                self._publisher.publish(message)

        self._subscription = drone.watch_assist(publish)

    def close(self):
        self._subscription.close()
        self._node.destroy_publisher(self._publisher)
