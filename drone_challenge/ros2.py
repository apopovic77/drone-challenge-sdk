"""ROS2 helper for the flight events, in the team's own node::

    from drone_challenge.ros2 import Challenge

    challenge = Challenge(self)      # in __init__ of your node
    challenge.takeoff()
    challenge.switch()               # external positioning (GNSS emulation) off
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
        return self._events.emit("switch", stamp)

    def stop(self, stamp=None) -> str:
        return self._events.emit("flight_end", stamp)
