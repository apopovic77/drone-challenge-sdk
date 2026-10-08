"""Drone Challenge mit ROS2: Positionen brauchen keinen Code, nur die Ereignisse.

Positionen veröffentlicht euer Node ohnehin (PoseStamped/Odometry …). Daneben läuft:
    drone-challenge session new          # Session für den aktiven Challenge-Kurs
    drone-challenge ros2 /drohne/pose

Dieser Node zeigt, wie Start und Ende gemeldet werden. Die Wertung beginnt am Start.
"""

import rclpy
from rclpy.node import Node

from drone_challenge.ros2 import Challenge


class Mission(Node):
    def __init__(self) -> None:
        super().__init__("mission")
        self.challenge = Challenge(self)
        self.challenge.start()  # system_start: ohne Start-Ereignis keine Wertung
        self.challenge.takeoff()  # optional
        self.create_timer(30.0, self.ende)
        self._beendet = False

    def ende(self) -> None:
        if not self._beendet:
            self.challenge.stop()
            self._beendet = True


def main() -> None:
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
