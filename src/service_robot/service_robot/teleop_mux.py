"""
teleop_mux.py — Keyboard-override mux for autonomous exploration
Package : service_robot
Node    : teleop_mux

Default: forwards /cmd_vel_auto (explorer) → /cmd_vel
Override: the moment any key is pressed on the keyboard (/cmd_vel_teleop),
          the mux switches to TELEOP and blocks explorer commands.
Resume:   after OVERRIDE_TIMEOUT seconds of keyboard silence, mux returns
          to AUTO and the explorer continues from wherever the robot is.

Topics
------
  Subscribes : /cmd_vel_auto   (Twist) — from explorer
               /cmd_vel_teleop (Twist) — from teleop_twist_keyboard
  Publishes  : /cmd_vel        (Twist) — to differential-drive plugin
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

OVERRIDE_TIMEOUT = 0.5   # seconds of keyboard silence before returning to AUTO


class TeleopMux(Node):
    def __init__(self):
        super().__init__('teleop_mux')

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._last_teleop_t = 0.0
        self._mode = 'auto'

        self.create_subscription(Twist, '/cmd_vel_auto',   self._on_auto,   10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self._on_teleop, 10)

        self.get_logger().info(
            'Teleop mux ready — AUTO mode active. '
            'Press any movement key to take over; release to return to AUTO.'
        )

    def _on_teleop(self, msg: Twist):
        self._last_teleop_t = time.monotonic()
        if self._mode != 'teleop':
            self._mode = 'teleop'
            self.get_logger().info('TELEOP — keyboard is driving')
        self._pub.publish(msg)

    def _on_auto(self, msg: Twist):
        if self._mode == 'teleop':
            if time.monotonic() - self._last_teleop_t < OVERRIDE_TIMEOUT:
                return   # keyboard still "active" — suppress explorer
            self._mode = 'auto'
            self.get_logger().info('AUTO — explorer resumed')
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub.publish(Twist())   # stop the robot on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()