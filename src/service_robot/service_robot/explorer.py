"""
explorer.py — Autonomous explorer for open environments (Part 1)
Package : service_robot
Node    : explorer

What it does
------------
Drives the robot autonomously using a 3-state reactive state machine.
Decisions are based purely on the live /scan laser ranges — no map needed.

State machine
-------------
  EXPLORE  drive forward at LINEAR_SPEED
           → BACKUP  if front < FRONT_DANGER (wall very close)
           → TURN    if front < FRONT_CLEAR  (wall approaching)
  TURN     rotate toward the more open side
           → EXPLORE once front is clear again
  BACKUP   short reverse to create clearance
           → TURN    once front is clear enough

Topics
------
  Subscribes : /scan    (sensor_msgs/LaserScan) — laser range data
  Publishes  : /cmd_vel (geometry_msgs/Twist)   — velocity commands
"""

import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# ── tuneable parameters ──────────────────────────────────────────── #
LINEAR_SPEED  = 0.28   # m/s forward — slow enough for safe stops
TURN_SPEED    = 0.55   # rad/s rotation
FRONT_CLEAR   = 1.20   # m  — start turning while still far from obstacles
FRONT_DANGER  = 0.50   # m  — back up well before hitting anything
FRONT_HALF    = 30.0   # deg half-arc to check "ahead"
SIDE_HALF     = 25.0   # deg half-arc for left/right open-side decision
# ─────────────────────────────────────────────────────────────────── #


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        self._state = 'EXPLORE'
        self._turn_dir = 1
        self._last_scan = None

        self.create_timer(0.1, self._step)
        self.get_logger().info('Explorer ready — waiting for first scan…')

    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _min_range(self, scan: LaserScan, center_deg: float, half: float) -> float:
        lo, hi = center_deg - half, center_deg + half
        angle_min_deg = math.degrees(scan.angle_min)
        inc_deg = math.degrees(scan.angle_increment)
        best = float('inf')
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            angle = angle_min_deg + i * inc_deg
            if lo <= angle <= hi and scan.range_min < r < scan.range_max:
                best = min(best, r)
        return best

    def _step(self):
        if self._last_scan is None:
            return

        scan = self._last_scan
        front = self._min_range(scan,  0.0, FRONT_HALF)
        left  = self._min_range(scan,  90.0, SIDE_HALF)
        right = self._min_range(scan, -90.0, SIDE_HALF)

        cmd = Twist()

        if self._state == 'EXPLORE':
            if front < FRONT_DANGER:
                self._state = 'BACKUP'
                self.get_logger().info(f'→ BACKUP  (front={front:.2f} m)')
            elif front < FRONT_CLEAR:
                self._turn_dir = 1 if (left + random.uniform(0, 0.1)) >= right else -1
                self._state = 'TURN'
                self.get_logger().info(
                    f'→ TURN {"left" if self._turn_dir > 0 else "right"} '
                    f'(f={front:.2f} l={left:.2f} r={right:.2f})'
                )
            else:
                cmd.linear.x = LINEAR_SPEED

        elif self._state == 'TURN':
            if front > FRONT_CLEAR:
                self._state = 'EXPLORE'
                self.get_logger().info('→ EXPLORE')
            else:
                cmd.angular.z = TURN_SPEED * self._turn_dir

        elif self._state == 'BACKUP':
            if front > FRONT_DANGER * 1.8:
                self._turn_dir = random.choice([-1, 1])
                self._state = 'TURN'
            else:
                cmd.linear.x = -LINEAR_SPEED * 0.5

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()