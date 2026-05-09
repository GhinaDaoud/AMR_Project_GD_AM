"""
explorer.py — Autonomous maze explorer (Part 1)
Package : service_robot
Node    : explorer
Run via : ros2 launch robot_description slam.launch.py mode:=auto  (default)
          ros2 run service_robot explorer                           (standalone)

What it does
------------
Drives the robot around the maze using a 3-state reactive state machine.
No map needed — decisions are based purely on the live /scan laser ranges.

State machine
-------------
  EXPLORE  drive forward at LINEAR_SPEED
           → BACKUP  if front < FRONT_DANGER (wall very close)
           → TURN    if front < FRONT_CLEAR  (wall approaching)
  TURN     rotate toward the more open side (left or right arc)
           → EXPLORE once front is clear again
  BACKUP   short reverse to create clearance
           → TURN    once front > FRONT_DANGER * 1.8

Topics
------
  Subscribes : /scan  (sensor_msgs/LaserScan)  — laser range data from LIDAR
  Publishes  : /cmd_vel (geometry_msgs/Twist)  — velocity commands to the robot

Tuning knobs (top of file)
--------------------------
  LINEAR_SPEED, TURN_SPEED  — how fast the robot moves / rotates
  FRONT_CLEAR, FRONT_DANGER — distance thresholds that trigger state changes
  FRONT_HALF, SIDE_HALF     — angular arcs (degrees) used when sampling /scan

Debug tips
----------
  Watch state changes : ros2 topic echo /rosout | grep explorer
  Check velocity      : ros2 topic echo /cmd_vel
  Check laser         : ros2 topic echo /scan --no-arr
"""

import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# ── tuneable parameters ──────────────────────────────────────────── #
LINEAR_SPEED  = 0.40   # m/s forward
TURN_SPEED    = 0.70   # rad/s rotation
FRONT_CLEAR   = 0.70   # m  — free to drive if front farther than this
FRONT_DANGER  = 0.30   # m  — back up immediately if closer than this
FRONT_HALF    = 30.0   # deg half-arc to check "ahead"
SIDE_HALF     = 25.0   # deg half-arc for left/right open-side decision
# ─────────────────────────────────────────────────────────────────── #


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        self._state = 'EXPLORE'
        self._turn_dir = 1        # +1 = left, -1 = right
        self._last_scan = None

        self.create_timer(0.1, self._step)   # 10 Hz control loop
        self.get_logger().info('Explorer ready — waiting for first scan…')

    # ---------------------------------------------------------------- #

    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _min_range(self, scan: LaserScan, center_deg: float, half: float) -> float:
        """Minimum valid range inside [center-half, center+half] degrees."""
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

    # ---------------------------------------------------------------- #

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
        node.pub_cmd.publish(Twist())   # stop robot on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
