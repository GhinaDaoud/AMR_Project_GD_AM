"""
traffic_wanderer.py — Random wanderer for traffic robots (Phase 2)
Package : service_robot
Node    : traffic_wanderer
Run via : ros2 launch robot_description slam.launch.py  (two instances auto-started)

What it does
------------
Drives a traffic robot around the environment using the same reactive
state machine as explorer.py but with different speeds and randomisation
so each traffic robot behaves differently.

One node instance per robot, distinguished by the robot_name parameter:
  robot_name=traffic_robot_1  →  /traffic_robot_1/scan + /traffic_robot_1/cmd_vel
  robot_name=traffic_robot_2  →  /traffic_robot_2/scan + /traffic_robot_2/cmd_vel

How the main robot avoids them
--------------------------------
The main robot's LIDAR (/scan) detects the traffic robots as solid obstacles.
Nav2's local costmap inflates them automatically. No extra code needed.

Parameters
----------
  robot_name   (string, required)  e.g. "traffic_robot_1"
  linear_speed (float,  default 0.25 m/s)
  turn_speed   (float,  default 0.55 rad/s)

Debug tips
----------
  Watch a traffic robot  : ros2 topic echo /traffic_robot_1/scan --no-arr
  Stop a traffic robot   : ros2 topic pub /traffic_robot_1/cmd_vel geometry_msgs/msg/Twist '{}'
  Check both odometries  : ros2 topic echo /traffic_robot_1/odom
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

FRONT_CLEAR  = 0.65   # m — drive forward if front farther than this
FRONT_DANGER = 0.28   # m — back up if closer than this
FRONT_HALF   = 35.0   # deg — half-arc checked for "ahead"
SIDE_HALF    = 30.0   # deg — half-arc for open-side decision


class TrafficWanderer(Node):

    def __init__(self):
        super().__init__('traffic_wanderer')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('turn_speed',   0.55)

        robot_name   = self.get_parameter('robot_name').value
        self._linear = self.get_parameter('linear_speed').value
        self._turn   = self.get_parameter('turn_speed').value

        if not robot_name:
            self.get_logger().error('robot_name parameter is required — shutting down')
            raise RuntimeError('robot_name not set')

        scan_topic   = f'/{robot_name}/scan'
        cmdvel_topic = f'/{robot_name}/cmd_vel'

        self._pub  = self.create_publisher(Twist, cmdvel_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)

        self._state     = 'EXPLORE'
        self._turn_dir  = random.choice([-1, 1])
        self._last_scan = None

        self.create_timer(0.1, self._step)

        self.get_logger().info(
            f'TrafficWanderer [{robot_name}]  '
            f'scan={scan_topic}  cmd={cmdvel_topic}  '
            f'v={self._linear}m/s  w={self._turn}rad/s'
        )

    # ── helpers ──────────────────────────────────────────────────────── #

    def _min_range(self, scan: LaserScan, centre_deg: float, half: float) -> float:
        lo = centre_deg - half
        hi = centre_deg + half
        angle_min_deg = math.degrees(scan.angle_min)
        inc_deg       = math.degrees(scan.angle_increment)
        best = float('inf')
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            ang = angle_min_deg + i * inc_deg
            if lo <= ang <= hi and scan.range_min < r < scan.range_max:
                best = min(best, r)
        return best

    # ── callbacks ────────────────────────────────────────────────────── #

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _step(self) -> None:
        if self._last_scan is None:
            return

        scan  = self._last_scan
        front = self._min_range(scan,   0.0, FRONT_HALF)
        left  = self._min_range(scan,  90.0, SIDE_HALF)
        right = self._min_range(scan, -90.0, SIDE_HALF)

        cmd = Twist()

        if self._state == 'EXPLORE':
            if front < FRONT_DANGER:
                self._state = 'BACKUP'
            elif front < FRONT_CLEAR:
                # add small random noise so the two robots don't sync up
                bias = random.uniform(-0.15, 0.15)
                self._turn_dir = 1 if (left + bias) >= right else -1
                self._state = 'TURN'
            else:
                cmd.linear.x = self._linear

        elif self._state == 'TURN':
            if front > FRONT_CLEAR:
                self._state = 'EXPLORE'
            else:
                cmd.angular.z = self._turn * self._turn_dir

        elif self._state == 'BACKUP':
            if front > FRONT_DANGER * 1.9:
                self._turn_dir = random.choice([-1, 1])
                self._state = 'TURN'
            else:
                cmd.linear.x = -self._linear * 0.5

        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficWanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub.publish(Twist())   # stop on exit
        node.destroy_node()
        rclpy.shutdown()
