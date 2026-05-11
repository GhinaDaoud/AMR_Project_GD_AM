"""
mission_executor.py — Nav2 action client for the mission planner.

Subscribes to /mission_goal (String: comma-separated 1-based landmark IDs),
navigates to each landmark via NavigateToPose, then returns to the docking
station (0, 0).  Publishes progress on /mission_status:

  GOING:<step>:<total>:<label>
  AT:<step>:<total>:<label>
  FAILED:<step>:<total>:<label>
  DOCKED
"""

import json
import math
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


def _maps_dir() -> str:
    share = get_package_share_directory('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'src', 'my_local', 'maps')


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_planner')

        cb = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=cb)
        self._status_pub = self.create_publisher(String, '/mission_status', 10)
        self.create_subscription(
            String, '/mission_goal', self._on_goal, 10, callback_group=cb)

        db_path = os.path.join(_maps_dir(), 'landmark_db.json')
        with open(db_path, 'r') as f:
            self._landmarks = list(json.load(f).items())

        self._running = False
        self.get_logger().info(
            f'Mission executor ready — {len(self._landmarks)} landmarks loaded')

    # ── public helpers ─────────────────────────────────────────────────

    def _pub_status(self, status: str):
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f'[STATUS] {status}')

    # ── goal subscription ──────────────────────────────────────────────

    def _on_goal(self, msg: String):
        if self._running:
            self.get_logger().warn('Mission already in progress, ignoring')
            return

        try:
            indices = [int(x.strip()) for x in msg.data.split(',') if x.strip()]
        except ValueError:
            self.get_logger().error(f'Bad goal format: {msg.data!r}')
            return

        waypoints = []
        for idx in indices:
            if 1 <= idx <= len(self._landmarks):
                name, data = self._landmarks[idx - 1]
                waypoints.append((name, float(data['x']), float(data['y'])))
            else:
                self.get_logger().warn(f'Landmark index {idx} out of range')

        if not waypoints:
            return

        self._running = True
        threading.Thread(target=self._execute, args=(waypoints,),
                         daemon=True).start()

    # ── navigation helper ──────────────────────────────────────────────

    def _navigate_to(self, x: float, y: float, label: str = '') -> bool:
        """Send a NavigateToPose goal; block until done. Returns True on success."""
        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('NavigateToPose server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _yaw_to_quat(0.0)

        done_event = threading.Event()
        result_holder = [None]

        def _on_accepted(future):
            gh = future.result()
            if not gh.accepted:
                result_holder[0] = GoalStatus.STATUS_ABORTED
                done_event.set()
                return
            gh.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            result_holder[0] = future.result().status
            done_event.set()

        self._nav_client.send_goal_async(goal).add_done_callback(_on_accepted)

        if not done_event.wait(timeout=180.0):
            self.get_logger().warn(f'Navigation to "{label}" timed out')
            return False

        return result_holder[0] == GoalStatus.STATUS_SUCCEEDED

    # ── mission execution (runs in its own thread) ─────────────────────

    def _execute(self, waypoints: list):
        total = len(waypoints) + 1   # landmark stops + return home

        for step, (name, x, y) in enumerate(waypoints, start=1):
            self._pub_status(f'GOING:{step}:{total}:{name}')

            if self._navigate_to(x, y, name):
                self._pub_status(f'AT:{step}:{total}:{name}')
                time.sleep(2.0)
            else:
                self._pub_status(f'FAILED:{step}:{total}:{name}')

        # Return to docking station
        self._pub_status(f'GOING:{total}:{total}:dock')
        if self._navigate_to(0.0, 0.0, 'dock'):
            self._pub_status('DOCKED')
        else:
            self._pub_status(f'FAILED:{total}:{total}:dock')

        self._running = False


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()