"""
mission_node.py — Autonomous service mission executor (Part 2)
Package : mission_planner
Node    : mission_planner

Goal format  (publish to /mission_goal)
----------------------------------------
  1  →  Dock → Pillar B → Dock
  2  →  Dock → Pillar A → Dock
  3  →  Dock → Pillar B → Pillar A → Dock

  ros2 topic pub --once /mission_goal std_msgs/msg/String "data: '3'"

State machine
-------------
  IDLE → GOING_TO_TARGET → AT_TARGET → ... → RETURNING_HOME → DOCKED → IDLE
  (GOING_TO_TARGET / AT_TARGET repeat for each intermediate waypoint)

Docking station
---------------
  Captured from TF (map→base_footprint) when the node starts up.

Topics
------
  Subscribes : /mission_goal   (std_msgs/String)  — mission number
  Publishes  : /mission_status (std_msgs/String)  — state updates
"""

import json
import os

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


# ── state machine states ──────────────────────────────────────────────
IDLE            = 'IDLE'
GOING_TO_TARGET = 'GOING_TO_TARGET'
AT_TARGET       = 'AT_TARGET'
RETURNING_HOME  = 'RETURNING_HOME'
DOCKED          = 'DOCKED'


def _maps_dir() -> str:
    share = get_package_share_directory('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'src', 'my_local', 'maps')


class MissionPlannerNode(Node):

    def __init__(self):
        super().__init__('mission_planner')

        # TF listener — dock position captured at startup
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._home_x: float | None = None
        self._home_y: float | None = None

        # Load landmark DB
        db_path = os.path.join(_maps_dir(), 'landmark_db.json')
        with open(db_path, 'r') as f:
            db = json.load(f)
        self._landmark_list = list(db.get('landmarks', {}).items())

        self._nav        = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._status_pub = self.create_publisher(String, '/mission_status', 10)
        self.create_subscription(String, '/mission_goal', self._on_goal, 10)

        self._state     = IDLE
        self._waypoints: list[tuple[float, float, str]] = []
        self._cur_label = ''

        # Poll TF until dock position is captured
        self._dock_timer = self.create_timer(1.0, self._capture_dock)

    # ── dock position capture ─────────────────────────────────────────

    def _capture_dock(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except Exception:
            self.get_logger().info(
                'Waiting for map→base_footprint TF…',
                throttle_duration_sec=5.0,
            )
            return

        self._home_x = tf.transform.translation.x
        self._home_y = tf.transform.translation.y
        self._dock_timer.cancel()

        self.get_logger().info(
            f'Docking station captured: ({self._home_x:.3f}, {self._home_y:.3f})'
        )

        lm = self._landmark_list
        self.get_logger().info('Available missions:')
        self.get_logger().info(
            f'  1  →  Dock → "{lm[0][0]}" → Dock'
        )
        self.get_logger().info(
            f'  2  →  Dock → "{lm[1][0]}" → Dock'
        )
        self.get_logger().info(
            f'  3  →  Dock → "{lm[0][0]}" → "{lm[1][0]}" → Dock'
        )
        self.get_logger().info(
            'Send:  ros2 topic pub --once /mission_goal '
            'std_msgs/msg/String "data: \'3\'"'
        )

    # ── incoming mission goal ─────────────────────────────────────────

    def _on_goal(self, msg: String):
        raw = msg.data.strip()

        if self._home_x is None:
            self.get_logger().warn('Dock not yet set — try again in a moment')
            return

        if self._state != IDLE:
            self.get_logger().warn(f'Busy ({self._state}) — ignoring "{raw}"')
            return

        try:
            number = int(raw)
        except ValueError:
            self.get_logger().error(f'Expected 1, 2, or 3 — got "{raw}"')
            return

        home = (self._home_x, self._home_y, 'Docking Station')
        lm   = self._landmark_list

        if number == 1:
            name_b, d_b = lm[0]
            self._waypoints = [(d_b['x'], d_b['y'], name_b), home]
        elif number == 2:
            name_a, d_a = lm[1]
            self._waypoints = [(d_a['x'], d_a['y'], name_a), home]
        elif number == 3:
            name_b, d_b = lm[0]
            name_a, d_a = lm[1]
            self._waypoints = [
                (d_b['x'], d_b['y'], name_b),
                (d_a['x'], d_a['y'], name_a),
                home,
            ]
        else:
            self.get_logger().error(f'Unknown mission {number}. Valid: 1, 2, 3')
            return

        self.get_logger().info(
            f'Mission {number} started: '
            + ' → '.join(label for _, _, label in self._waypoints)
        )
        self._advance()

    # ── waypoint sequencer ────────────────────────────────────────────

    def _advance(self):
        if not self._waypoints:
            self._set_state(IDLE)
            return

        x, y, label = self._waypoints.pop(0)
        self._cur_label = label
        is_home = len(self._waypoints) == 0  # home is always the last entry

        self.get_logger().info(f'Navigating to "{label}" @ ({x:.3f}, {y:.3f})')
        self._set_state(RETURNING_HOME if is_home else GOING_TO_TARGET)
        self._send_nav_goal(x, y, self._on_waypoint_reached)

    def _on_waypoint_reached(self, future):
        status   = future.result().status
        is_home  = self._state == RETURNING_HOME
        success  = status == GoalStatus.STATUS_SUCCEEDED

        if not success:
            self.get_logger().warn(
                f'Failed to reach "{self._cur_label}" (status={success})'
            )
            self._publish_status('FAILED')

        if is_home:
            if success:
                self.get_logger().info('Docked. Ready for next mission.')
                self._set_state(DOCKED)
                self._publish_status('DOCKED')
            self._set_state(IDLE)
            return

        # Intermediate waypoint — pause 5 s then continue
        self.get_logger().info(
            f'Arrived at "{self._cur_label}". Waiting 5 s…'
        )
        self._set_state(AT_TARGET)
        self._publish_status('AT_TARGET')
        self._pause_timer = self.create_timer(5.0, self._pause_done)

    def _pause_done(self):
        self._pause_timer.cancel()
        self._advance()

    # ── navigation helpers ────────────────────────────────────────────

    def _send_nav_goal(self, x: float, y: float, done_cb):
        if not self._nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose server not available')
            self._set_state(IDLE)
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        future = self._nav.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        future.add_done_callback(lambda f: self._on_goal_accepted(f, done_cb))

    def _on_goal_accepted(self, future, done_cb):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self._set_state(IDLE)
            self._publish_status('FAILED')
            return
        handle.get_result_async().add_done_callback(done_cb)

    def _on_feedback(self, feedback_msg):
        self.get_logger().info(
            f'[{self._state}] distance remaining: '
            f'{feedback_msg.feedback.distance_remaining:.2f} m',
            throttle_duration_sec=5.0,
        )

    # ── helpers ───────────────────────────────────────────────────────

    def _set_state(self, state: str):
        self._state = state
        self.get_logger().info(f'State → {state}')

    def _publish_status(self, text: str):
        self._status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()