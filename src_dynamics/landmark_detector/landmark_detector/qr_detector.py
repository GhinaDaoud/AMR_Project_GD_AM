"""
qr_detector.py — QR-code landmark detector (Part 1)
Package : landmark_detector
Node    : qr_detector
Run via : ros2 launch robot_description slam.launch.py  (always started)
          ros2 run landmark_detector qr_detector         (standalone)

What it does
------------
Reads the robot's front camera feed, finds QR codes, resolves their position
in the map frame via TF2, and appends each unique landmark to a JSON database.
The database is the handoff from Part 1 → Part 2: the mission_planner reads
it to know where named locations are on the map.

Topics
------
  Subscribes : /camera/image_raw  (sensor_msgs/Image)      — raw camera frames
               /odom              (nav_msgs/Odometry)       — fallback pose
  Publishes  : /landmark_detected (std_msgs/String)         — QR text on detect
  TF lookup  : map ← base_footprint                        — preferred pose source

Output file
-----------
  src/my_local/maps/landmark_db.json
  Schema: { "_meta": {...}, "landmarks": { "<QR text>": { x, y, frame_id,
            first_seen, last_updated, detection_count } } }

Cooldown
--------
  Same landmark is not re-saved within COOLDOWN_SEC (default 5 s) to avoid
  flooding the DB while the robot lingers in front of a QR code.

Debug tips
----------
  Watch detections  : ros2 topic echo /landmark_detected
  Check camera feed : the OpenCV window "QR Detector" shows live bounding boxes
                      green = fully decoded, orange = partial detection
  Check DB contents : cat src/my_local/maps/landmark_db.json
  TF not available? : ros2 run tf2_tools view_frames  — check map→base_footprint
"""

import os
import json
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

COOLDOWN_SEC = 5.0   # min seconds before re-saving the same landmark


def _maps_dir() -> str:
    """Resolve <workspace>/src/my_local/maps from the installed share path."""
    share = get_package_share_directory('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    path = os.path.join(ws_root, 'src', 'my_local', 'maps')
    os.makedirs(path, exist_ok=True)
    return path


MAPS_DIR = _maps_dir()
LANDMARK_DB_PATH = os.path.join(MAPS_DIR, 'landmark_db.json')


class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector')

        self.bridge = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()

        # TF2 — map-frame pose (preferred); falls back to odom when SLAM not ready
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        # Landmark DB loaded from disk
        self.db: dict = self._load_db()
        self._last_saved: dict[str, float] = {}

        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)
        self.pub_detection = self.create_publisher(String, '/landmark_detected', 10)

        self.get_logger().info(
            f'QR Detector ready\n'
            f'  DB path : {LANDMARK_DB_PATH}\n'
            f'  Maps dir: {MAPS_DIR}\n'
            f'  Loaded  : {len(self.db.get("landmarks", {}))} landmarks'
        )

    # ── persistence ──────────────────────────────────────────────────────── #

    def _load_db(self) -> dict:
        if os.path.exists(LANDMARK_DB_PATH):
            try:
                with open(LANDMARK_DB_PATH, 'r') as f:
                    db = json.load(f)
                n = len(db.get('landmarks', {}))
                self.get_logger().info(f'Loaded {n} existing landmarks from DB')
                return db
            except Exception as e:
                self.get_logger().warn(f'Could not load DB, starting fresh: {e}')
        # fresh DB with metadata header
        return {
            '_meta': {
                'version': '1.0',
                'frame_id': 'map',
                'description': 'Landmark positions detected during autonomous exploration. '
                               'x/y are in the ROS map frame (metres). '
                               'Load this file in mission_planner to navigate to each landmark.',
            },
            'landmarks': {}
        }

    def _save_db(self):
        try:
            with open(LANDMARK_DB_PATH, 'w') as f:
                json.dump(self.db, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to write landmark DB: {e}')

    # ── pose helpers ─────────────────────────────────────────────────────── #

    def _on_odom(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def _get_pose(self) -> tuple[float, float, str]:
        """Return (x, y, frame_id) — map frame preferred, odom fallback."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            t = tf.transform.translation
            return float(t.x), float(t.y), 'map'
        except (LookupException, ConnectivityException, ExtrapolationException):
            return self.odom_x, self.odom_y, 'odom'

    # ── image callback ───────────────────────────────────────────────────── #

    def _on_image(self, msg: Image):
        # Convert ROS image → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        # ── detection ────────────────────────────────────────────────────── #
        try:
            retval, decoded_info, points, _ = self.qr_decoder.detectAndDecodeMulti(frame)
        except Exception as e:
            self.get_logger().warn(f'QR detection error: {e}')
            cv2.imshow('QR Detector', frame)
            cv2.waitKey(1)
            return

        # ── draw bounding boxes for EVERY detected QR (decoded or not) ───── #
        if retval and points is not None:
            for i, pts in enumerate(points):
                if pts is None:
                    continue
                # reshape to (N, 1, 2) as cv2.polylines expects
                ipts = pts.reshape((-1, 1, 2)).astype(np.int32)
                text = decoded_info[i] if decoded_info and i < len(decoded_info) else ''
                color = (0, 255, 0) if text else (0, 165, 255)  # green=decoded, orange=partial
                cv2.polylines(frame, [ipts], True, color, 2)
                if text:
                    corner = tuple(ipts[0][0])
                    cv2.putText(frame, text,
                                (corner[0], corner[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # ── save newly decoded landmarks ──────────────────────────────────── #
        if retval and decoded_info:
            now_mono = time.monotonic()
            now_iso = datetime.now(timezone.utc).isoformat()

            for i, text in enumerate(decoded_info):
                if not text:
                    continue
                # cooldown: don't spam-save the same landmark
                if now_mono - self._last_saved.get(text, 0.0) < COOLDOWN_SEC:
                    continue

                try:
                    x, y, frame_id = self._get_pose()
                except Exception as e:
                    self.get_logger().warn(f'Pose error: {e}')
                    continue

                landmarks = self.db.setdefault('landmarks', {})
                existing = landmarks.get(text)
                is_new = existing is None
                count = 1 if is_new else existing.get('detection_count', 1) + 1

                landmarks[text] = {
                    'x': round(x, 3),
                    'y': round(y, 3),
                    'frame_id': frame_id,
                    'first_seen': (existing or {}).get('first_seen', now_iso),
                    'last_updated': now_iso,
                    'detection_count': count,
                }
                self._last_saved[text] = now_mono
                self._save_db()

                label = 'NEW' if is_new else f'UPDATED (×{count})'
                self.get_logger().info(
                    f'[{label}] "{text}" @ {frame_id} ({x:.2f}, {y:.2f})'
                )
                self.pub_detection.publish(String(data=text))

        # ── always show the camera feed ───────────────────────────────────── #
        try:
            cv2.imshow('QR Detector', frame)
            cv2.waitKey(1)
        except Exception:
            pass   # GUI unavailable in headless mode


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
