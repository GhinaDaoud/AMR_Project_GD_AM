import os
import json

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    _PYZBAR = True
except ImportError:
    _PYZBAR = False


def _maps_dir() -> str:
    share = get_package_share_directory('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'src', 'my_local', 'maps')


class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector')

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.bridge = CvBridge()
        self.current_x = 0.0
        self.current_y = 0.0
        self.detected_landmarks = {}

        self._cv2_detector = cv2.QRCodeDetector()

        self.db_path = os.path.join(_maps_dir(), 'landmark_db.json')

        if os.path.exists(self.db_path):
            with open(self.db_path, 'r') as f:
                self.detected_landmarks = json.load(f)
            self.get_logger().info(
                f'Loaded {len(self.detected_landmarks)} existing landmarks '
                f'from {self.db_path}')

        self.get_logger().info('QR Detector Node started')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def _decode_qr(self, cv_image):
        """Return list of (landmark_name, polygon_pts) decoded from the frame.

        Both decoders run always and results are merged — cv2 handles some QR
        images that pyzbar cannot (e.g. fire_station, restaurant, supermarket)
        and pyzbar handles others that cv2 cannot (e.g. house_4).
        """
        seen = set()
        results = []

        # cv2 QRCodeDetector — handles most of the town sign textures
        # detectAndDecodeMulti returns (retval, decoded_info, points, straight)
        retval, decoded_info, points, _ = self._cv2_detector.detectAndDecodeMulti(cv_image)
        if retval and decoded_info:
            for i, name in enumerate(decoded_info):
                if name and name not in seen:
                    seen.add(name)
                    pts = (points[i].astype(int).reshape(-1, 2).tolist()
                           if points is not None else [])
                    results.append((name, pts))

        # cv2 single-QR fallback — catches images detectAndDecodeMulti misses
        if not results:
            name, bbox, _ = self._cv2_detector.detectAndDecode(cv_image)
            if name and name not in seen:
                seen.add(name)
                pts = (bbox.astype(int).reshape(-1, 2).tolist()
                       if bbox is not None else [])
                results.append((name, pts))

        # pyzbar — ONLY when cv2 found nothing (handles house_4 which cv2 misses).
        # Must not run after a cv2 success: pyzbar hangs on some rendered QR
        # patterns (e.g. fire_station) and would freeze Gazebo via callback block.
        if not results and _PYZBAR:
            for qr in pyzbar_decode(cv_image):
                name = qr.data.decode('utf-8')
                if name and name not in seen:
                    seen.add(name)
                    pts = [(p.x, p.y) for p in qr.polygon]
                    results.append((name, pts))

        return results

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            detections = self._decode_qr(cv_image)

            for landmark_name, pts in detections:
                # Draw polygon
                if pts:
                    for i in range(len(pts)):
                        cv2.line(cv_image, tuple(pts[i]),
                                 tuple(pts[(i + 1) % len(pts)]),
                                 (0, 255, 0), 2)
                    cv2.putText(cv_image, landmark_name,
                                (pts[0][0], pts[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if landmark_name not in self.detected_landmarks:
                    position = {
                        'x': round(self.current_x, 3),
                        'y': round(self.current_y, 3),
                        'z': 0.0,
                    }
                    self.detected_landmarks[landmark_name] = position
                    self.get_logger().info(
                        f'NEW LANDMARK: "{landmark_name}" '
                        f'at x={position["x"]}, y={position["y"]}')
                    self._save()
                else:
                    self.get_logger().debug(f'Already known: {landmark_name}')

            cv2.imshow('QR Detector', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')

    def _save(self):
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        with open(self.db_path, 'w') as f:
            json.dump(self.detected_landmarks, f, indent=2)
        self.get_logger().info(
            f'Saved {len(self.detected_landmarks)} landmarks → {self.db_path}')


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()