import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from pyzbar.pyzbar import decode
import json
import os

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.bridge = CvBridge()
        self.current_x = 0.0
        self.current_y = 0.0
        self.detected_landmarks = {}

        self.db_path = os.path.expanduser('~/ros2_ws/landmark_db.json')

        if os.path.exists(self.db_path):
            with open(self.db_path, 'r') as f:
                self.detected_landmarks = json.load(f)
            self.get_logger().info(
                f'Loaded {len(self.detected_landmarks)} existing landmarks')

        self.get_logger().info('QR Detector Node started')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            qr_codes = decode(cv_image)

            for qr in qr_codes:
                landmark_name = qr.data.decode('utf-8')

                # Draw box around QR
                points = qr.polygon
                if points:
                    pts = [(p.x, p.y) for p in points]
                    for i in range(len(pts)):
                        cv2.line(cv_image, pts[i],
                                pts[(i+1) % len(pts)],
                                (0, 255, 0), 2)

                cv2.putText(cv_image, landmark_name,
                           (qr.rect.left, qr.rect.top - 10),
                           cv2.FONT_HERSHEY_SIMPLEX,
                           0.5, (0, 255, 0), 2)

                if landmark_name not in self.detected_landmarks:
                    position = {
                        'x': round(self.current_x, 3),
                        'y': round(self.current_y, 3),
                        'z': 0.0
                    }
                    self.detected_landmarks[landmark_name] = position
                    self.get_logger().info(
                        f'NEW LANDMARK DETECTED: {landmark_name} '
                        f'at x={position["x"]}, y={position["y"]}')
                    self.save_landmarks()
                else:
                    self.get_logger().info(
                        f'Already know: {landmark_name}')

            cv2.imshow('QR Detector', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def save_landmarks(self):
        with open(self.db_path, 'w') as f:
            json.dump(self.detected_landmarks, f, indent=2)
        self.get_logger().info(
            f'Saved {len(self.detected_landmarks)} landmarks')

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