"""Pedestrian detection with OpenCV's HOG + linear SVM people detector.

Publishes:
  /perception/pedestrian_in_path  std_msgs/Bool   true if a pedestrian is in the forward zone
  /perception/annotated/compressed sensor_msgs/CompressedImage  boxes + zone, for Foxglove

Only detections whose box center falls in the middle "forward zone" of the
image count, the same idea as an ADAS camera ignoring objects in other lanes.
"""
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.declare_parameter('max_rate_hz', 5.0)       # HOG is heavy on a Pi 4; throttle it
        self.declare_parameter('zone_width_frac', 0.5)   # central 50% of the image = forward zone
        self.declare_parameter('min_confidence', 0.5)

        self.min_period = 1.0 / self.get_parameter('max_rate_hz').value
        self.zone_frac = self.get_parameter('zone_width_frac').value
        self.min_conf = self.get_parameter('min_confidence').value

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.bridge = CvBridge()
        self.last_run = 0.0

        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.flag_pub = self.create_publisher(Bool, '/perception/pedestrian_in_path', 10)
        self.img_pub = self.create_publisher(
            CompressedImage, '/perception/annotated/compressed', qos_profile_sensor_data)
        self.get_logger().info('HOG pedestrian detector ready')

    def on_image(self, msg):
        now = time.monotonic()
        if now - self.last_run < self.min_period:
            return
        self.last_run = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        x0 = int(w * (1 - self.zone_frac) / 2)
        x1 = w - x0

        rects, weights = self.hog.detectMultiScale(
            frame, winStride=(8, 8), padding=(8, 8), scale=1.05)

        in_path = False
        for (x, y, bw, bh), conf in zip(rects, weights):
            conf = float(conf)
            if conf < self.min_conf:
                continue
            cx = x + bw / 2
            hit = x0 <= cx <= x1
            in_path |= hit
            color = (0, 0, 255) if hit else (0, 200, 255)
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(frame, f'{conf:.2f}', (x, max(y - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Draw the forward zone
        cv2.line(frame, (x0, 0), (x0, h), (0, 255, 0), 1)
        cv2.line(frame, (x1, 0), (x1, h), (0, 255, 0), 1)

        self.flag_pub.publish(Bool(data=in_path))

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
        self.img_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
