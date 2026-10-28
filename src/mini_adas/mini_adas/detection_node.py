"""Person detection with MobileNet-SSD (OpenCV DNN), with a HOG fallback.

Publishes:
  /perception/pedestrian_in_path   std_msgs/Bool   true if a person is in the forward zone
  /perception/annotated/compressed sensor_msgs/CompressedImage  boxes + zone, for Foxglove

Only detections whose box center falls in the middle "forward zone" of the
image count, the same idea as an ADAS camera ignoring objects in other lanes.

Why MobileNet-SSD instead of HOG: HOG only recognizes full standing bodies,
so it misses people who are close to the camera or only partly in frame.
MobileNet-SSD is a small CNN trained on 20 object classes (including person)
that runs at several frames per second on a Raspberry Pi 4 CPU.
"""
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge

PERSON_CLASS_ID = 15  # index of "person" in MobileNet-SSD's VOC class list


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.declare_parameter('model_dir', os.path.expanduser('~/mini-adas-ros2/models'))
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('zone_width_frac', 0.5)   # central 50% of the image = forward zone
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('hold_s', 0.7)            # keep a detection alive briefly to stop flicker

        p = lambda n: self.get_parameter(n).value
        self.min_period = 1.0 / p('max_rate_hz')
        self.zone_frac = p('zone_width_frac')
        self.min_conf = p('min_confidence')
        self.hold_s = p('hold_s')

        model_dir = p('model_dir')
        proto = os.path.join(model_dir, 'MobileNetSSD_deploy.prototxt')
        weights = os.path.join(model_dir, 'MobileNetSSD_deploy.caffemodel')
        if os.path.exists(proto) and os.path.exists(weights):
            self.net = cv2.dnn.readNetFromCaffe(proto, weights)
            self.backend = 'MobileNet-SSD'
        else:
            self.net = None
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.backend = 'HOG (model files not found in %s)' % model_dir

        self.bridge = CvBridge()
        self.last_run = 0.0
        self.last_seen = -1e9

        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.flag_pub = self.create_publisher(Bool, '/perception/pedestrian_in_path', 10)
        self.img_pub = self.create_publisher(
            CompressedImage, '/perception/annotated/compressed', qos_profile_sensor_data)
        self.get_logger().info(f'Person detector ready: {self.backend}')

    def detect(self, frame):
        """Return a list of (x, y, w, h, confidence) person boxes in pixel coordinates."""
        h, w = frame.shape[:2]
        if self.net is not None:
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
            self.net.setInput(blob)
            out = self.net.forward()  # shape (1, 1, N, 7): [_, class, conf, x1, y1, x2, y2]
            boxes = []
            for det in out[0, 0]:
                cls, conf = int(det[1]), float(det[2])
                if cls != PERSON_CLASS_ID or conf < self.min_conf:
                    continue
                x1, y1 = int(det[3] * w), int(det[4] * h)
                x2, y2 = int(det[5] * w), int(det[6] * h)
                boxes.append((x1, y1, x2 - x1, y2 - y1, conf))
            return boxes
        rects, weights = self.hog.detectMultiScale(frame, winStride=(8, 8), padding=(8, 8), scale=1.05)
        return [(int(x), int(y), int(bw), int(bh), float(c))
                for (x, y, bw, bh), c in zip(rects, weights) if float(c) >= self.min_conf]

    def on_image(self, msg):
        now = time.monotonic()
        if now - self.last_run < self.min_period:
            return
        self.last_run = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        x0 = int(w * (1 - self.zone_frac) / 2)
        x1 = w - x0

        hit_now = False
        for (x, y, bw, bh, conf) in self.detect(frame):
            cx = x + bw / 2
            hit = bool(x0 <= cx <= x1)
            hit_now = hit_now or hit
            color = (0, 0, 255) if hit else (0, 165, 255)  # red in path, orange off to the side
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(frame, f'person {conf:.2f}', (x, max(y - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        if hit_now:
            self.last_seen = now
        in_path = (now - self.last_seen) < self.hold_s

        # Forward zone lines
        cv2.line(frame, (x0, 0), (x0, h), (0, 255, 0), 1)
        cv2.line(frame, (x1, 0), (x1, h), (0, 255, 0), 1)
        if in_path:
            cv2.putText(frame, 'PERSON IN PATH', (x0 + 4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        # ROS2 message fields need a plain Python bool, not numpy.bool_
        self.flag_pub.publish(Bool(data=bool(in_path)))

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
