"""Camera node: publishes webcam frames as sensor_msgs/Image."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device', 0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('frame_id', 'camera_link')

        dev = self.get_parameter('device').value
        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always grab the newest frame
        if not self.cap.isOpened():
            self.get_logger().fatal(f'Cannot open camera /dev/video{dev}')
            raise RuntimeError('camera not available')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(f'Camera /dev/video{dev} at {w}x{h}, {fps} fps')

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Frame grab failed', throttle_duration_sec=2.0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
