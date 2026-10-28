"""Arduino bridge: one node owns the serial port in both directions.

Arduino -> Pi : "D:<cm>\n"   ultrasonic distance in centimeters (-1 = no echo)
Pi -> Arduino : "W:<0|1|2>\n" warning level drives the LED and buzzer

A serial port can only be opened by one process, so reading the sensor and
writing the alert live in the same node instead of two separate nodes.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from adas_interfaces.msg import CollisionWarning

import serial


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'ultrasonic_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open {port}: {e}')
            raise

        self.range_pub = self.create_publisher(Range, '/ultrasonic/range', qos_profile_sensor_data)
        self.create_subscription(CollisionWarning, '/adas/warning', self.on_warning, 10)

        self.last_level = None
        self.lock = threading.Lock()
        self.running = True
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(f'Arduino bridge on {port} @ {baud}')

    def read_loop(self):
        while self.running and rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read failed: {e}')
                continue
            if not line.startswith('D:'):
                continue
            try:
                cm = float(line[2:])
            except ValueError:
                continue
            self.publish_range(cm)

    def publish_range(self, cm):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(15.0)  # HC-SR04 beam is roughly 15 degrees
        msg.min_range = 0.02
        msg.max_range = 4.0
        # No echo (or out of range) -> +inf per REP-117 ("nothing detected")
        msg.range = float('inf') if cm < 0 else cm / 100.0
        self.range_pub.publish(msg)

    def on_warning(self, msg):
        # Only send on change to keep the serial link quiet
        if msg.level == self.last_level:
            return
        with self.lock:
            try:
                self.ser.write(f'W:{msg.level}\n'.encode('ascii'))
                self.last_level = msg.level
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')

    def destroy_node(self):
        self.running = False
        try:
            self.ser.write(b'W:0\n')  # leave the buzzer off on shutdown
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
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
