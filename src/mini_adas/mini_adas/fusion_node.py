"""Fusion node: combines ultrasonic distance and camera pedestrian detection.

Decision logic (evaluated at a fixed rate):
  - Ultrasonic stale            -> CAUTION  (fail-safe: never report SAFE without range data)
  - distance <= brake_m         -> BRAKE    (anything this close is a hazard)
  - distance <= caution_m AND pedestrian in path -> BRAKE  (vulnerable road user, act earlier)
  - distance <= caution_m       -> CAUTION
  - pedestrian in path, far     -> CAUTION
  - otherwise                   -> SAFE

The camera answers "what is it", the ultrasonic answers "how far". Each covers
the other's blind spot: the camera can't measure range, and the ultrasonic
can't tell a person from a wall.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from adas_interfaces.msg import CollisionWarning

NAMES = {0: 'SAFE', 1: 'CAUTION', 2: 'BRAKE'}


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('brake_m', 0.30)
        self.declare_parameter('caution_m', 0.80)
        self.declare_parameter('stale_s', 0.5)
        self.declare_parameter('camera_stale_s', 1.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('release_cycles', 5)  # hysteresis: cycles before downgrading

        p = lambda n: self.get_parameter(n).value
        self.brake_m, self.caution_m = p('brake_m'), p('caution_m')
        self.stale_s, self.cam_stale_s = p('stale_s'), p('camera_stale_s')
        self.release_cycles = p('release_cycles')

        self.distance = math.nan
        self.range_time = None
        self.pedestrian = False
        self.cam_time = None
        self.level = CollisionWarning.SAFE
        self.lower_count = 0

        self.create_subscription(Range, '/ultrasonic/range', self.on_range, qos_profile_sensor_data)
        self.create_subscription(Bool, '/perception/pedestrian_in_path', self.on_ped, 10)
        self.pub = self.create_publisher(CollisionWarning, '/adas/warning', 10)
        self.create_timer(1.0 / p('rate_hz'), self.tick)

    def on_range(self, msg):
        self.distance = msg.range
        self.range_time = self.get_clock().now()

    def on_ped(self, msg):
        self.pedestrian = msg.data
        self.cam_time = self.get_clock().now()

    def age(self, t):
        if t is None:
            return math.inf
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def decide(self, us_ok, cam_ok):
        d = self.distance
        ped = self.pedestrian and cam_ok
        if not us_ok:
            return CollisionWarning.CAUTION, 'ultrasonic data stale'
        if d <= self.brake_m:
            return CollisionWarning.BRAKE, f'obstacle at {d:.2f} m'
        if d <= self.caution_m and ped:
            return CollisionWarning.BRAKE, f'pedestrian at {d:.2f} m'
        if d <= self.caution_m:
            return CollisionWarning.CAUTION, f'obstacle at {d:.2f} m'
        if ped:
            return CollisionWarning.CAUTION, 'pedestrian ahead, out of range'
        return CollisionWarning.SAFE, 'path clear'

    def tick(self):
        us_ok = self.age(self.range_time) < self.stale_s
        cam_ok = self.age(self.cam_time) < self.cam_stale_s
        target, reason = self.decide(us_ok, cam_ok)

        # Escalate immediately, de-escalate only after N consistent cycles
        # (stops the buzzer flickering when a reading sits on a threshold)
        if target >= self.level:
            self.level, self.lower_count = target, 0
        else:
            self.lower_count += 1
            if self.lower_count >= self.release_cycles:
                self.level, self.lower_count = target, 0

        msg = CollisionWarning()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.level = self.level
        msg.distance_m = float(self.distance) if us_ok else math.nan
        msg.pedestrian_detected = bool(self.pedestrian and cam_ok)
        msg.ultrasonic_ok = us_ok
        msg.camera_ok = cam_ok
        msg.reason = f'{NAMES[self.level]}: {reason}'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
