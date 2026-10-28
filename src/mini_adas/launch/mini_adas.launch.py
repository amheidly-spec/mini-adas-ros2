"""Bring up the full mini ADAS pipeline with one command:

    ros2 launch mini_adas mini_adas.launch.py
    ros2 launch mini_adas mini_adas.launch.py port:=/dev/ttyUSB0 foxglove:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('mini_adas'), 'config', 'params.yaml')
    port = LaunchConfiguration('port')
    foxglove = LaunchConfiguration('foxglove')

    def tf(name, x, z, child):
        # Static sensor mounting positions relative to the vehicle body
        return Node(
            package='tf2_ros', executable='static_transform_publisher', name=name,
            arguments=['--x', x, '--z', z, '--frame-id', 'base_link', '--child-frame-id', child],
        )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('foxglove', default_value='true'),

        tf('tf_camera', '0.05', '0.10', 'camera_link'),
        tf('tf_ultrasonic', '0.08', '0.05', 'ultrasonic_link'),

        Node(package='mini_adas', executable='arduino_bridge_node',
             parameters=[params, {'port': port}], output='screen'),
        Node(package='mini_adas', executable='camera_node',
             parameters=[params], output='screen'),
        Node(package='mini_adas', executable='detection_node',
             parameters=[params], output='screen'),
        Node(package='mini_adas', executable='fusion_node',
             parameters=[params], output='screen'),

        Node(package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[{'port': 8765}], condition=IfCondition(foxglove)),
    ])
