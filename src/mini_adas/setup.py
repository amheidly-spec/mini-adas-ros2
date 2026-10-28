from glob import glob
from setuptools import setup

package_name = 'mini_adas'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Mheidly',
    maintainer_email='alimheidly107@gmail.com',
    description='Camera + ultrasonic forward-collision warning on ROS2 Humble',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = mini_adas.arduino_bridge_node:main',
            'camera_node = mini_adas.camera_node:main',
            'detection_node = mini_adas.detection_node:main',
            'fusion_node = mini_adas.fusion_node:main',
        ],
    },
)
