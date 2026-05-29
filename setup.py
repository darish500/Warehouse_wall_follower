"""
setup.py
========
Python ament package setup for warehouse_wall_follower.

PURPOSE:
  Registers all Python nodes as console_scripts entry points so that
  `ros2 run warehouse_wall_follower <node_name>` works correctly.
  Also installs all non-Python resource files (launch, urdf, worlds,
  config, rviz) into the ament share directory so launch files can
  find them using ament_index.

NODE ENTRY POINTS:
  wall_detector       - Processes LiDAR scan, extracts wall distance
  pid_controller      - PID control loop publishing /cmd_vel
  state_machine       - Finite State Machine managing robot behaviour
  metrics_node        - Performance metrics and diagnostics publisher

INSTALLED DATA FILES:
  launch/    - All launch files
  urdf/      - Robot URDF/Xacro description
  worlds/    - Gazebo Harmonic SDF world
  config/    - ROS2 parameter YAML files
  rviz/      - RViz2 configuration
  resource/  - ament package marker
"""

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_wall_follower'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament package marker - required for ros2 pkg to find this package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml must be installed into share
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py')) +
            glob(os.path.join('launch', '*.xml'))),

        # URDF / Xacro robot description files
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf')) +
            glob(os.path.join('urdf', '*.xacro'))),

        # Gazebo Harmonic SDF world files
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf')) +
            glob(os.path.join('worlds', '*.world'))),

        # ROS2 parameter configuration YAML files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),

        # RViz2 configuration files
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Warehouse Robot',
    maintainer_email='robot@warehouse.com',
    description='Autonomous Warehouse Inspection Robot using Wall Following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Wall detection node:
            # Subscribes /scan -> filters -> publishes /wall_distance
            'wall_detector = warehouse_wall_follower.wall_detector:main',

            # PID controller node:
            # Subscribes /wall_distance -> computes PID -> publishes /cmd_vel
            'pid_controller = warehouse_wall_follower.pid_controller:main',

            # Finite State Machine node:
            # Orchestrates wall_detector and pid_controller signals
            # Manages SEARCH/FOLLOW/TURN/LOST states
            'state_machine = warehouse_wall_follower.state_machine:main',

            # Performance metrics node:
            # Subscribes to odometry + wall_distance
            # Publishes accumulated statistics
            'metrics_node = warehouse_wall_follower.metrics_node:main',
        ],
    },
)
