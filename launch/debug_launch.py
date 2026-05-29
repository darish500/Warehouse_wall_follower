#!/usr/bin/env python3
"""
debug_launch.py
===============
Debug/development launch file.

PURPOSE:
  Starts only the ROS2 software nodes WITHOUT Gazebo.
  Useful for:
    - Testing node communication with rosbag data
    - Unit testing individual nodes
    - Debugging state machine transitions
    - Monitoring PID diagnostics
    - Running with a real robot

WHAT IT STARTS:
  - wall_detector
  - pid_controller
  - state_machine
  - metrics_node
  - rqt_graph (optional, shows node topology)

DOES NOT START:
  - Gazebo (run separately or use rosbag)
  - robot_state_publisher (not needed for node testing)
  - ros_gz_bridge (not needed without Gazebo)
  - RViz2 (run separately if needed)

USAGE:
  ros2 launch warehouse_wall_follower debug_launch.py
  ros2 launch warehouse_wall_follower debug_launch.py use_rqt:=true

NOTE:
  When running without Gazebo/bridge, use_sim_time should be FALSE.
  Set use_sim_time:=false (default here) so nodes use wall clock.
  If replaying a rosbag with --clock, set use_sim_time:=true.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share  = get_package_share_directory('warehouse_wall_follower')
    params_path = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (true when replaying rosbag with --clock)'
    )

    declare_use_rqt = DeclareLaunchArgument(
        'use_rqt',
        default_value='false',
        description='Launch rqt_graph to visualise node topology'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rqt      = LaunchConfiguration('use_rqt')

    wall_detector = Node(
        package='warehouse_wall_follower',
        executable='wall_detector',
        name='wall_detector',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}],
    )

    pid_controller = Node(
        package='warehouse_wall_follower',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}],
    )

    state_machine = Node(
        package='warehouse_wall_follower',
        executable='state_machine',
        name='state_machine',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}],
    )

    metrics_node = Node(
        package='warehouse_wall_follower',
        executable='metrics_node',
        name='metrics_node',
        output='screen',
        parameters=[params_path, {'use_sim_time': use_sim_time}],
    )

    rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen',
        condition=IfCondition(use_rqt),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rqt,
        LogInfo(msg='[debug_launch] Starting ROS2 nodes only (no Gazebo).'),
        wall_detector,
        pid_controller,
        state_machine,
        metrics_node,
        rqt_graph,
    ])
