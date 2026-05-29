#!/usr/bin/env python3
"""
warehouse_launch.py
===================
Master launch file — ROS2 Jazzy + Gazebo Harmonic.

XACRO STRATEGY (this is why the previous versions failed):
  - subprocess.run(['xacro',...]) fails if xacro is not on PATH at launch time.
  - import xacro; xacro.process_file() fails with ExpatError on ${-PI}.
  - CORRECT approach: Command(['xacro ', path]) — this is the official ROS2
    substitution that runs xacro lazily when robot_state_publisher starts,
    fully inside the ROS2 environment where xacro is guaranteed on PATH.

  For the robot spawner we use '-topic /robot_description' so it reads the
  URDF that robot_state_publisher already processed and published.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── PATHS ─────────────────────────────────────────────────────────────────
    pkg_share = get_package_share_directory('warehouse_wall_follower')

    urdf_xacro_path = os.path.join(pkg_share, 'urdf',   'warehouse_robot.urdf.xacro')
    world_path      = os.path.join(pkg_share, 'worlds', 'warehouse.sdf')
    params_path     = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    bridge_path     = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    rviz_path       = os.path.join(pkg_share, 'rviz',   'warehouse_robot.rviz')

    # ── ROBOT DESCRIPTION via Command substitution ────────────────────────────
    # Command(['xacro ', path]) is the standard ROS2 Jazzy approach.
    # It runs: $ xacro /path/to/robot.urdf.xacro
    # and returns the resulting URDF string when robot_state_publisher starts.
    # This avoids ALL Python xacro module issues and subprocess PATH problems.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro_path]),
            value_type=str
        )
    }

    # ── LAUNCH ARGUMENTS ──────────────────────────────────────────────────────
    declare_use_rviz   = DeclareLaunchArgument('use_rviz',   default_value='true')
    declare_robot_x    = DeclareLaunchArgument('robot_x',    default_value='-14.0')
    declare_robot_y    = DeclareLaunchArgument('robot_y',    default_value='3.0')
    declare_robot_z    = DeclareLaunchArgument('robot_z',    default_value='0.02')
    declare_robot_yaw  = DeclareLaunchArgument('robot_yaw',  default_value='3.0')

    use_rviz  = LaunchConfiguration('use_rviz')
    robot_x   = LaunchConfiguration('robot_x')
    robot_y   = LaunchConfiguration('robot_y')
    robot_z   = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')

    # ── A. GAZEBO HARMONIC ────────────────────────────────────────────────────
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 3 ', world_path]}.items()
    )

    # ── B. ROBOT STATE PUBLISHER ──────────────────────────────────────────────
    # Processes xacro → publishes /robot_description + TF from joint_states.
    # use_sim_time=True so TF timestamps match Gazebo simulation clock.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # ── C. ROBOT SPAWNER (t+5s) ───────────────────────────────────────────────
    # Uses '-topic /robot_description' — reads the URDF that
    # robot_state_publisher already published. No xacro processing needed here.
    spawn_robot = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_robot',
                output='screen',
                arguments=[
                    '-world', 'warehouse',
                    '-topic', '/robot_description',
                    '-name',  'warehouse_robot',
                    '-x',     robot_x,
                    '-y',     robot_y,
                    '-z',     robot_z,
                    '-Y',     robot_yaw,
                ],
            )
        ]
    )

    # ── D. ROS-GZ BRIDGE (t+6s) ───────────────────────────────────────────────
    ros_gz_bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{
                    'config_file': bridge_path,
                    'use_sim_time': True,
                }],
            )
        ]
    )

    # ── E. RVIZ2 (t+7s) ───────────────────────────────────────────────────────
    rviz2 = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_path],
                condition=IfCondition(use_rviz),
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    # ── F. WALL DETECTOR (t+8s) ───────────────────────────────────────────────
    wall_detector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='warehouse_wall_follower',
                executable='wall_detector',
                name='wall_detector',
                output='screen',
                parameters=[params_path, {'use_sim_time': True}],
            )
        ]
    )

    # ── G. PID CONTROLLER (t+8s) ──────────────────────────────────────────────
    pid_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='warehouse_wall_follower',
                executable='pid_controller',
                name='pid_controller',
                output='screen',
                parameters=[params_path, {'use_sim_time': True}],
            )
        ]
    )

    # ── H. STATE MACHINE (t+9s) ───────────────────────────────────────────────
    state_machine = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='warehouse_wall_follower',
                executable='state_machine',
                name='state_machine',
                output='screen',
                parameters=[params_path, {'use_sim_time': True}],
            )
        ]
    )

    # ── I. METRICS NODE (t+9s) ────────────────────────────────────────────────
    metrics_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='warehouse_wall_follower',
                executable='metrics_node',
                name='metrics_node',
                output='screen',
                parameters=[params_path, {'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_robot_x,
        declare_robot_y,
        declare_robot_z,
        declare_robot_yaw,
        LogInfo(msg='[warehouse_launch] Starting — Gazebo loads world, robot spawns at t+5s, nodes start at t+8s'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rviz2,
        wall_detector,
        pid_controller,
        state_machine,
        metrics_node,
    ])
