#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file to display the CoDroid humanoid robot with HAND end-effectors in RViz.

Displays:
  - Full robot body (base_link, dual 7-DOF arms, 2-DOF head)
  - Left and right dexterous hands (5 fingers each with force sensors)

Usage:
  ros2 launch codroid_description display_hand.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('codroid_description')

    # Paths
    urdf_path = os.path.join(pkg_share, 'urdf', 'hand.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display_hand.rviz')

    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch joint_state_publisher_gui'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )
    base_height_arg = DeclareLaunchArgument(
        'base_height', default_value='1.5',
        description='Height of base_link above world in meters')

    mesh_dir = os.path.join(pkg_share, 'meshes')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # Replace relative mesh paths with file:// absolute paths
    robot_desc = robot_desc.replace(
        'filename="../meshes/',
        f'filename="file://{mesh_dir}/'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
            'publish_frequency': 50.0,
        }]
    )
    world_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('base_height'),
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'world', '--child-frame-id', 'base_link'])

    # Joint state publisher (with GUI for manual joint control)
    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Fallback: headless joint state publisher
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        base_height_arg,
        robot_state_pub,
        world_to_base,
        joint_state_pub_gui,
        joint_state_pub,
        rviz_node,
    ])
