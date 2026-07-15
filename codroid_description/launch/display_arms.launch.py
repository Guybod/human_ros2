#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file to display the CoDroid robot ARMS ONLY (no end-effectors) in RViz.

Displays:
  - base_link
  - Left arm:  7-DOF (J_arm_l_01 ~ J_arm_l_07)
  - Right arm: 7-DOF (J_arm_r_01 ~ J_arm_r_07)
  - Head:      2-DOF (J_head_yaw, J_head_pitch)

  Total: 17 movable joints (16 arm + 2 head)

Usage:
  ros2 launch codroid_description display_arms.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('codroid_description')

    urdf_path = os.path.join(pkg_share, 'urdf', 'arms_only.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display_arms.rviz')

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

    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

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
