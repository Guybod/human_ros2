#!/usr/bin/env python3
"""Display and control the real CoDroid dual arms through the UDP API."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('codroid_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'arms_only.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display_arms.rviz')
    mesh_dir = os.path.join(pkg_share, 'meshes')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read().replace(
            'filename="../meshes/', f'filename="file://{mesh_dir}/')

    arguments = [
        DeclareLaunchArgument('robot_ip', default_value='192.168.2.16'),
        DeclareLaunchArgument('command_port', default_value='9001'),
        DeclareLaunchArgument('feedback_port', default_value='9002'),
        DeclareLaunchArgument('auto_connect', default_value='true'),
        DeclareLaunchArgument('frequency_hz', default_value='100.0'),
        DeclareLaunchArgument('default_max_velocity', default_value='0.25'),
        DeclareLaunchArgument('ik_solver', default_value='controller'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('base_height', default_value='1.5'),
    ]

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'publish_frequency': 100.0}],
        output='screen',
    )
    world_to_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('base_height'),
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'world', '--child-frame-id', 'base_link'],
        output='screen',
    )
    bridge = Node(
        package='codroid_description',
        executable='codroid_arm_bridge',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'ik_solver': LaunchConfiguration('ik_solver'),
            'command_port': ParameterValue(LaunchConfiguration('command_port'), value_type=int),
            'feedback_port': ParameterValue(LaunchConfiguration('feedback_port'), value_type=int),
            'auto_connect': ParameterValue(LaunchConfiguration('auto_connect'), value_type=bool),
        }],
        output='screen',
    )
    trajectory = Node(
        package='codroid_description',
        executable='codroid_arm_trajectory',
        parameters=[{
            'robot_description': robot_description,
            'robot_ip': LaunchConfiguration('robot_ip'),
            'frequency_hz': ParameterValue(LaunchConfiguration('frequency_hz'), value_type=float),
            'default_max_velocity': ParameterValue(
                LaunchConfiguration('default_max_velocity'), value_type=float),
        }],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(arguments + [state_publisher, world_to_base, bridge, trajectory, rviz])
