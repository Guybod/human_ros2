#!/usr/bin/env python3
"""Launch RViz with a visualization-only CoDroid right-arm wave."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('codroid_description')
    urdf_path = os.path.join(share, 'urdf', 'arms_only.urdf')
    mesh_dir = os.path.join(share, 'meshes')
    with open(urdf_path, 'r') as urdf_file:
        description = urdf_file.read().replace(
            'filename="../meshes/', f'filename="file://{mesh_dir}/')
    return LaunchDescription([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description}], output='screen'),
        Node(
            package='codroid_description', executable='codroid_wave_demo',
            output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'display_arms.rviz')],
            output='screen'),
    ])
