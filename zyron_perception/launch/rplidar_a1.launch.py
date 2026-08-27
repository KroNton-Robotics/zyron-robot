#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('zyron_perception')
    parameters_file = os.path.join(
        package_share, 'config', 'rplidar_a1.yaml'
    )

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            parameters=[parameters_file],
            output='screen'),
    ])

