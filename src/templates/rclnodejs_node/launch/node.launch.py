#!/usr/bin/env python3

"""
Launch file for {{package_name}} ROS 2 Node.js node
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    package_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    return LaunchDescription([
        ExecuteProcess(
            cmd=['node', os.path.join(package_dir, 'node.js')],
            cwd=package_dir,
            output='screen',
            name='{{node_name}}'
        )
    ])