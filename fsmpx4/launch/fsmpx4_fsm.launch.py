#!/usr/bin/env python3
"""Parameterized single-UAV FSM launch. Replaces fsmpx4_px4_1/2/3.launch.py."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('fsmpx4')
    config_file = os.path.join(pkg_share, 'config', 'fsm.yaml')

    px4_ns = LaunchConfiguration('px4_ns')
    target_system_id = LaunchConfiguration('target_system_id')
    node_namespace = LaunchConfiguration('node_namespace')

    return LaunchDescription([
        DeclareLaunchArgument('px4_ns', default_value='',
                              description='PX4 namespace, e.g. /px4_1'),
        DeclareLaunchArgument('target_system_id', default_value='1',
                              description='MAVLink target system ID'),
        DeclareLaunchArgument(
            'node_namespace',
            default_value='',
            description='ROS namespace for fsmpx4_fsm and its private topics, e.g. px4_1',
        ),
        Node(
            package='fsmpx4',
            executable='fsm_node',
            name='fsmpx4_fsm',
            namespace=node_namespace,
            output='screen',
            parameters=[config_file, {
                'px4_ns': px4_ns,
                'vehicle_command.target_system_id': target_system_id,
            }],
        ),
    ])
