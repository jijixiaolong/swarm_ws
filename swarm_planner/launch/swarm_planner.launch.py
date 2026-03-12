#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('swarm_planner')
    config_file = os.path.join(pkg_share, 'config', 'planner.yaml')

    px4_ns = LaunchConfiguration('px4_ns')
    self_index = LaunchConfiguration('self_index')
    node_namespace = LaunchConfiguration('node_namespace')

    return LaunchDescription([
        DeclareLaunchArgument('px4_ns', default_value='',
                              description='PX4 namespace, e.g. /px4_1'),
        DeclareLaunchArgument('self_index', default_value='-1',
                              description='Planner self index in swarm [0, 1, 2]'),
        DeclareLaunchArgument(
            'node_namespace',
            default_value='',
            description='ROS namespace for swarm_planner and its private topics, e.g. px4_1',
        ),
        Node(
            package='swarm_planner',
            executable='swarm_planner_node',
            name='swarm_planner',
            namespace=node_namespace,
            output='screen',
            parameters=[config_file, {
                'px4_ns': px4_ns,
                'self_index': self_index,
            }],
        ),
    ])
