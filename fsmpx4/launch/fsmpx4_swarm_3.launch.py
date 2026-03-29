#!/usr/bin/env python3
"""3-UAV swarm launch: per-UAV fsmpx4 executor."""

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Per-UAV config: (px4_ns, target_system_id, self_index)
UAV_CONFIGS = [
    ('/px4_1', '2', '0'),
    ('/px4_2', '3', '1'),
    ('/px4_3', '4', '2'),
]


def _bag_topics():
    topics = [
        '/px4_4/fmu/out/vehicle_global_position',
        '/px4_4/fmu/out/vehicle_local_position',
        '/swarm/land_trigger',
        '/swarm/cmd_target_ned',
        '/swarm/rviz/cmd_target_pose',
        '/swarm/rviz/markers',
        '/visualization_marker_array',
        '/swarm/rviz/payload/pose',
        '/rc/manual_control_setpoint',
    ]
    for px4_ns, _sys_id, _self_index in UAV_CONFIGS:
        ns = px4_ns.lstrip('/')
        topics.extend([
            f'/swarm/rviz/{ns}/pose',
            f'/{ns}/rc/manual_control_setpoint',
            f'/{ns}/fsmpx4_fsm/debug',
            f'/{ns}/swarm_planner/debug',
            f'/{ns}/fmu/in/offboard_control_mode',
            f'/{ns}/fmu/in/vehicle_attitude_setpoint',
            f'/{ns}/fmu/in/vehicle_command',
            f'/{ns}/fmu/out/vehicle_attitude',
            f'/{ns}/fmu/out/vehicle_command_ack',
            f'/{ns}/fmu/out/vehicle_global_position',
            f'/{ns}/fmu/out/vehicle_land_detected',
            f'/{ns}/fmu/out/vehicle_local_position',
            f'/{ns}/fmu/out/vehicle_odometry',
            f'/{ns}/fmu/out/vehicle_status_v1',
        ])
    return topics


def _maybe_start_bag_recording(context, *_args, **_kwargs):
    if LaunchConfiguration('record_bag').perform(context).lower() != 'true':
        return []

    bag_root = os.path.expanduser(LaunchConfiguration('bag_root').perform(context))
    bag_name = LaunchConfiguration('bag_name').perform(context)
    bag_storage = LaunchConfiguration('bag_storage').perform(context)
    record_all_topics = LaunchConfiguration('record_all_topics').perform(context).lower() == 'true'
    os.makedirs(bag_root, exist_ok=True)
    bag_path = os.path.join(bag_root, bag_name)
    cmd = [
        'ros2',
        'bag',
        'record',
        '--storage',
        bag_storage,
        '-o',
        bag_path,
    ]
    if record_all_topics:
        cmd.append('-a')
    else:
        cmd.extend(_bag_topics())

    return [
        ExecuteProcess(
            cmd=cmd,
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('fsmpx4')
    fsm_launch = os.path.join(pkg_share, 'launch', 'fsmpx4_fsm.launch.py')
    default_bag_root = os.path.expanduser('~/swarm_ws/src/data')
    default_bag_name = datetime.now().strftime('swarm_%Y%m%d_%H%M%S_%f')

    actions = [
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='If true, start ros2 bag record alongside the swarm launch',
        ),
        DeclareLaunchArgument(
            'bag_root',
            default_value=default_bag_root,
            description='Directory that will contain the recorded rosbag folder',
        ),
        DeclareLaunchArgument(
            'bag_name',
            default_value=default_bag_name,
            description='Output rosbag folder name for this run',
        ),
        DeclareLaunchArgument(
            'bag_storage',
            default_value='sqlite3',
            description='rosbag2 storage plugin to use, e.g. sqlite3 or mcap',
        ),
        DeclareLaunchArgument(
            'record_all_topics',
            default_value='false',
            description='If true, use ros2 bag record -a instead of the curated experiment topic list',
        ),
        DeclareLaunchArgument(
            'enable_rviz_enu',
            default_value='true',
            description='If true, publish UAV/payload ENU markers and poses for RViz',
        ),
        DeclareLaunchArgument(
            'rviz_fixed_frame',
            default_value='map',
            description='Fixed frame used by the ENU RViz bridge',
        ),
        DeclareLaunchArgument(
            'rviz_publish_rate_hz',
            default_value='20.0',
            description='Publish rate of the ENU RViz bridge',
        ),
        OpaqueFunction(function=_maybe_start_bag_recording),
        Node(
            package='fsmpx4',
            executable='swarm_enu_rviz.py',
            name='swarm_enu_rviz',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'fsm.yaml'),
                {
                    'rviz.fixed_frame': LaunchConfiguration('rviz_fixed_frame'),
                    'rviz.publish_rate_hz': LaunchConfiguration('rviz_publish_rate_hz'),
                },
            ],
            condition=IfCondition(LaunchConfiguration('enable_rviz_enu')),
        ),
    ]
    for px4_ns, sys_id, self_index in UAV_CONFIGS:
        node_namespace = px4_ns.lstrip('/')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fsm_launch),
            launch_arguments={
                'px4_ns': px4_ns,
                'target_system_id': sys_id,
                'self_index': self_index,
                'node_namespace': node_namespace,
            }.items(),
        ))

    return LaunchDescription(actions)
