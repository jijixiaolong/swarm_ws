#!/usr/bin/env python3
"""3-UAV swarm launch: per-UAV fsmpx4 executor + swarm_planner."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Per-UAV config: (px4_ns, target_system_id, self_index)
UAV_CONFIGS = [
    ('/px4_1', '2', '0'),
    ('/px4_2', '3', '1'),
    ('/px4_3', '4', '2'),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('fsmpx4')
    planner_share = get_package_share_directory('swarm_planner')
    fsm_launch = os.path.join(pkg_share, 'launch', 'fsmpx4_fsm.launch.py')
    planner_launch = os.path.join(planner_share, 'launch', 'swarm_planner.launch.py')

    actions = []
    for px4_ns, sys_id, self_index in UAV_CONFIGS:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fsm_launch),
            launch_arguments={
                'px4_ns': px4_ns,
                'target_system_id': sys_id,
            }.items(),
        ))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner_launch),
            launch_arguments={
                'px4_ns': px4_ns,
                'self_index': self_index,
            }.items(),
        ))

    return LaunchDescription(actions)
