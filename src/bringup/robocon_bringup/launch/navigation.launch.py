#!/usr/bin/env python3
"""导航子系统: Nav2 启动。"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_navigation2'),
                    'launch', 'navigation2.launch.py'
                ])
            ]),
            launch_arguments={
                'params_file': PathJoinSubstitution([
                    FindPackageShare('robot_navigation2'),
                    'config', 'nav2_params.yaml'
                ]),
                'map': '',
                'use_map_topic': 'true',
            }.items()
        ),
    ])
