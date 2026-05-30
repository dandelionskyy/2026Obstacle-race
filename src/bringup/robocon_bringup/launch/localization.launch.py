#!/usr/bin/env python3
"""定位子系统: FAST-LIO2 + pointcloud_to_laserscan + localization_manager。"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # FAST-LIO2 (激光-惯性里程计)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'
                ])
            ]),
            launch_arguments={'config_file': 'mid360.yaml'}.items()
        ),

        # 点云转 LaserScan (供 Nav2 costmap 使用)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_registered'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'base_link',
                'min_height': -0.3,
                'max_height': 0.8,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'range_min': 0.3,
                'range_max': 30.0,
                'scan_time': 0.1,
            }]
        ),

        # 定位管理器 (提供 map->odom 变换)
        Node(
            package='robocon_localization',
            executable='localization_manager_node',
            name='localization_manager',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('robocon_localization'),
                'config', 'localization_params.yaml'
            ])]
        ),
    ])
