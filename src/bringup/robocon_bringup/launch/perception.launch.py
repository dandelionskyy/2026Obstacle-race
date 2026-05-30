#!/usr/bin/env python3
"""感知子系统: 障碍物检测 + AprilTag。"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # 障碍物检测器 (C++)
        Node(
            package='robocon_perception',
            executable='obstacle_detector_node',
            name='obstacle_detector',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('robocon_perception'),
                'config', 'obstacle_params.yaml'
            ])]
        ),
        # AprilTag 检测器 (Python)
        Node(
            package='robocon_apriltag',
            executable='apriltag_detector_node.py',
            name='apriltag_detector',
            output='screen',
        ),
        # 标签位姿修正器 (C++)
        Node(
            package='robocon_apriltag',
            executable='tag_pose_corrector_node',
            name='tag_pose_corrector',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('robocon_apriltag'),
                'config', 'tag_world_poses.yaml'
            ])]
        ),
    ])
