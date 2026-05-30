#!/usr/bin/env python3
"""AprilTag 检测和位姿修正启动文件。"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # AprilTag 检测器 (Python — 使用 pupil_apriltags)
        Node(
            package='robocon_apriltag',
            executable='apriltag_detector_node.py',
            name='apriltag_detector',
            output='screen',
            parameters=[{
                'tag_family': 'tag36h11',
                'tag_size': 0.16,
                'image_topic': '/camera/color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'visualize': True,
            }]
        ),
        # 标签位姿修正器 (C++ — 从标签观测计算全局位姿)
        Node(
            package='robocon_apriltag',
            executable='tag_pose_corrector_node',
            name='tag_pose_corrector',
            output='screen',
            parameters=[{
                'tag_poses_file': PathJoinSubstitution([
                    FindPackageShare('robocon_apriltag'),
                    'config', 'tag_world_poses.yaml'
                ]),
                'ema_alpha': 0.3,
                'max_translation_jump': 0.5,
                'max_rotation_jump': 15.0,
                'min_correction_interval': 0.5,
            }]
        ),
    ])
