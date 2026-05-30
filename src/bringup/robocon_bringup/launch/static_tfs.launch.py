#!/usr/bin/env python3
"""机器人的静态坐标变换发布器。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # base_link -> livox_frame (MID-360 倒装, 180° 朝下)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_livox',
            arguments=[
                '--x', '0.15', '--y', '0.0', '--z', '0.35',
                '--roll', '3.14159', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'
            ]
        ),
        # base_link -> camera_link (RealSense D455, 前向)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            arguments=[
                '--x', '0.25', '--y', '0.0', '--z', '0.30',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link'
            ]
        ),
        # base_link -> imu_link (MID-360 内置 IMU)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=[
                '--x', '0.15', '--y', '0.0', '--z', '0.35',
                '--roll', '3.14159', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link'
            ]
        ),
    ])
