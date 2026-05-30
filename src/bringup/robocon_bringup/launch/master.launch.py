import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Robocon 2026 障碍赛主启动文件。"""

    # ============================================================
    # 启动参数
    # ============================================================
    pcd_map_path_arg = DeclareLaunchArgument(
        'pcd_map_path',
        default_value='/home/dandelion/fast-lio-nav-obstacle/maps/field_map.pcd',
        description='用于 ICP 初始化的预构建 PCD 地图路径'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='2D 占栅格地图路径 (空则不加载)'
    )

    start_zone_arg = DeclareLaunchArgument(
        'start_zone',
        default_value='ground',
        choices=['ground', 't_steps'],
        description='起始区域: ground 或 t_steps'
    )

    use_apriltag_arg = DeclareLaunchArgument(
        'use_apriltag',
        default_value='true',
        description='启用 AprilTag 检测和位姿修正'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='启动 RViz2 可视化 (无显示器时设为 false)'
    )

    # ============================================================
    # 静态坐标变换
    # ============================================================
    # base_link -> livox_frame (MID-360 倒装, 朝下)
    static_livox_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_livox',
        arguments=[
            '--x', '0.15', '--y', '0.0', '--z', '0.35',
            '--roll', '3.14159', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'
        ]
    )

    # base_link -> camera_link (RealSense D455 前向)
    static_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            '--x', '0.25', '--y', '0.0', '--z', '0.30',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link'
        ]
    )

    # base_link -> imu_link (MID-360 内置 IMU, 与 livox_frame 同刚体)
    static_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=[
            '--x', '0.15', '--y', '0.0', '--z', '0.35',
            '--roll', '3.14159', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'imu_link'
        ]
    )

    # ============================================================
    # FAST-LIO2 (激光-惯性里程计)
    # ============================================================
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )

    # ============================================================
    # 点云转 LaserScan (供 Nav2 使用)
    # ============================================================
    pc2laser_node = Node(
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
            'angle_increment': 0.0087,  # ~0.5 度
            'range_min': 0.3,
            'range_max': 30.0,
            'scan_time': 0.1,
        }]
    )

    # ============================================================
    # 感知: 障碍物检测器
    # ============================================================
    obstacle_detector_node = Node(
        package='robocon_perception',
        executable='obstacle_detector_node',
        name='obstacle_detector',
        output='screen',
        parameters=[{
            'cloud_topic': '/cloud_registered',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
        }]
    )

    # ============================================================
    # 视觉: AprilTag 检测 + 位姿修正
    # ============================================================
    apriltag_detector_node = Node(
        package='robocon_apriltag',
        executable='apriltag_detector_node.py',
        name='apriltag_detector',
        output='screen',
        condition=None,  # 将根据条件启动
        parameters=[{
            'tag_family': 'tag36h11',
            'tag_size': 0.16,
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
        }]
    )

    tag_pose_corrector_node = Node(
        package='robocon_apriltag',
        executable='tag_pose_corrector_node',
        name='tag_pose_corrector',
        output='screen',
        parameters=[{
            'tag_poses_file': PathJoinSubstitution([
                FindPackageShare('robocon_apriltag'), 'config', 'tag_world_poses.yaml'
            ]),
        }]
    )

    # ============================================================
    # 导航: Nav2
    # ============================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_navigation2'), 'launch', 'navigation2.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                FindPackageShare('robot_navigation2'), 'config', 'nav2_params.yaml'
            ]),
            'map': LaunchConfiguration('map'),
            'use_map_topic': 'true',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )

    # ============================================================
    # 定位管理器 (提供 map->odom 变换)
    # ============================================================
    localization_manager_node = Node(
        package='robocon_localization',
        executable='localization_manager_node',
        name='localization_manager',
        output='screen',
        parameters=[{
            'pcd_map_path': LaunchConfiguration('pcd_map_path'),
            'start_zone': LaunchConfiguration('start_zone'),
        }]
    )

    # ============================================================
    # 任务状态机
    # ============================================================
    mission_fsm_node = Node(
        package='robocon_state_machine',
        executable='mission_fsm_node',
        name='mission_fsm',
        output='screen',
        parameters=[{
            'mission_params': PathJoinSubstitution([
                FindPackageShare('robocon_state_machine'), 'config', 'mission_params.yaml'
            ]),
        }]
    )

    # ============================================================
    # 命令桥接
    # ============================================================
    command_bridge_node = Node(
        package='robocon_command_bridge',
        executable='command_bridge_node',
        name='command_bridge',
        output='screen',
    )

    # ============================================================
    # 组装启动描述
    # ============================================================
    return LaunchDescription([
        # 参数
        pcd_map_path_arg,
        map_arg,
        start_zone_arg,
        use_apriltag_arg,
        use_rviz_arg,

        # 静态坐标变换 (无时序依赖)
        static_livox_tf,
        static_camera_tf,
        static_imu_tf,

        # FAST-LIO2 (立即启动)
        fast_lio_launch,

        # 点云处理 (等待 FAST-LIO2)
        TimerAction(period=3.0, actions=[pc2laser_node]),

        # 感知 (等待 FAST-LIO2)
        TimerAction(period=3.0, actions=[obstacle_detector_node]),

        # AprilTag (等待相机)
        TimerAction(period=5.0, actions=[
            apriltag_detector_node,
            tag_pose_corrector_node,
        ]),

        # 定位 (等待 FAST-LIO2 + 点云)
        TimerAction(period=5.0, actions=[localization_manager_node]),

        # Nav2 (等待定位 + scan)
        TimerAction(period=8.0, actions=[nav2_launch]),

        # 状态机和命令桥接 (等待所有模块就绪)
        TimerAction(period=10.0, actions=[
            mission_fsm_node,
            command_bridge_node,
        ]),
    ])
