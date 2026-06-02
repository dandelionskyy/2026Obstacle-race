"""Open3D 全局定位节点启动文件 (适配 fast-lio-nav-obstacle).

仅启动 global_localization_node.
FAST-LIO、pointcloud_to_laserscan、Nav2 由 master.launch.py 统一管理.
不需要 camera_init / motion_link 静态 TF (与 luckrobot 不同).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='启动 RViz2'
    )

    config_file = PathJoinSubstitution([open3d_loc_share, 'config', 'loc_param_g1.yaml'])

    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            config_file,
            {
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.02,
                'voxelsize_fine': 0.3,
                'threshold_fitness': 0.5,
                'threshold_fitness_init': 0.5,
                'loc_frequence': 2.0,
                'save_scan': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 300000,
                'filter_odom2map': False,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,
            }
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        global_localization_node,
    ])
