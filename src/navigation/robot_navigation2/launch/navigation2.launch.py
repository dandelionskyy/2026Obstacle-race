import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')
    use_rviz = launch.substitutions.LaunchConfiguration('use_rviz', default='True')
    map_path = launch.substitutions.LaunchConfiguration('map', default='')
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_rviz',
            default_value=use_rviz,
            description='Launch RViz2'
        ),
        launch.actions.DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='2D map file path (empty = no static map)'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file to load'
        ),


        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'map': map_path,
                'use_map_topic': 'true'
            }.items(),
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])