#!/bin/bash
# ================================================================
# 纯导航模式 — SLAM + Nav2 + AprilTag 定位修正 + 航点跟随
#
# 用法:
#   ./scripts/start_nav.sh
#
# 不启动: 障碍物检测器 / 任务状态机 / 命令桥接
# 航点定义: src/control/robocon_state_machine/config/waypoints.yaml
# ================================================================
set -e

WORKSPACE=~/2026Obstacle-race

source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null || {
    echo "正在编译..."
    cd ${WORKSPACE}
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
}

echo "启动纯导航模式..."

# ================================================================
# Tab 1: LiDAR 驱动
# ================================================================
gnome-terminal --tab --title="LiDAR" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
echo '=== LiDAR 驱动 ==='
ros2 launch livox_ros_driver2 msg_MID360_launch.py
exec bash
"

sleep 2

# ================================================================
# Tab 2: FAST-LIO2 + 静态 TF + D455 相机
# ================================================================
gnome-terminal --tab --title="SLAM+TF+相机" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo '=== 静态 TF + FAST-LIO2 ==='

# 静态 TF
ros2 run tf2_ros static_transform_publisher \
    --x 0.15 --y 0 --z 0.35 --roll 3.14159 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id livox_frame &
ros2 run tf2_ros static_transform_publisher \
    --x 0.25 --y 0 --z 0.30 --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id camera_link &
ros2 run tf2_ros static_transform_publisher \
    --x 0.15 --y 0 --z 0.35 --roll 3.14159 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id imu_link &

sleep 1

echo '启动 FAST-LIO2...'
ros2 launch fast_lio mapping.launch.py \
    config_file:=mid360.yaml \
    rviz:=false

exec bash
"

# ================================================================
# Tab 3: AprilTag
# ================================================================
gnome-terminal --tab --title="AprilTag" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo '=== 启动 D455 相机 ==='
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=false &

sleep 5

echo '=== AprilTag 检测 + 修正 ==='
ros2 launch robocon_apriltag apriltag.launch.py

exec bash
"

sleep 2

# ================================================================
# Tab 4: Nav2 导航栈 (pc2laser + localization + nav2)
# ================================================================
gnome-terminal --tab --title="Nav2" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo '=== 点云→LaserScan ==='
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args -r cloud_in:=/cloud_registered -r scan:=/scan \
    -p target_frame:=base_link \
    -p min_height:=-0.3 -p max_height:=0.8 \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 \
    -p angle_increment:=0.0087 \
    -p range_min:=0.3 -p range_max:=30.0 \
    -p scan_time:=0.1 &

sleep 2

echo '=== 定位管理器 ==='
ros2 run robocon_localization localization_manager_node \
    --ros-args \
    -p pcd_map_path:=/home/dandelion/fast-lio-nav-obstacle/maps/field_map.pcd \
    -p start_zone:=ground &

sleep 2

echo '=== Nav2 ==='
ros2 launch robot_navigation2 navigation2.launch.py \
    use_rviz:=false \
    map:=/home/dandelion/fast-lio-nav-obstacle/maps/field_map.yaml

exec bash
"

sleep 2

# ================================================================
# Tab 5: 航点跟随
# ================================================================
gnome-terminal --tab --title="航点导航" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo '╔════════════════════════════════════════════╗'
echo '║  航点跟随节点                              ║'
echo '║  配置文件: waypoints.yaml                  ║'
echo '╚════════════════════════════════════════════╝'

echo '等待 Nav2 就绪...'
sleep 15

ros2 run robocon_state_machine waypoint_nav_node.py \
    --ros-args \
    --params-file \$(ros2 pkg prefix robocon_state_machine --share)/config/waypoints.yaml

exec bash
"

# ================================================================
# Tab 6: RViz
# ================================================================
gnome-terminal --tab --title="RViz" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

sleep 5

PKG_SHARE=\$(ros2 pkg prefix robocon_bringup --share)
rviz2 -d \${PKG_SHARE}/config/rviz_config.rviz

exec bash
"

echo ''
echo '==== 纯导航模式已启动 ===='
echo ''
echo '  6 个终端标签页:'
echo '    [LiDAR]        - Livox MID-360 驱动'
echo '    [SLAM+TF+相机] - FAST-LIO2 + 静态TF + D455'
echo '    [AprilTag]     - 标签检测 + 位姿修正'
echo '    [Nav2]         - AMCL定位 + 规划 + 控制'
echo '    [航点导航]     - 自动航点跟随'
echo '    [RViz]         - 3D/2D 可视化'
echo ''
echo '  修改航点: 编辑 src/control/robocon_state_machine/config/waypoints.yaml'
echo ''
