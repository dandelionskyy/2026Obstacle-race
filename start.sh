#!/bin/bash
# ================================================================
# 2026 Robocon 障碍赛 — 系统启动脚本
# 用法: ./start.sh [use_rviz:=true/false]
# 示例: ./start.sh                    # 默认启动 rviz
#       ./start.sh use_rviz:=false    # 不启动 rviz
# ================================================================

set -e

WORKSPACE=~/2026Obstacle-race
USE_RVIZ=${1:-use_rviz:=true}

echo "========================================"
echo "  2026 Robocon 障碍赛 系统启动"
echo "  rviz: ${USE_RVIZ}"
echo "========================================"

# 检查 ROS2 环境
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "错误: 未找到 ROS2 Humble, 请先安装"
    exit 1
fi

# Source 环境
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null || {
    echo "警告: 工作空间未编译, 尝试编译..."
    cd ${WORKSPACE}
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source ${WORKSPACE}/install/setup.bash
}

echo ""
echo "环境就绪, 启动各子系统..."
echo ""

# ================================================================
# 终端1: LiDAR 驱动 (需要先安装 livox_ros_driver2)
# ================================================================
gnome-terminal --tab --title="LiDAR驱动" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null
echo '=== LiDAR 驱动 (Livox MID-360) ==='
echo '请确保 livox_ros_driver2 已安装并编译'
echo ''
ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 || {
    echo ''
    echo 'ERROR: LiDAR 驱动未安装!'
    echo '请先安装 livox_ros_driver2:'
    echo '  cd ~/livox_ros_driver2'
    echo '  colcon build'
    echo ''
    read -p '按回车关闭...'
}
exec bash
" 2>/dev/null &

sleep 1

# ================================================================
# 终端2: RealSense D455 相机
# ================================================================
gnome-terminal --tab --title="D455相机" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null
echo '=== RealSense D455 相机 ==='
echo '请确保 realsense2_camera 已安装'
echo ''
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 2>&1 || {
    echo ''
    echo 'ERROR: 相机驱动未安装!'
    echo 'sudo apt install ros-humble-realsense2-camera'
    read -p '按回车关闭...'
}
exec bash
" 2>/dev/null &

sleep 1

# ================================================================
# 终端3: 主系统 (FAST-LIO2 + 感知 + 导航 + 状态机 + 控制)
# ================================================================
gnome-terminal --tab --title="主系统" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
echo '=== 主系统启动 ==='
echo '  - FAST-LIO2 里程计'
echo '  - pointcloud_to_laserscan'
echo '  - obstacle_detector 障碍物检测'
echo '  - AprilTag 检测 + 位姿修正'
echo '  - localization_manager TF'
echo '  - Nav2 导航'
echo '  - mission_fsm 状态机'
echo '  - command_bridge 控制桥接'
echo ''
ros2 launch robocon_bringup master.launch.py ${USE_RVIZ} 2>&1
exec bash
" 2>/dev/null &

echo ""
echo "========================================"
echo "  启动完成!"
echo ""
echo "  终端1: LiDAR 驱动"
echo "  终端2: D455 相机"
echo "  终端3: 主系统"
echo ""
echo "  Rivz2: 按 Ctrl+C 在主系统终端退出"
echo "========================================"