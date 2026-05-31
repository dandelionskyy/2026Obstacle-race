#!/bin/bash
# ================================================================
# 感知测试 — 步骤2: LiDAR 地面分割 + 障碍物检测 + 分类
#
# 用法:
#   ./scripts/test_perception.sh
#
# 打开 4 个 gnome-terminal 标签页:
#   Tab 1: Livox MID-360 LiDAR 驱动
#   Tab 2: FAST-LIO2 里程计 (发布 /cloud_registered + /Odometry)
#   Tab 3: 障碍物检测器 (地面分割 + 聚类 + PCA 分类)
#   Tab 4: RViz 可视化
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

echo "启动感知测试环境..."

# ================================================================
# Tab 1: LiDAR 驱动
# ================================================================
gnome-terminal --tab --title="LiDAR 驱动" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
echo '=== Livox MID-360 LiDAR 驱动 ==='
ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 || {
    echo 'ERROR: LiDAR 驱动未安装!'
    echo '请先安装 livox_ros_driver2'
}
exec bash
"

sleep 2

# ================================================================
# Tab 2: FAST-LIO2 (里程计 + 点云)
# ================================================================
gnome-terminal --tab --title="FAST-LIO2" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo '╔════════════════════════════════════════════╗'
echo '║  FAST-LIO2 激光惯性里程计                  ║'
echo '║  输出: /Odometry, /cloud_registered, /path ║'
echo '╚════════════════════════════════════════════╝'

# 发布静态 TF: map->odom 恒等 (测试用)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
# 发布静态 TF: odom->base_link 恒等 (FAST-LIO 会覆盖, 先占位)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
# 发布静态 TF: base_link->livox_frame (MID-360 倒装)
ros2 run tf2_ros static_transform_publisher \
    0.15 0 0.35 3.14159 0 0 base_link livox_frame &

echo '等待 LiDAR 数据 (5s)...'
sleep 5

echo '启动 FAST-LIO2...'
ros2 launch fast_lio mapping.launch.py \
    config_file:=mid360.yaml \
    rviz:=false

exec bash
"

sleep 3

# ================================================================
# Tab 3: 障碍物检测器 (调试全开)
# ================================================================
gnome-terminal --tab --title="障碍物检测" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo '╔════════════════════════════════════════════╗'
echo '║  障碍物检测器 (调试可视化全开)             ║'
echo '║                                           ║'
echo '║  管道:                                    ║'
echo '║  ROI滤波 → 体素降采样 → SOR去噪          ║'
echo '║  → RANSAC地面分割 → 欧几里得聚类         ║'
echo '║  → PCA特征提取 → 决策树分类 (7类)        ║'
echo '║                                           ║'
echo '║  发布话题:                                ║'
echo '║  /debug/ground_cloud    - 地面点(绿)      ║'
echo '║  /debug/non_ground_cloud- 非地面点(白)    ║'
echo '║  /debug/cluster_cloud   - 聚类着色点云    ║'
echo '║  /obstacle_markers      - 文字标签        ║'
echo '║  /obstacle_info         - 障碍物信息      ║'
echo '╚════════════════════════════════════════════╝'

echo '等待 FAST-LIO2 点云 (8s)...'
sleep 8

echo '启动障碍物检测器...'
ros2 run robocon_perception obstacle_detector_node \
    --ros-args \
    -p cloud_topic:=/cloud_registered \
    -p base_frame:=base_link \
    -p publish_debug_clouds:=true

exec bash
"

sleep 1

# ================================================================
# Tab 4: RViz 可视化
# ================================================================
gnome-terminal --tab --title="RViz" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo '=== RViz2 可视化 ==='
echo ''
echo '  显示列表:'
echo '  - /cloud_registered     原始点云 (彩虹色)'
echo '  - /debug/ground_cloud    地面点 (绿色)'
echo '  - /debug/non_ground_cloud 非地面点 (白色)'
echo '  - /debug/cluster_cloud   聚类按类型着色'
echo '  - /obstacle_markers      障碍物文字标签'
echo '  - /path                  运动轨迹'
echo ''

sleep 2

PKG_SHARE=\$(ros2 pkg prefix robocon_bringup --share)
rviz2 -d \${PKG_SHARE}/config/rviz_config.rviz

exec bash
"

# ================================================================
# Tab 5: 实时数据监控
# ================================================================
gnome-terminal --tab --title="检测数据" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo '=== 等待 /obstacle_info ==='
sleep 12
echo ''
echo '实时障碍物检测结果:'
echo '  type:       障碍物类型 (1-7)'
echo '  distance:   距离 (米)'
echo '  heading:    方位角 (弧度)'
echo '  confidence: 置信度 [0-1]'
echo '=========================================='
ros2 topic echo /obstacle_info
exec bash
"

echo ''
echo '==== 感知测试环境已启动 ===='
echo ''
echo '  5 个终端标签页:'
echo '    [LiDAR 驱动]    - Livox MID-360'
echo '    [FAST-LIO2]     - 里程计 + /cloud_registered'
echo '    [障碍物检测]     - 地面分割 + 聚类 + 分类'
echo '    [RViz]          - 3D 可视化'
echo '    [检测数据]       - /obstacle_info 原始数据'
echo ''
echo '  RViz 中观察:'
echo '    绿色点云  = 地面 (RANSAC)'
echo '    白色点云  = 非地面障碍物'
echo '    彩色点云  = 聚类按类型着色:'
echo '       蓝=杆  青=限高杆  红=墙'
echo '       橙=坡  紫=桥    黄=台阶  棕=砾石'
echo '    文字标签  = 障碍物名称 (TEXT_VIEW_FACING)'
echo ''
