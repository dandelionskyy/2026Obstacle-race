#!/bin/bash
# ================================================================
# AprilTag 识别测试 — 步骤1: 验证相机坐标系下的检测结果
#
# 用法:
#   ./scripts/test_apriltag.sh
#
# 打开 3 个 gnome-terminal 标签页:
#   Tab 1: D455 相机驱动
#   Tab 2: AprilTag 检测器 (OpenCV 可视化窗口 + 相机系位姿)
#   Tab 3: 实时输出 /tag_detections 原始数据
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

echo "启动 AprilTag 识别测试环境..."

# ================================================================
# Tab 1: D455 相机驱动
# ================================================================
gnome-terminal --tab --title="D455 相机" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
echo '=== D455 相机驱动 ==='
echo '等待相机连接...'
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true
exec bash
"

sleep 2

# ================================================================
# Tab 2: AprilTag 检测器 + OpenCV 可视化
# ================================================================
gnome-terminal --tab --title="AprilTag 检测" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
echo '╔════════════════════════════════════════════╗'
echo '║  AprilTag 检测器 (pupil_apriltags)        ║'
echo '║  可视化窗口即将弹出 (OpenCV imshow)        ║'
echo '║                                           ║'
echo '║  输出内容:                                ║'
echo '║  - tag_id:     标签编号                   ║'
echo '║  - x, y, z:    相机光心坐标系 (米)       ║'
echo '║  - distance:   标签到相机距离 (米)        ║'
echo '║  - confidence: 置信度                     ║'
echo '║                                           ║'
echo '║  相机坐标系:                              ║'
echo '║    +x → 右, +y → 下, +z → 前             ║'
echo '╚════════════════════════════════════════════╝'
echo ''

echo '等待相机就绪 (5s)...'
sleep 5

echo '启动检测器...'
ros2 run robocon_apriltag apriltag_detector_node.py \
    --ros-args \
    -p tag_family:=tag36h11 \
    -p tag_size:=0.16 \
    -p image_topic:=/camera/color/image_raw \
    -p camera_info_topic:=/camera/color/camera_info \
    -p visualize:=true
exec bash
"

sleep 1

# ================================================================
# Tab 3: 实时输出原始检测数据
# ================================================================
gnome-terminal --tab --title="检测数据" -- bash -c "
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
echo '=== 等待 /tag_detections 话题... ==='
sleep 8
echo ''
echo '下面实时输出每个检测到的 AprilTag 的相机系位姿:'
echo '  field: detections[].tag_id'
echo '  field: detections[].pose.position.x  (右+)'
echo '  field: detections[].pose.position.y  (下+)'
echo '  field: detections[].pose.position.z  (前+)'
echo '  field: detections[].distance         (米)'
echo '  field: detections[].confidence'
echo '=========================================='
ros2 topic echo /tag_detections
exec bash
"

echo ''
echo '==== AprilTag 测试环境已启动 ===='
echo ''
echo '  3 个终端标签页:'
echo '    [D455 相机]  - 相机驱动日志'
echo '    [AprilTag 检测] - OpenCV 画面 + 终端位姿打印'
echo '    [检测数据]   - /tag_detections 原始数据'
echo ''
echo '  拿一个 AprilTag (ID 0-12) 对着相机,'
echo '  OpenCV 窗口会画出绿框和 ID/距离,'
echo '  终端会打印相机坐标系下的 x, y, z 位姿。'
echo ''
