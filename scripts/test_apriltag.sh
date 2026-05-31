#!/bin/bash
# ================================================================
# AprilTag 识别测试 — 仅相机 + 检测 + 实时输出相机系位姿
#
# 用法:
#   ./scripts/test_apriltag.sh
#
# 分屏布局:
#  ┌──────────────────────────────────────────────────────┐
#  │  pane 0: 相机 + AprilTag 检测 (可视化窗口)           │
#  ├──────────────────────────────────────────────────────┤
#  │  pane 1: 实时输出 /tag_detections                    │
#  └──────────────────────────────────────────────────────┘
# ================================================================
set -e

WORKSPACE=~/2026Obstacle-race
SESSION="apriltag_test"

# 环境
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null || {
    echo "正在编译..."
    cd ${WORKSPACE}
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
}

# 杀掉旧 session
tmux kill-session -t ${SESSION} 2>/dev/null || true

tmux new-session -d -s ${SESSION} -n "AprilTag测试" -x 120 -y 40
tmux set-option -t ${SESSION} mouse on

# Pane 0: 相机驱动 + AprilTag 检测器 (OpenCV 可视化窗口)
tmux send-keys -t ${SESSION}:0.0 "
echo '╔════════════════════════════════════════════╗'
echo '║  AprilTag 识别测试 — 步骤1               ║'
echo '║  相机驱动 + pupil_apriltags 检测器        ║'
echo '╚════════════════════════════════════════════╝'
echo ''

# 启动 D455 相机
echo '[1/2] 启动 RealSense D455...'
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true &
CAM_PID=\$!

sleep 3

# 启动 AprilTag 检测器 (visualize=true 会弹出 OpenCV 窗口)
echo '[2/2] 启动 AprilTag 检测器...'
ros2 run robocon_apriltag apriltag_detector_node.py \
    --ros-args \
    -p tag_family:=tag36h11 \
    -p tag_size:=0.16 \
    -p image_topic:=/camera/color/image_raw \
    -p camera_info_topic:=/camera/color/camera_info \
    -p visualize:=true

echo ''
echo '检测器已退出 (Ctrl-C)'
kill \$CAM_PID 2>/dev/null || true
exec bash
" C-m

# 分割
tmux split-window -v -t ${SESSION}:0.0

# Pane 1: 实时输出相机坐标系下的位姿
tmux send-keys -t ${SESSION}:0.1 "
echo '╔════════════════════════════════════════════╗'
echo '║  实时输出 /tag_detections                 ║'
echo '║  坐标: 相机光心坐标系 (单位: 米)          ║'
echo '╚════════════════════════════════════════════╝'
echo ''
echo '等待话题...'
sleep 5
ros2 topic echo /tag_detections
" C-m

echo ''
echo '============================================'
echo '  AprilTag 测试环境已启动'
echo '  tmux attach -t ${SESSION}'
echo ''
echo '  观察要点:'
echo '  1. 上方窗格: OpenCV 窗口显示识别画面'
echo '  2. 下方窗格: 相机坐标系下的 (x,y,z) 位姿'
echo '  3. 拿 AprilTag (ID 0-12) 对着相机'
echo '============================================'
echo ''
tmux attach -t ${SESSION}
