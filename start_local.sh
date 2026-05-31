#!/bin/bash
# ================================================================
# 本机启动 — 带 RViz 可视化 (需要显示器)
# 用法: ./start_local.sh
#
# 分屏布局:
#  ┌──────────────────────────────────────┐
#  │  pane 0: 主系统 + RViz (最大)        │
#  ├──────────────────────┬───────────────┤
#  │  pane 1: LiDAR驱动   │ pane 2: 相机  │
#  └──────────────────────┴───────────────┘
# ================================================================
set -e

WORKSPACE=~/2026Obstacle-race
SESSION="robo2026"

source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null || {
    echo "正在编译..."
    cd ${WORKSPACE}
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
}

tmux kill-session -t ${SESSION} 2>/dev/null || true

# 创建 session
tmux new-session -d -s ${SESSION} -n "主系统" -x 120 -y 40
tmux set-option -t ${SESSION} mouse on

# Pane 0: 主系统 + RViz
tmux send-keys -t ${SESSION}:0.0 "
echo '╔════════════════════════════════════════════╗'
echo '║  2026 Robocon — 本机模式 (RViz)            ║'
echo '╚════════════════════════════════════════════╝'
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
ros2 launch robocon_bringup master.launch.py map:=/home/nano/2026Obstacle-race/maps/field_map.yaml use_rviz:=true

" C-m

# 分割
tmux split-window -h -t ${SESSION}:0
tmux split-window -v -t ${SESSION}:0.1

# Pane 1: LiDAR 驱动
tmux send-keys -t ${SESSION}:0.1 "
echo '=== LiDAR 驱动 (Livox MID-360) ==='
source /opt/ros/humble/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 || {
    echo 'ERROR: LiDAR 驱动未安装'
}
exec bash
" C-m

# Pane 2: D455 相机
tmux send-keys -t ${SESSION}:0.2 "
echo '=== RealSense D455 相机 ==='
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true 2>&1 || {
    echo 'ERROR: 相机驱动未安装'
    echo 'sudo apt install ros-humble-realsense2-camera'
}
exec bash
" C-m

# 调整布局
tmux resize-pane -t ${SESSION}:0.0 -x 70

echo ""
echo "  本机模式已启动 (RViz + LiDAR + 相机)"
echo "  tmux attach -t ${SESSION}"
echo ""
tmux attach -t ${SESSION}
