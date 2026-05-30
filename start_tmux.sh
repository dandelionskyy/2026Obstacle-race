#!/bin/bash
# ================================================================
# 2026 Robocon 障碍赛 — tmux 分屏启动脚本
# 用法: ./start_tmux.sh [use_rviz:=true/false]
#
# 分屏布局:
#  ┌──────────────────────────────────────┐
#  │  pane 0: 主系统 (最大)               │
#  │  - FAST-LIO2 + 感知 + 导航 + 状态机  │
#  ├──────────────────────┬───────────────┤
#  │  pane 1: LiDAR驱动   │ pane 2: 相机  │
#  └──────────────────────┴───────────────┘
# ================================================================

set -e

WORKSPACE=~/2026Obstacle-race
USE_RVIZ=${1:-use_rviz:=true}
SESSION="robocon2026"

# 检查 tmux
if ! command -v tmux &>/dev/null; then
    echo "tmux 未安装, 请先安装: sudo apt install tmux"
    exit 1
fi

# Source 环境
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null || {
    echo "警告: 工作空间未编译, 尝试编译..."
    cd ${WORKSPACE}
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
}

# 杀掉已有同名 session
tmux kill-session -t ${SESSION} 2>/dev/null || true

# 创建 session
tmux new-session -d -s ${SESSION} -n "主系统"
tmux set-option -t ${SESSION} mouse on

# Pane 0: 主系统 (大窗口)
tmux send-keys -t ${SESSION}:0.0 "
echo '=== 2026 Robocon 障碍赛 — 主系统 ==='
echo '  FAST-LIO2 | 感知 | 导航 | 状态机 | 控制桥'
echo ''
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
ros2 launch robocon_bringup master.launch.py ${USE_RVIZ}
" C-m

# 分割窗口
tmux split-window -h -t ${SESSION}:0
tmux split-window -v -t ${SESSION}:0.1

# Pane 1: LiDAR 驱动
tmux send-keys -t ${SESSION}:0.1 "
echo '=== LiDAR 驱动 (Livox MID-360) ==='
source /opt/ros/humble/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 || {
    echo 'ERROR: LiDAR 驱动未安装, 请先编译 livox_ros_driver2'
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

# 调整布局 (主系统大, 驱动和相机小)
tmux resize-pane -t ${SESSION}:0.0 -x 60

echo ""
echo "========================================"
echo "  tmux 会话已启动: ${SESSION}"
echo "========================================"
echo ""
echo "  连接会话:  tmux attach -t ${SESSION}"
echo "  离开会话:  Ctrl+B 然后按 D"
echo "  关闭会话:  tmux kill-session -t ${SESSION}"
echo ""
echo "  快捷键:"
echo "    Ctrl+B →  切换到右侧面板"
echo "    Ctrl+B ↑  切换到上方面板"
echo "    Ctrl+B D  断开连接 (后台运行)"
echo ""
echo "========================================"

# 自动连接
tmux attach -t ${SESSION}