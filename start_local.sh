#!/bin/bash
# ================================================================
# 本机启动 — 带 RViz 可视化 (需要显示器)
# 用法: ./start_local.sh
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

# ── Session ──
tmux new-session -d -s ${SESSION} -n "rviz" -x 120 -y 40
tmux set-option -t ${SESSION} mouse on

# Pane 0: RViz + 主系统
tmux send-keys -t ${SESSION}:0.0 "
echo '╔════════════════════════════════════════════╗'
echo '║  RViz 可视化 (显示器模式)                  ║'
echo '╚════════════════════════════════════════════╝'
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
ros2 launch robocon_bringup master.launch.py use_rviz:=true
" C-m

# Pane 1: 航点控制
tmux split-window -v -t ${SESSION}:0.0 -l 10
tmux send-keys -t ${SESSION}:0.1 "
echo '航点导航: ./scripts/waypoint_nav.sh [起始编号]'
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
exec bash
" C-m

echo ""
echo "  本机模式已启动"
echo "  连接: tmux attach -t ${SESSION}"
echo ""

tmux attach -t ${SESSION}