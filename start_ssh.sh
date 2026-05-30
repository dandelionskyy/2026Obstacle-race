#!/bin/bash
# ================================================================
# SSH 远程启动 — 终端调试 (无显示器)
# 用法: ./start_ssh.sh
#
# 分屏布局:
#  ┌───────────────────────────────┐
#  │  pane 0: 主系统 (无 rviz)     │
#  ├───────────────────────────────┤
#  │  pane 1: 调试监控 (实时状态)   │
#  ├───────────────────────────────┤
#  │  pane 2: 航点控制             │
#  └───────────────────────────────┘
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
tmux new-session -d -s ${SESSION} -n "主系统" -x 120 -y 40
tmux set-option -t ${SESSION} mouse on
tmux set-option -t ${SESSION} status on
tmux set-option -t ${SESSION} status-left "Robocon | "
tmux set-option -t ${SESSION} status-right "%H:%M:%S"

# Pane 0: 主系统 (无 rviz)
tmux send-keys -t ${SESSION}:0.0 "
clear
echo '╔════════════════════════════════════════════╗'
echo '║  2026 Robocon — SSH 远程模式               ║'
echo '║  主系统启动中...                           ║'
echo '╚════════════════════════════════════════════╝'
echo ''
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
ros2 launch robocon_bringup master.launch.py use_rviz:=false
" C-m

# Pane 1: 调试监控
tmux split-window -v -t ${SESSION}:0.0 -l 16
tmux send-keys -t ${SESSION}:0.1 "
sleep 10
clear
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
${WORKSPACE}/scripts/debug_monitor.sh
" C-m

# Pane 2: 航点控制
tmux split-window -v -t ${SESSION}:0.1 -l 5
tmux send-keys -t ${SESSION}:0.2 "
cd ${WORKSPACE}
echo '═══════════════════════════════════════════'
echo '  航点导航:  ./scripts/waypoint_nav.sh 1'
echo '═══════════════════════════════════════════'
echo ''
source /opt/ros/humble/setup.bash
source install/setup.bash
exec bash
" C-m

echo ""
echo "╔════════════════════════════════════════════╗"
echo "║  SSH 远程模式已启动                        ║"
echo "╠════════════════════════════════════════════╣"
echo "║  tmux attach -t ${SESSION}                 ║"
echo "║  Ctrl+B D    退出 (后台继续运行)            ║"
echo "╚════════════════════════════════════════════╝"
echo ""

tmux attach -t ${SESSION}