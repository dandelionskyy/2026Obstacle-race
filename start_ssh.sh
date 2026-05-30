#!/bin/bash
# ================================================================
# SSH 远程启动 — 终端调试 (无显示器)
# 用法: ./start_ssh.sh
#
# 分屏布局:
#  ┌──────────────────────────────────────────┐
#  │  pane 0: 主系统 (无 rviz)                │
#  ├──────────────────────┬───────────────────┤
#  │  pane 1: 调试监控    │  pane 2: LiDAR    │
#  ├──────────────────────┴───────────────────┤
#  │  pane 3: 航点控制                        │
#  └──────────────────────────────────────────┘
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
tmux set-option -t ${SESSION} status-left "Robocon | "
tmux set-option -t ${SESSION} status-right "%H:%M:%S"

# Pane 0: 主系统 (无 rviz)
tmux send-keys -t ${SESSION}:0.0 "
clear
echo '╔════════════════════════════════════════════╗'
echo '║  2026 Robocon — SSH 远程模式               ║'
echo '║  主系统启动中...                           ║'
echo '╚════════════════════════════════════════════╝'
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
ros2 launch robocon_bringup master.launch.py use_rviz:=false
" C-m

# 右侧: 调试监控 + LiDAR
tmux split-window -h -t ${SESSION}:0.0 -l 60

# Pane 1: LiDAR 驱动
tmux send-keys -t ${SESSION}:0.1 "
echo '=== LiDAR 驱动 (Livox MID-360) ==='
source /opt/ros/humble/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 || {
    echo 'ERROR: LiDAR 驱动未安装'
}
exec bash
" C-m

# Pane 2: 调试监控 (在 LiDAR 下方)
tmux split-window -v -t ${SESSION}:0.1 -l 16
tmux send-keys -t ${SESSION}:0.2 "
sleep 10
clear
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
${WORKSPACE}/scripts/debug_monitor.sh
" C-m

# Pane 3: 航点控制 (在主系统下方)
tmux split-window -v -t ${SESSION}:0.0 -l 5
tmux send-keys -t ${SESSION}:0.3 "
cd ${WORKSPACE}
echo '═══════════════════════════════════════════'
echo '  航点导航:  ./scripts/waypoint_nav.sh 1'
echo '═══════════════════════════════════════════'
source /opt/ros/humble/setup.bash
source install/setup.bash
exec bash
" C-m

echo ""
echo "  SSH 远程模式已启动"
echo "  tmux attach -t ${SESSION}"
echo "  Ctrl+B D 断开"
echo ""
tmux attach -t ${SESSION}