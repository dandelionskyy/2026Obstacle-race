#!/bin/bash
# ================================================================
# 2026 Robocon 障碍赛 — tmux 分屏启动 (SSH友好)
# 用法: ./start_tmux.sh [use_rviz:=false]
#
# 分屏布局:
#  ┌──────────────────────────────────────────┐
#  │  pane 0: 主系统                          │
#  │  (FAST-LIO2 + 感知 + 导航 + 状态机)       │
#  ├──────────────────────┬───────────────────┤
#  │  pane 1: 调试监控    │  pane 2: LiDAR    │
#  │  (TF/里程计/障碍物)   │                   │
#  ├──────────────────────┴───────────────────┤
#  │  pane 3: 航点控制                        │
#  └──────────────────────────────────────────┘
# ================================================================

set -e

WORKSPACE=~/2026Obstacle-race
USE_RVIZ=${1:-use_rviz:=false}
SESSION="robocon2026"

if ! command -v tmux &>/dev/null; then
    echo "tmux 未安装: sudo apt install tmux"
    exit 1
fi

source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash 2>/dev/null || {
    echo "工作空间未编译, 正在编译..."
    cd ${WORKSPACE}
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
}

tmux kill-session -t ${SESSION} 2>/dev/null || true

# ── 创建 session ──
tmux new-session -d -s ${SESSION} -n "主系统" -x 120 -y 40
tmux set-option -t ${SESSION} mouse on
tmux set-option -t ${SESSION} status on
tmux set-option -t ${SESSION} status-style "bg=blue,fg=white"
tmux set-option -t ${SESSION} status-left "Robocon2026 |"

# ── Pane 0: 主系统 ──
tmux send-keys -t ${SESSION}:0.0 "
echo '╔══════════════════════════════════════════════════════╗'
echo '║  2026 Robocon 障碍赛 — 主系统                       ║'
echo '║  FAST-LIO2 | 感知 | 导航 | 状态机 | 控制桥          ║'
echo '╚══════════════════════════════════════════════════════╝'
echo ''
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
ros2 launch robocon_bringup master.launch.py ${USE_RVIZ}
" C-m

# ── 分割: 右侧调试 + LiDAR ──
tmux split-window -v -t ${SESSION}:0.0 -l 12

# ── Pane 1: 调试监控 ──
tmux send-keys -t ${SESSION}:0.1 "
echo '╔══════════════════════════════════════════════════════╗'
echo '║  调试监控  (2秒刷新)                                 ║'
echo '╚══════════════════════════════════════════════════════╝'
sleep 15  # 等待主系统启动
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash
${WORKSPACE}/scripts/debug_monitor.sh
" C-m

# ── 分割调试面板右侧: LiDAR驱动 ──
tmux split-window -h -t ${SESSION}:0.1
tmux send-keys -t ${SESSION}:0.2 "
echo '╔══════════════════════════════════════════════════════╗'
echo '║  LiDAR 驱动 (Livox MID-360)                          ║'
echo '╚══════════════════════════════════════════════════════╝'
echo ''
source /opt/ros/humble/setup.bash
if ros2 pkg list 2>/dev/null | grep -q livox_ros_driver2; then
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
else
    echo '⚠  livox_ros_driver2 未安装'
    echo '   请先编译 Livox SDK 和 ROS2 驱动'
    exec bash
fi
" C-m

# ── 回到主面板, 再分割底部: 航点控制 ──
tmux select-pane -t ${SESSION}:0.0
tmux split-window -h -t ${SESSION}:0.0 -l 50
tmux send-keys -t ${SESSION}:0.3 "
echo '╔══════════════════════════════════════════════════════╗'
echo '║  航点导航控制                                        ║'
echo '║  用法: cd ${WORKSPACE}                                ║'
echo '║        ./scripts/waypoint_nav.sh [起始编号]           ║'
echo '║        ./scripts/waypoint_nav.sh 1  (从头开始)       ║'
echo '╚══════════════════════════════════════════════════════╝'
echo ''
echo '等待系统就绪后执行:'
echo '  ./scripts/waypoint_nav.sh'
echo ''
exec bash
" C-m

# ── 最终布局调整 ──
tmux select-layout -t ${SESSION}:0 tiled

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  tmux 会话: ${SESSION}                               ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  连接:  tmux attach -t ${SESSION}                     ║"
echo "║  断开:  Ctrl+B D                                     ║"
echo "║  切换:  Ctrl+B 方向键                                 ║"
echo "║  关闭:  tmux kill-session -t ${SESSION}               ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

tmux attach -t ${SESSION}