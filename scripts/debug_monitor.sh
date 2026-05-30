#!/bin/bash
# ================================================================
# SSH 调试终端 — 实时显示系统状态 (替代 rviz)
# 用法: ./scripts/debug_monitor.sh
# ================================================================

WORKSPACE=~/2026Obstacle-race
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

clear
echo "╔══════════════════════════════════════════════════════════╗"
echo "║     2026 Robocon 障碍赛 — 调试监控                      ║"
echo "║     按 Ctrl+C 退出                                      ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ── 1. TF 树 ──
echo "─── TF 树 ───"
frames_stats() {
    echo "  期望: map → odom → base_link → livox_frame/camera_link/imu_link"
    echo "  实际:"
    ros2 run tf2_tools view_frames --text 2>/dev/null | grep -E "Frame|->" | head -10 || \
    ros2 run tf2_ros tf2_echo map base_link --once 2>&1 | head -5
    echo ""
}

# ── 2. LiDAR 状态 ──
lidar_stats() {
    echo "─── LiDAR 状态 ───"
    HZ=$(ros2 topic hz /livox/lidar --window 10 2>&1 | tail -1 | awk '{print $2}')
    echo "  /livox/lidar 频率: ${HZ:-无数据} Hz"
    echo ""
}

# ── 3. 里程计 ──
odom_stats() {
    echo "─── 里程计 (FAST-LIO2) ───"
    ODOM=$(ros2 topic echo /Odometry --once --field pose.pose.position 2>/dev/null)
    if [ -n "$ODOM" ]; then
        X=$(echo "$ODOM" | grep 'x:' | head -1 | awk '{printf "%.2f", $2}')
        Y=$(echo "$ODOM" | grep 'y:' | head -1 | awk '{printf "%.2f", $2}')
        Z=$(echo "$ODOM" | grep 'z:' | head -1 | awk '{printf "%.2f", $2}')
        HZ_O=$(ros2 topic hz /Odometry --window 10 2>&1 | tail -1 | awk '{print $2}')
        echo "  位置: x=${X} y=${Y} z=${Z}"
        echo "  频率: ${HZ_O:-?} Hz"
    else
        echo "  无数据 (LiDAR驱动是否运行?)"
    fi
    echo ""
}

# ── 4. 障碍物检测 ──
obstacle_stats() {
    echo "─── 障碍物检测 ───"
    local data
    data=$(ros2 topic echo /obstacle_info --once 2>/dev/null)
    if [ -n "$data" ]; then
        local type dist head conf
        type=$(echo "$data" | grep 'type:' | head -1 | awk '{print $2}')
        dist=$(echo "$data" | grep 'distance:' | head -1 | awk '{printf "%.2f", $2}')
        head=$(echo "$data" | grep 'heading:' | head -1 | awk '{printf "%.1f", $2}')
        conf=$(echo "$data" | grep 'confidence:' | head -1 | awk '{printf "%.2f", $2}')
        local names=("未知" "绕杆" "砂砾坑" "限高杆" "斜坡" "木桥" "T台阶" "高墙")
        local t=${names[$type]:-"未知"}
        echo "  类型: ${t}  距离: ${dist}m  方位: ${head}°  置信度: ${conf}"
    else
        echo "  未检测到障碍物"
    fi
    echo ""
}

# ── 5. AprilTag ──
tag_stats() {
    echo "─── AprilTag ───"
    local data
    data=$(ros2 topic echo /tag_detections --once 2>/dev/null)
    if echo "$data" | grep -q "tag_id"; then
        echo "$data" | grep -E "tag_id|distance" | head -6
    else
        echo "  未检测到标签"
    fi
    echo ""
}

# ── 6. 导航状态 ──
nav_stats() {
    echo "─── 导航状态 ───"
    CMD=$(ros2 topic echo /cmd_vel --once 2>/dev/null)
    if [ -n "$CMD" ]; then
        VX=$(echo "$CMD" | grep 'x:' | head -1 | awk '{printf "%.3f", $2}')
        VZ=$(echo "$CMD" | grep 'angular' -A3 | grep 'z:' | head -1 | awk '{printf "%.3f", $2}')
        echo "  cmd_vel: vx=${VX} m/s  wz=${VZ} rad/s"
    else
        echo "  cmd_vel: 无输出 (未导航)"
    fi
    echo ""
}

# ── 7. 状态机 ──
fsm_stats() {
    echo "─── 任务状态 ───"
    local data
    data=$(ros2 topic echo /robot_command --once 2>/dev/null)
    if [ -n "$data" ]; then
        local cmd val gait
        cmd=$(echo "$data" | grep 'command_type:' | head -1 | awk '{print $2}')
        val=$(echo "$data" | grep 'value:' | head -1 | awk '{printf "%.2f", $2}')
        local cmds=("STOP" "WALK" "TURN_L" "TURN_R" "CROUCH" "STAND" "CLIMB" "GAIT")
        local c=${cmds[$cmd]:-"?"}
        echo "  指令: ${c}  参数: ${val}"
    else
        echo "  状态机未激活"
    fi
    echo ""
}

# ── 主循环 ──
while true; do
    echo -e "\033[2J\033[H"  # 清屏
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║     2026 Robocon 障碍赛 — 调试监控                      ║"
    echo "║     $(date '+%H:%M:%S')  按 Ctrl+C 退出                              ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo ""
    frames_stats
    lidar_stats
    odom_stats
    obstacle_stats
    tag_stats
    nav_stats
    fsm_stats
    sleep 2
done