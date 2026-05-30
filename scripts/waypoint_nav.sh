#!/bin/bash
# ================================================================
# 航点导航 — 按顺序依次导航至预设航点
# 用法: ./scripts/waypoint_nav.sh [起始障碍物编号 1-8]
# ================================================================

WORKSPACE=~/2026Obstacle-race
source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

START_OBS=${1:-1}
TIMEOUT=60  # 每个航点超时(秒)

# ── 航点定义 (map坐标系, 需实测后修改) ──
# 格式: "障碍物名称 x y z yaw(度)"
declare -A WAYPOINTS
WAYPOINTS[1]="直角绕杆入口 1.0 0.0 0.0 0"
WAYPOINTS[2]="砂砾坑 4.0 -1.0 0.0 0"
WAYPOINTS[3]="限高杆 6.5 0.0 0.0 0"
WAYPOINTS[4]="大斜坡底 8.0 0.0 0.0 0"
WAYPOINTS[5]="木桥A 9.5 1.5 0.0 0"
WAYPOINTS[6]="木桥B 9.5 -1.5 0.0 0"
WAYPOINTS[7]="T台阶底 7.0 2.0 0.0 0"
WAYPOINTS[8]="高墙 3.0 0.0 0.0 0"
WAYPOINTS[9]="终点 0.0 0.0 0.0 0"

echo "╔══════════════════════════════════════════════════════════╗"
echo "║     航点导航模式                                        ║"
echo "║     从障碍物 #${START_OBS} 开始依次导航                         ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

for ((i=START_OBS; i<=9; i++)); do
    NAME=$(echo "${WAYPOINTS[$i]}" | awk '{print $1}')
    X=$(echo "${WAYPOINTS[$i]}" | awk '{print $2}')
    Y=$(echo "${WAYPOINTS[$i]}" | awk '{print $3}')
    Z=$(echo "${WAYPOINTS[$i]}" | awk '{print $4}')
    YAW_DEG=$(echo "${WAYPOINTS[$i]}" | awk '{print $5}')
    YAW=$(echo "scale=6; ${YAW_DEG} * 3.14159 / 180" | bc)
    QZ=$(echo "scale=6; s(${YAW}/2)" | bc -l)
    QW=$(echo "scale=6; c(${YAW}/2)" | bc -l)

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  航点 #${i}: ${NAME}"
    echo "  坐标: (${X}, ${Y}, ${Z})  朝向: ${YAW_DEG}°"
    echo ""

    echo "  发送导航目标..."
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}}}}" \
        --feedback 2>&1 &

    GOAL_PID=$!

    # 等待到达或超时
    START_TIME=$(date +%s)
    while kill -0 $GOAL_PID 2>/dev/null; do
        ELAPSED=$(($(date +%s) - START_TIME))

        # 实时显示当前位置
        ODOM=$(ros2 topic echo /Odometry --once --field pose.pose.position 2>/dev/null)
        if [ -n "$ODOM" ]; then
            CX=$(echo "$ODOM" | grep 'x:' | head -1 | awk '{printf "%.2f", $2}')
            CY=$(echo "$ODOM" | grep 'y:' | head -1 | awk '{printf "%.2f", $2}')
            DX=$(echo "${X} - ${CX}" | bc -l 2>/dev/null)
            DY=$(echo "${Y} - ${CY}" | bc -l 2>/dev/null)
            DIST=$(echo "sqrt(${DX}*${DX} + ${DY}*${DY})" | bc -l 2>/dev/null)
            printf "\r  当前位置: (%.2f, %.2f)  距目标: %.2fm  已用: %ds  " ${CX} ${CY} ${DIST} ${ELAPSED}
        fi

        if [ ${ELAPSED} -gt ${TIMEOUT} ]; then
            echo ""
            echo "  ⚠ 超时! 跳过此航点"
            kill $GOAL_PID 2>/dev/null
            break
        fi

        sleep 1
    done
    echo ""
    echo "  ✓ 航点 #${i} 完成"
    echo ""

    # 到达后短暂停留 (让机器人稳定)
    sleep 1
done

echo "╔══════════════════════════════════════════════════════════╗"
echo "║     全部航点完成!                                       ║"
echo "╚══════════════════════════════════════════════════════════╝"