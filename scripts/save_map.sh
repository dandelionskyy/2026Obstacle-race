#!/bin/bash
# ================================================================
# PCD → 2D 占栅格地图 保存流程
# 用法: ./scripts/save_map.sh
# ================================================================

WORKSPACE=~/2026Obstacle-race
PCD_DIR=${WORKSPACE}/maps          # PCD 文件目录
PCD_NAME="scan"                     # PCD 文件名 (不带 .pcd)
MAP_OUTPUT=${WORKSPACE}/maps/field_map  # 输出地图名

source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

echo "========================================"
echo "  PCD → 2D 地图保存"
echo "========================================"
echo "  PCD 目录: ${PCD_DIR}"
echo "  PCD 文件: ${PCD_NAME}.pcd"
echo "  输出地图: ${MAP_OUTPUT}"
echo "========================================"
echo ""

# 1. 检查 PCD 文件
if [ ! -f "${PCD_DIR}/${PCD_NAME}.pcd" ]; then
    echo "❌ 未找到 ${PCD_DIR}/${PCD_NAME}.pcd"
    echo ""
    echo "请先运行 FAST-LIO2 建图, 然后:"
    echo "  ros2 service call /map_save std_srvs/srv/Trigger"
    echo "  cp ~/2026Obstacle-race/src/lio/FAST_LIO/PCD/*.pcd ${PCD_DIR}/"
    echo "  修改本脚本中的 PCD_NAME 为实际文件名"
    exit 1
fi

# 2. 更新 pcd2pgm 配置
CONFIG_FILE=${WORKSPACE}/src/mapper/pcd2pgm/config/pcd.yaml
sed -i "s|file_directory:.*|file_directory: ${PCD_DIR}/|" ${CONFIG_FILE}
sed -i "s|file_name:.*|file_name: ${PCD_NAME}|" ${CONFIG_FILE}

echo "✓ pcd2pgm 配置已更新"

# 3. 启动 pcd2pgm (后台)
echo ""
echo "  启动 pcd2pgm..."
ros2 launch pcd2pgm pcd2pgm.launch.py &
PCD2PGM_PID=$!
sleep 3

# 4. 保存 /map 为 PGM+YAML
echo ""
echo "  保存地图..."
ros2 run nav2_map_server map_saver_cli \
    -f ${MAP_OUTPUT} \
    --ros-args -p map_subscribe_transient_local:=true 2>&1 || {
    echo ""
    echo "⚠ map_saver 失败, 请确认 /map 话题有数据"
    echo "  ros2 topic echo /map --once"
}

# 5. 关闭 pcd2pgm
kill ${PCD2PGM_PID} 2>/dev/null

echo ""
echo "========================================"
echo "  完成!"
echo "  地图文件: ${MAP_OUTPUT}.pgm"
echo "  配置文件: ${MAP_OUTPUT}.yaml"
echo "========================================"
echo ""
echo "Nav2 加载此地图:"
echo "  ros2 launch robocon_bringup master.launch.py map:=${MAP_OUTPUT}.yaml"
echo ""
echo "注意: 需要恢复 nav2_params.yaml 中 global_costmap 的 static_layer"