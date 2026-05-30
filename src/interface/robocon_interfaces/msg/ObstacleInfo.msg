# 障碍物类型枚举
uint8 UNKNOWN=0
uint8 POLE=1
uint8 GRAVEL_PIT=2
uint8 HEIGHT_BAR=3
uint8 SLOPE=4
uint8 BRIDGE=5
uint8 T_STEPS=6
uint8 HIGH_WALL=7

# 检测到的障碍物信息
uint8 type                    # 障碍物类型枚举值
float32 distance              # 到障碍物的最近距离 (米)
float32 heading               # base_link 坐标系中的相对朝向 (弧度)
float32 confidence            # 检测置信度 [0.0, 1.0]
geometry_msgs/Point[] bounding_box  # 3D 包围盒的 8 个角点 (或最小/最大两个点)
