# 命令类型枚举
uint8 STOP=0
uint8 WALK=1
uint8 TURN_LEFT=2
uint8 TURN_RIGHT=3
uint8 CROUCH=4
uint8 STAND=5
uint8 CLIMB=6
uint8 SPECIAL_GAIT=7

# 步态类型 (与 SPECIAL_GAIT 配合使用)
uint8 GAIT_NORMAL=0
uint8 GAIT_GRAVEL=1
uint8 GAIT_CRAWL=2
uint8 GAIT_STEP_CLIMB=3

uint8 command_type            # 要执行的命令类型
float32 value                 # 速度 (m/s)、角速度 (rad/s)、距离 (m) 等
float32 gait_type             # 当 command_type == SPECIAL_GAIT 时的步态类型
