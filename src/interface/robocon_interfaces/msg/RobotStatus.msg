# 机器人状态枚举
uint8 STOPPED=0
uint8 WALKING=1
uint8 TURNING=2
uint8 CROUCHING=3
uint8 CROUCHED=4
uint8 CLIMBING=5
uint8 ERROR=6

uint8 state                   # 当前机器人状态
bool is_crouching             # 机器人是否处于匍匐姿态
bool is_climbing              # 机器人是否正在执行攀爬序列
float32 climbing_progress     # 攀爬进度 [0.0, 1.0]
float32 imu_yaw               # 板载 IMU 偏航角 (弧度)
float32 imu_pitch              # 板载 IMU 俯仰角 (弧度)
