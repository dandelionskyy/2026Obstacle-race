# fast-lio-nav-obstacle

ROS2 Humble 工作空间 — **2026 Robocon 障碍赛** 12自由度四足机器人自主导航系统。

## 硬件
- NVIDIA Jetson Orin Nano 8GB
- Livox MID-360 3D 激光雷达（倒装，头部180°朝下安装）
- Intel RealSense D455 RGB-D 深度相机
- 9轴 IMU（MID-360 内置）
- 自研 12 自由度串联四足机器人

## 架构
```
map → odom → base_link → livox_frame（倒装）
                        → camera_link
                        → imu_link
```

## 功能包

| 功能包 | 类型 | 说明 |
|---------|------|------|
| `robocon_interfaces` | CMake | 自定义 ROS2 消息定义 |
| `robocon_perception` | C++ | 激光雷达障碍物检测与分类 |
| `robocon_apriltag` | C++/Python | AprilTag 检测与全局位姿修正 |
| `robocon_state_machine` | C++ | 任务状态机与各障碍物处理器 |
| `robocon_localization` | C++ | map→odom 变换提供者 |
| `robocon_command_bridge` | C++ | Nav2 /cmd_vel 与 /robot_command 仲裁切换 |
| `robocon_bringup` | Launch | 主启动文件与配置 |
| `FAST_LIO` | C++（复制） | 激光-惯性里程计 |
| `pointcloud_to_laserscan` | C++（复制） | 3D点云转2D激光扫描 |
| `icp_registration` | C++（复制） | ICP 初始全局定位 |
| `robot_navigation2` | 配置（复制） | Nav2 导航参数 |

## 编译
```bash
cd ~/fast-lio-nav-obstacle
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行
```bash
# 全系统启动
ros2 launch robocon_bringup master.launch.py

# 或分别启动各子系统
ros2 launch robocon_bringup static_tfs.launch.py       # 静态坐标变换
ros2 launch robocon_bringup localization.launch.py     # 定位系统
ros2 launch robocon_bringup perception.launch.py       # 感知系统
ros2 launch robocon_bringup navigation.launch.py       # 导航系统
```

## 注意事项
- MID-360 为倒装（180°朝下），静态 TF 和 FAST-LIO2 配置已包含对应旋转
- 所有算法节点均用 C++ 实现以保证实时性能，仅 AprilTag 检测使用 Python（依赖 pupil_apriltags 库）
- 驱动包（`livox_ros_driver2`、`realsense2_camera`）需单独安装/编译，不在本工作空间内
- 首次编译前需从 `~/fast-lio-nav/ros2_humble_main/src/` 复制以下现有功能包：
  - `lio/FAST_LIO/`
  - `mapper/pointcloud_to_laserscan/`
  - `registration/icp_registration/`
  - `navigation/robot_navigation2/`
