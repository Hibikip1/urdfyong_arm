# 六轴机械臂 — MoveIt2 + ros2_control 完整项目

基于 DM4310 电机 + USB2CANFD 的六轴机械臂 ROS 2 Humble 控制程序。

## 目录结构

```
urdfyong_arm/
├── urdfyong/                  # URDF 机器人模型
│   ├── urdf/                  #   urdfyong.urdf, urdfyong_hardware.urdf
│   ├── meshes/                #   STL 网格文件 (base_link, Link1-6)
│   ├── launch/                #   可视化启动文件
│   └── config/                #   RViz 配置
├── urdfyong_hardware/         # ros2_control 硬件驱动
│   ├── src/                   #   DmMotorHardware (CAN 通信 + 电机控制)
│   ├── include/               #   头文件
│   ├── scripts/               #   trajectory_executor.py (替代 JTC)
│   ├── launch/                #   完整启动文件
│   └── config/                #   控制器参数
├── urdfyong_moveit_config/    # MoveIt2 配置
│   ├── config/                #   SRDF, kinematics, joint_limits, OMPL
│   └── launch/                #   move_group, RViz, demo
├── u2can/                     # USB2CANFD 测试工具
│   ├── src/                   #   test_3rads, test_damiao, set_zero_test
│   └── include/               #   damiao.h, SerialPort.h
├── dm_motor_demo/             # DM 官方 SDK 测试（备用参考）
├── motor_PDF/                 # 电机说明书
└── docs/                      # 文档
```

## 在新电脑上部署

### 1. 克隆仓库
```bash
git clone <本仓库地址>
cd urdfyong_arm
```

### 2. 编译硬件驱动
```bash
cd urdfyong_hardware
colcon build --symlink-install
```

### 3. 编译 URDF 和 MoveIt 配置
```bash
cd ../urdfyong && colcon build --symlink-install
cd ../urdfyong_moveit_config && colcon build --symlink-install
```

### 4. 编译 u2can 测试工具
```bash
cd ../u2can
mkdir -p build && cd build
cmake .. && make
```

### 5. source 并启动
```bash
source /opt/ros/humble/setup.bash
# 按顺序 source 三个包的 install/setup.bash
source urdfyong/install/setup.bash
source urdfyong_hardware/install/setup.bash
source urdfyong_moveit_config/install/setup.bash

# 启动 MoveIt2 + 硬件控制
ros2 launch urdfyong_hardware hardware_moveit.launch.py
```

### 6. 首次校准
连接电机和 USB2CANFD 后：
```bash
# 将关节置于原点位置，然后：
cd u2can/build && ./set_zero_test
```

## 硬件要求

| 组件 | 型号 |
|------|------|
| 电机 | DM4310 × 6（当前连接 1 个：joint1 / CAN ID 0x01） |
| USB2CANFD | HDSC CDC Device (VID:2e88 PID:4603) |
| 串口 | /dev/ttyACM0 @ 921600 baud |
| ROS 2 | Humble |
| 系统 | Ubuntu 22.04 |

## 当前状态 (Stage 1)

- ✅ 单电机 (joint1) MoveIt2 完整控制链路
- ✅ 拖拽、规划、执行正常
- ✅ 电机方向修正 (motor_direction)
- ✅ 位置解码修正 (DM 协议 uint16/65535)
- ⏳ Stage 2: 连接剩余 5 个电机
- ⏳ Stage 3: PID 调优
