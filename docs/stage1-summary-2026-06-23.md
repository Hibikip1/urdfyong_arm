# 6-Axis Robotic Arm — Stage 1 Summary & Future Plan

> **Date**: 2026-06-23  
> **Status**: ✅ Stage 1 Complete — Single motor (joint1) fully operational

## Stage 1 Achievement

单电机（joint1 / CAN ID 0x01）的完整 MoveIt2 控制链路已打通：

- ✅ **USB2CANFD 通信** — u2can 串口协议 @ 921600 baud，`/dev/ttyACM0`
- ✅ **ros2_control 硬件接口** — DmMotorHardware 插件，位置-速度控制模式
- ✅ **MoveIt2 集成** — 规划、拖拽、执行全部正常，电机不走远路
- ✅ **电机方向修正** — motor_direction 参数解决 URDF 与物理电机方向不一致问题
- ✅ **位置解码修正** — 使用 DM 官方协议公式 (uint16/65535)，替代错误的 int16/32768

## Fixes Applied (2026-06-23)

### 1. 位置解码公式修正

DM 电机协议使用 **uint16** 编码位置，映射公式为：

```
position = (raw_uint16 / 65535) × (pos_max - pos_min) + pos_min
```

之前代码使用 `int16/32768` 公式，导致位置反馈完全错误（显示 19~32 rad 等不可能值）。

### 2. 电机方向修正

物理电机旋转方向与 URDF 约定相反。添加 `motor_direction` 参数：

- URDF: `<param name="motor_direction">-1</param>` （joint1）
- 命令路径：发往电机前乘以 motor_direction
- 反馈路径：报告给 MoveIt 前乘以 motor_direction

这样 MoveIt 的 IK 种子和规划在正确的坐标系中工作，拖拽手感正常。

### 3. 死电机状态处理

未连接的电机 (CAN ID 0x02-0x06) 报告位置 0.0，避免 MoveIt 看到不可能的关节角度。

### 4. trajectory_executor 替代 JTC

Humble 的 JointTrajectoryController 存在参数服务 bug，使用自定义 Python executor 替代，通过 `/position_commands` 话题发布位置指令。

## Known Configuration

| 参数 | 值 |
|------|-----|
| 电机型号 | DM4310 |
| 位置范围 | ±12.566 rad (4π) |
| 速度上限 | 2.618 rad/s |
| 关节机械限位 (joint1) | ±2.967 rad (±170°) |
| 控制模式 | POS_VEL (位置-速度) |
| CAN ID | 0x101 (0x100 + 0x01) |

## Future Plan

### Stage 2: 多电机扩展
- 连接剩余 5 个电机 (CAN ID 0x02-0x06)
- 为每个关节设置正确的 motor_direction
- 校准各关节零点位置 (CMD_SET_ZERO)

### Stage 3: 控制系统优化
- 电机 PID 参数调优（减少振荡）
- 速度前馈优化
- 控制周期稳定性改进

### Stage 4: 安全与可靠性
- 关节机械限位保护（在硬件接口层截断超限指令）
- 通信超时检测与急停
- 电机温度/电流监控

### Stage 5: 高级功能
- 多电机同步控制
- 碰撞检测与响应
- Gazebo 仿真集成

## Repository Structure

```
ws_moveit/
├── 6_axis_robotic_arm/
│   ├── urdfyong/                  # URDF 模型 (git repo)
│   ├── urdfyong_hardware/         # ros2_control 硬件接口 (git repo)
│   ├── urdfyong_moveit_config/    # MoveIt2 配置 (git repo)
│   └── C++例程/u2can/            # USB2CANFD 测试工具
└── docs/
    └── stage1-summary-2026-06-23.md  # 本文件
```

## Quick Start

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch urdfyong_hardware hardware_moveit.launch.py
```

首次启动后，需将关节置于原点位置并运行 set_zero_test 校准零点。
