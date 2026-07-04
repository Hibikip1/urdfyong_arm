# RDK X5 部署说明

本文档说明如何将本仓库部署到地瓜派 / D-Robotics RDK X5 上，用于六轴机械臂的 ROS 2 Humble、MoveIt2 和 ros2_control 控制链路。

## 目标平台

| 项目 | 要求 |
|------|------|
| 开发板 | RDK X5 |
| 系统 | Ubuntu 22.04 |
| 架构 | ARM64 / aarch64 |
| ROS 2 | Humble |
| 电机通信 | USB2CANFD，默认 `/dev/ttyACM0`，921600 baud |

RDK X5 是 ARM64 平台，不能直接运行 x86_64 电脑上编译出的可执行文件。本仓库没有依赖已提交的 x86_64 预编译库，推荐做法是在 RDK X5 上重新安装依赖并重新编译源码。

## 推荐部署方式

在 RDK X5 上克隆仓库后，从仓库根目录执行：

```bash
./scripts/deploy_rdk_x5_humble.sh
```

脚本会执行以下步骤：

1. 检查系统是否接近目标环境：Ubuntu 22.04、ARM64 / aarch64。
2. 配置 ROS 2 apt 源。
3. 安装 ROS 2 Humble、MoveIt2、ros2_control、RViz、Gazebo 和编译工具。
4. 使用 `rosdep` 安装本仓库三个 ROS 包的系统依赖。
5. 使用 `colcon build` 编译 `urdfyong`、`urdfyong_hardware`、`urdfyong_moveit_config`。
6. 编译 `u2can` 测试工具。
7. 检查 USB2CANFD 串口设备和 `dialout` 用户组权限。
8. 输出后续启动命令。

默认编译并行数是 2，适合 RDK X5 的 4GB/8GB 内存配置。内存紧张时可以降到 1：

```bash
./scripts/deploy_rdk_x5_humble.sh --workers 1
```

如果系统环境已经装好，可以跳过部分步骤：

```bash
./scripts/deploy_rdk_x5_humble.sh --skip-apt
./scripts/deploy_rdk_x5_humble.sh --skip-rosdep
./scripts/deploy_rdk_x5_humble.sh --skip-u2can
```

如果 USB2CANFD 设备不是默认串口：

```bash
./scripts/deploy_rdk_x5_humble.sh --serial-port /dev/ttyACM1
```

脚本不要用 `sudo ./scripts/deploy_rdk_x5_humble.sh` 运行。请用普通用户运行，脚本内部会在需要安装软件或修改权限时调用 `sudo`。

## 启动

部署完成后，重新打开终端：

```bash
cd <urdfyong_arm 仓库路径>
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch urdfyong_hardware hardware_moveit.launch.py
```

如果脚本刚刚把用户加入了 `dialout` 组，需要退出登录并重新登录后，串口权限才会生效。

## 远程控制建议

RDK X5 可以承担机械臂硬件控制，但不建议一开始就把所有图形界面和规划调试都压到板子上。更稳的分工是：

| 设备 | 建议运行内容 |
|------|--------------|
| RDK X5 | `urdfyong_hardware`、`ros2_control_node`、USB2CANFD 串口通信 |
| 电脑 | RViz、MoveIt 可视化、规划调试、远程发送轨迹 |

两边保持同一个 ROS 2 网络环境后，电脑可以远程查看话题、发送 MoveIt 目标或调用 action，RDK X5 专注执行底层电机控制。

部署阶段建议先在 RDK X5 上验证：

```bash
ls -l /dev/ttyACM0
ros2 topic list
ros2 launch urdfyong_hardware hardware_moveit.launch.py
```

确认串口、控制器、`/joint_states` 和 `/joint_trajectory_controller/follow_joint_trajectory` 正常后，再接入电脑端 RViz/MoveIt 工作流。
