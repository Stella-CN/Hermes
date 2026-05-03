# Hermes

Hermes 是一个面向两轮轮足机器人的软硬件工程，围绕达妙桌面轮足底盘进行智能化改造与系统集成。

## 项目目的

本项目旨在构建一套分层机器人系统：底层控制器负责姿态稳定、运动控制和安全保护，上层计算平台负责感知、交互、任务管理和后续自主导航能力。

项目希望在保持轮足机器人基础运动能力的前提下，逐步接入深度相机、单板计算机、ROS2 软件栈和本机交互界面，为后续实现避障、建图、定位和自主移动打下基础。

## 主体内容

本工程主要包含以下内容：

- 轮足机器人底盘控制固件。
- LubanCat/ROS2 上位机工程预留。
- RealSense D435 深度视觉相关依赖与规划。
- 运动系统、通信链路、机械改造、电源方案和硬件接口文档。
- 达妙平衡/轮足机器人参考工程与第三方依赖。

## 项目结构

```text
.
├── core/
│   ├── hermes_body/        # STM32 底盘控制固件
│   └── hermes_ros/         # ROS2 / LubanCat 上位机工程
├── docs/
│   ├── damiao_lubancat_d435_robot_plan.md
│   ├── motion_system_lubancat_comm_plan.md
│   └── stm32f407vgt6_pin_config.md
├── deps/
│   ├── librealsense/       # Intel RealSense SDK
│   └── balance_robot/      # 达妙机器人参考工程
├── .gitmodules
├── LICENSE
└── README.md
```

## 开源协议

本项目采用 Apache License 2.0 开源协议，详见 `LICENSE`。

第三方依赖和参考工程遵循其各自目录中的许可证文件。
