<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
    <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="88">
  </picture>
</p>
<h1 align="center">uBotSensor</h1>
<p align="center"><b>为 Unreal Engine 5.5 提供仿真传感器：位姿、IMU、里程计、激光雷达和相机。</b></p>
<p align="center">
  <a href="https://github.com/AstreoX/uBotSensor/releases"><img alt="release" src="https://img.shields.io/github/v/release/AstreoX/uBotSensor?label=release"></a>
  <a href="LICENSE"><img alt="license" src="https://img.shields.io/github/license/AstreoX/uBotSensor"></a>
  <img alt="Unreal Engine 5.5" src="https://img.shields.io/badge/Unreal%20Engine-5.5-313131">
</p>
<p align="center">
  <a href="https://astreox.github.io/uBotDocs/sensor/">文档</a> ·
  <a href="https://github.com/AstreoX/uBotSensor/releases">发布版本</a> ·
  <a href="https://github.com/users/AstreoX/projects/2">路线图</a> ·
  <a href="https://github.com/AstreoX/uBotSensor/issues">问题反馈</a>
</p>

[English](README.md) | **简体中文**

## 简介

uBotSensor 为 Actor 提供仿真传感器：位姿、里程计、IMU、2D / 3D 激光雷达、RGB 相机和深度相机。每种传感器都是在编辑器里添加的组件，可以在蓝图或 C++ 中读取数据。它建立在 [uBotCore](https://github.com/AstreoX/uBotCore) 之上，由 Core 提供仿真时钟和每个样本都带的帧头；目前处于 Beta 阶段。

- **蓝图优先**：每种传感器都在组件列表的 `uBot` 分类下，属性、函数和事件都暴露给蓝图。
- **没有 ROS 依赖**：输出是 UE 原生的结构体和对象，位置、距离和速度用米和秒，角度用弧度；发布到 ROS 2 交给 [uBotROS](https://github.com/AstreoX/uBotROS)。
- **按仿真时间采样**：定时采样跟随仿真时钟，每个 world tick 最多一个样本，所以时间戳严格递增；噪声使用固定种子。
- **自描述的帧**：每个样本都带帧头，包含传感器 ID、坐标系 ID、序号、时间戳和状态。
- **相机同步**：采集协调器让多个相机在同一个仿真时刻采集。

| 组件 | 输出 | 默认频率 |
| --- | --- | --- |
| `uBot Pose Sensor` | 位置（m）、姿态四元数、可选的速度 | 30 Hz |
| `uBot Odometry Sensor` | 相对起点的位姿（m）、本地坐标系下的速度、协方差 | 30 Hz |
| `uBot IMU Sensor` | 姿态、角速度（rad/s）、比力（m/s²）、方差 | 100 Hz |
| `uBot 2D LiDAR Sensor` | 每个角度的距离、命中标志和强度 | 10 Hz |
| `uBot 3D LiDAR Sensor` | 点云：命中点（m）、环号、强度 | 10 Hz |
| `uBot RGB Camera Sensor` | `BGRA8` 图像，像素按需读回 CPU | 30 Hz |
| `uBot Depth Camera Sensor` | `Depth32FC1` 深度图（m），0 表示无效 | 30 Hz |
| 感知介质（烟雾） | 不是传感器：截断激光雷达射线，给 RGB 图像染色，把深度改为入口距离 | 无 |

## 安装

**推荐：用 uBot Manager。** 到 [uBotManager 的 Releases 页面](https://github.com/AstreoX/uBotManager/releases)下载安装包，打开工程的 `.uproject`，在 **Package** 视图里选中 `UBotSensor`，点 **安装**，再点 **应用…**。uBotCore 会一起装好，插件位于 `Plugins/uBot/`。

**手动安装：** 把两个仓库克隆到工程里，在 `Edit > Plugins` 中启用 **uBot Core** 和 **uBot Sensor**（分类 uBot），然后重启编辑器：

```powershell
mkdir <Project>/Plugins/uBot -Force; cd <Project>/Plugins/uBot
git clone --branch v0.1.2 https://github.com/AstreoX/uBotCore.git
git clone --branch v0.1.1 https://github.com/AstreoX/uBotSensor.git
```

**要求：** Unreal Engine 5.5 和 uBotCore ^0.1.0。从源码安装要编译 C++，需要带 C++ 工具链的 Visual Studio 2022；uBot Manager 运行在 Windows 10 / 11 上，条件允许时优先下载预编译包。

## 快速开始

给一个 Actor 加上 IMU，看它采样：

1. 在 Actor 蓝图的 **Components** 面板点 **Add**，选择 `uBot IMU Sensor`。
2. 选中该组件，在 **Details** 的 **Events** 区点 `On Sensor Sampled` 旁的 **+**。
3. 把 `Header` 经 `Break` 节点接出，将 `Sequence` 连到 `Print String`。
4. 把 Actor 放进关卡并点 Play。屏幕上的数字不断增加（`Auto Start` 默认开启；IMU 采样频率 100 Hz，每帧最多采样一次）。

## 文档

| 我想…… | 去这里 |
| --- | --- |
| 跟着完整的第一个传感器教程做 | [五分钟上手](https://astreox.github.io/uBotDocs/start/quickstart/) |
| 查看所有传感器和它们的输出 | [uBotSensor 概述](https://astreox.github.io/uBotDocs/sensor/) |
| 启动、停止和读取任意传感器 | [传感器通用用法](https://astreox.github.io/uBotDocs/sensor/basics/) |
| 选择采样频率 | [性能与采样频率](https://astreox.github.io/uBotDocs/sensor/performance/) |
| 加入干扰激光雷达和相机的烟雾 | [感知介质（烟雾干扰）](https://astreox.github.io/uBotDocs/sensor/medium/) |
| 把传感器数据发布到 ROS 2 | [发布传感器数据](https://astreox.github.io/uBotDocs/ros/publish-sensors/) |
| 在 C++ 里使用传感器 | [在 C++ 中使用传感器与接口](https://astreox.github.io/uBotDocs/develop/cpp/) |
| 从 AgentSensorCore 迁移 | [从旧插件迁移](https://astreox.github.io/uBotDocs/develop/migration/) |
| 了解模块划分和采样调度的实现 | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| 运行自动化测试 | [docs/TESTING.md](docs/TESTING.md) |

## uBot 家族

| 仓库 | 说明 |
| --- | --- |
| [uBotCore](https://github.com/AstreoX/uBotCore) | 仿真时钟、帧头、共享接口、插件管理 |
| **[uBotSensor](https://github.com/AstreoX/uBotSensor)** | **位姿、IMU、里程计、2D/3D 激光雷达、RGB 与深度相机** |
| [uBotROS](https://github.com/AstreoX/uBotROS) | 通过 TCP 桥接入 ROS 2：传感器、TF、/clock、cmd_vel |
| [uBotAerial](https://github.com/AstreoX/uBotAerial) | 多旋翼无人机：简化飞行、刚体动力学、风 |
| [uBotManager](https://github.com/AstreoX/uBotManager) | 在 UE 工程里安装、更新、构建 uBot 包的桌面程序和命令行 |
| [uBotDocs](https://github.com/AstreoX/uBotDocs) | 文档站：https://astreox.github.io/uBotDocs/ |

## 开发

- **构建：** 把仓库放到 `<Project>/Plugins/uBot/uBotSensor` 并启用 uBotCore；首次打开工程时，编辑器会提示编译 `UBotSensor` 和 `UBotSensorEditor` 两个模块。
- **测试：** 测试是名称以 `UBotSensor.` 开头的 Unreal 自动化测试，可在 Session Frontend 或命令行运行，见 [docs/TESTING.md](docs/TESTING.md)。
- **实现细节：** 模块、能力键和采样调度见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。
- **反馈问题：** 到 [GitHub Issues](https://github.com/AstreoX/uBotSensor/issues) 提交，请附上 UE 版本、插件版本和复现步骤。

## 许可

Copyright 2026 AstreoX。本项目以 [Apache License 2.0](LICENSE) 发布，附 [NOTICE](NOTICE)，可以自由使用、修改和再分发，包括商业用途。再分发时需保留 `LICENSE` 与 `NOTICE` 文件，并注明你修改过的文件。
