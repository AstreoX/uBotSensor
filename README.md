<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
    <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
    <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="88">
  </picture>
</p>
<h1 align="center">uBotSensor</h1>
<p align="center"><b>Simulated sensors for Unreal Engine 5.5: pose, IMU, odometry, lidar and cameras.</b></p>
<p align="center">
  <a href="https://github.com/AstreoX/uBotSensor/releases"><img alt="release" src="https://img.shields.io/github/v/release/AstreoX/uBotSensor?label=release"></a>
  <a href="LICENSE"><img alt="license" src="https://img.shields.io/github/license/AstreoX/uBotSensor"></a>
  <img alt="Unreal Engine 5.5" src="https://img.shields.io/badge/Unreal%20Engine-5.5-313131">
</p>
<p align="center">
  <a href="https://astreox.github.io/uBotDocs/sensor/">Documentation</a> ·
  <a href="https://github.com/AstreoX/uBotSensor/releases">Releases</a> ·
  <a href="https://github.com/users/AstreoX/projects/2">Roadmap</a> ·
  <a href="https://github.com/AstreoX/uBotSensor/issues">Issues</a>
</p>

**English** | [简体中文](README.zh-CN.md)

## What it is

uBotSensor gives an Actor simulated sensors: pose, odometry, IMU, 2D and 3D lidar, RGB and depth cameras. Each one is a component you add in the editor and read from Blueprint or C++. It builds on [uBotCore](https://github.com/AstreoX/uBotCore), which supplies the simulation clock and the frame header every sample carries, and it is in beta.

- **Blueprint-first.** Every sensor sits under `uBot` in the component list, with its properties, functions and events exposed to Blueprint.
- **No ROS dependency.** Output is plain UE structs and objects, with positions, ranges and velocities in meters and seconds and angles in radians; publishing to ROS 2 is left to [uBotROS](https://github.com/AstreoX/uBotROS).
- **Sampling on simulation time.** Scheduled sampling follows the simulation clock, at most one sample per world tick, so timestamps strictly increase; noise is seeded.
- **Self-describing frames.** Each sample carries a header with sensor id, frame id, sequence number, timestamp and status.
- **Synchronized cameras.** A capture coordinator triggers several cameras at the same simulation instant.

| Component | Output | Default rate |
| --- | --- | --- |
| `uBot Pose Sensor` | Position (m), orientation quaternion, optional velocity | 30 Hz |
| `uBot Odometry Sensor` | Pose relative to the start (m), velocity in the local frame, covariance | 30 Hz |
| `uBot IMU Sensor` | Orientation, angular velocity (rad/s), specific force (m/s²), variance | 100 Hz |
| `uBot 2D LiDAR Sensor` | Range, hit flag and intensity for each angle | 10 Hz |
| `uBot 3D LiDAR Sensor` | Point cloud: hit points (m), ring, intensity | 10 Hz |
| `uBot RGB Camera Sensor` | `BGRA8` image; pixels are read back to the CPU on request | 30 Hz |
| `uBot Depth Camera Sensor` | `Depth32FC1` depth in meters, 0 means invalid | 30 Hz |
| Perception medium (smoke) | Not a sensor: stops lidar rays, tints RGB, sets depth to the entry distance | n/a |

## Install

**Recommended: uBot Manager.** Download the installer from the [uBotManager releases](https://github.com/AstreoX/uBotManager/releases), open your project's `.uproject`, select `UBotSensor` in the **Packages** view, click **Install**, then **Apply…**. uBotCore is installed with it, and the plugin lands in `Plugins/uBot/`.

**Manual.** Clone both repositories into the project, enable **uBot Core** and **uBot Sensor** in `Edit > Plugins` (category uBot), and restart the editor:

```powershell
mkdir <Project>/Plugins/uBot -Force; cd <Project>/Plugins/uBot
git clone --branch v0.1.2 https://github.com/AstreoX/uBotCore.git
git clone --branch v0.1.1 https://github.com/AstreoX/uBotSensor.git
```

**Requirements:** Unreal Engine 5.5 and uBotCore ^0.1.0. Source installs compile C++, so they need Visual Studio 2022 with the C++ toolchain; uBot Manager runs on Windows 10/11 and prefers prebuilt packages when it can.

## Quick start

Add an IMU to an Actor and watch it sample:

1. In an Actor Blueprint, open the **Components** panel, click **Add** and choose `uBot IMU Sensor`.
2. Select the component and click **+** next to `On Sensor Sampled` under **Events** in **Details**.
3. Connect `Header` through a `Break` node and wire `Sequence` into `Print String`.
4. Drop the Actor in a level and press Play. The counter climbs on screen (`Auto Start` is on; the IMU samples at 100 Hz, at most once per frame).

## Documentation

The documentation site is written in Simplified Chinese.

| I want to ... | Go to |
| --- | --- |
| Follow the full first-sensor guide | [Five-minute quick start](https://astreox.github.io/uBotDocs/start/quickstart/) |
| See every sensor and what it outputs | [uBotSensor overview](https://astreox.github.io/uBotDocs/sensor/) |
| Start, stop and read any sensor | [Common usage](https://astreox.github.io/uBotDocs/sensor/basics/) |
| Choose sample rates | [Performance and sample rate](https://astreox.github.io/uBotDocs/sensor/performance/) |
| Add smoke that disturbs lidar and cameras | [Perception medium](https://astreox.github.io/uBotDocs/sensor/medium/) |
| Publish sensor data to ROS 2 | [Publish sensor data](https://astreox.github.io/uBotDocs/ros/publish-sensors/) |
| Use the sensors from C++ | [Sensors and interfaces in C++](https://astreox.github.io/uBotDocs/develop/cpp/) |
| Migrate from AgentSensorCore | [Migrating from old plugins](https://astreox.github.io/uBotDocs/develop/migration/) |
| Read about modules and scheduling internals | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) (Chinese) |
| Run the automation tests | [docs/TESTING.md](docs/TESTING.md) (Chinese) |

## uBot family

| Repository | What it is |
| --- | --- |
| [uBotCore](https://github.com/AstreoX/uBotCore) | Simulation clock, frame header, shared interfaces, package management |
| **[uBotSensor](https://github.com/AstreoX/uBotSensor)** | **Pose, IMU, odometry, 2D/3D lidar, RGB and depth cameras** |
| [uBotROS](https://github.com/AstreoX/uBotROS) | ROS 2 over a TCP bridge: sensors, TF, /clock, cmd_vel |
| [uBotAerial](https://github.com/AstreoX/uBotAerial) | Multirotor drones: simplified flight, rigid-body dynamics, wind |
| [uBotManager](https://github.com/AstreoX/uBotManager) | Desktop app and CLI to install, update and build uBot packages |
| [uBotDocs](https://github.com/AstreoX/uBotDocs) | Documentation site: https://astreox.github.io/uBotDocs/ |

## Development

- **Build:** place the repository in `<Project>/Plugins/uBot/uBotSensor` with uBotCore enabled; on first open the editor offers to compile the `UBotSensor` and `UBotSensorEditor` modules.
- **Test:** the tests are Unreal automation tests named `UBotSensor.*`, run from Session Frontend or the command line; see [docs/TESTING.md](docs/TESTING.md).
- **Internals:** modules, capability keys and scheduling details are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).
- **Issues:** report bugs on [GitHub Issues](https://github.com/AstreoX/uBotSensor/issues) with your UE version, the plugin version and steps to reproduce.

## License

Copyright 2026 AstreoX. Licensed under the [Apache License 2.0](LICENSE); see [NOTICE](NOTICE). You may use, modify and redistribute it, including commercially. Redistributions must keep `LICENSE` and `NOTICE` and state which files you changed.
