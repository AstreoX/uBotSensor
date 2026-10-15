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

uBotSensor provides simulated sensors for Actors: pose, odometry, IMU, 2D and 3D lidar, RGB and depth cameras. Each sensor is a component added in the editor and read from Blueprint or C++. It builds on [uBotCore](https://github.com/AstreoX/uBotCore), which supplies the simulation clock and the frame header carried by every sample.

- **Blueprint-first.** Every sensor appears under `uBot` in the component list; its properties, functions and events are exposed to Blueprint.
- **No ROS dependency.** Output consists of native UE structs and objects: positions, ranges and velocities in meters and seconds, angles in radians. [uBotROS](https://github.com/AstreoX/uBotROS) publishes it to ROS 2.
- **Sampling on simulation time.** Scheduled sampling follows the simulation clock with at most one sample per world tick, so timestamps strictly increase. Noise uses a fixed seed.
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
| Perception medium (smoke) | Stops lidar rays, tints RGB images, sets depth to the entry distance | — |

## Install

**Recommended: uBot Manager.** Download the installer from the [uBotManager releases](https://github.com/AstreoX/uBotManager/releases), open the project's `.uproject`, select `UBotSensor` in the **Packages** view, click **Install**, then **Apply…**. uBotCore is installed as a dependency; the plugin goes into `Plugins/uBot/`.

**Manual.** Clone both repositories into the project, enable **uBot Core** and **uBot Sensor** in `Edit > Plugins` (category uBot) and restart the editor:

```powershell
mkdir <Project>/Plugins/uBot -Force; cd <Project>/Plugins/uBot
git clone --branch v0.1.2 https://github.com/AstreoX/uBotCore.git
git clone --branch v0.1.1 https://github.com/AstreoX/uBotSensor.git
```

**Requirements:** Unreal Engine 5.5, uBotCore ^0.1.0, Visual Studio 2022 with the C++ toolchain for source builds, Windows 10/11 for uBot Manager.

## Documentation

The documentation site and the files under `docs/` are in Simplified Chinese.

| I want to ... | Go to |
| --- | --- |
| Build the first sensor step by step | [Five-minute quick start](https://astreox.github.io/uBotDocs/start/quickstart/) |
| Look up every sensor and its output | [uBotSensor overview](https://astreox.github.io/uBotDocs/sensor/) |
| Start, stop and read a sensor | [Common usage](https://astreox.github.io/uBotDocs/sensor/basics/) |
| Choose sample rates | [Performance and sample rate](https://astreox.github.io/uBotDocs/sensor/performance/) |
| Add smoke that affects lidar and cameras | [Perception medium](https://astreox.github.io/uBotDocs/sensor/medium/) |
| Publish sensor data to ROS 2 | [Publish sensor data](https://astreox.github.io/uBotDocs/ros/publish-sensors/) |
| Use the sensors from C++ | [Sensors and interfaces in C++](https://astreox.github.io/uBotDocs/develop/cpp/) |
| Migrate from AgentSensorCore | [Migrating from old plugins](https://astreox.github.io/uBotDocs/develop/migration/) |
| Look up modules and scheduling internals | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| Run the automation tests | [docs/TESTING.md](docs/TESTING.md) |

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

- **Build:** place the repository in `<Project>/Plugins/uBot/uBotSensor` with uBotCore enabled; the editor offers to compile `UBotSensor` and `UBotSensorEditor` on first open.
- **Test:** the tests are Unreal automation tests named `UBotSensor.*`, run from Session Frontend or the command line; see [docs/TESTING.md](docs/TESTING.md).
- **Internals:** modules, capability keys and scheduling details are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).
- **Issues:** report bugs on [GitHub Issues](https://github.com/AstreoX/uBotSensor/issues) and include the UE version, the plugin version and steps to reproduce.

## License

Copyright 2026 AstreoX. Licensed under the [Apache License 2.0](LICENSE) (see [NOTICE](NOTICE)); use, modification and redistribution, including commercial use, are permitted. Redistributions must keep `LICENSE` and `NOTICE` and mark modified files.
