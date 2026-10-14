# 架构与实现细节

本页收录文档站（<https://astreox.github.io/uBotDocs/>）没有覆盖的实现细节，摘自旧版 README，文字保持原样。用法、属性和各传感器的行为见文档站的 [uBotSensor 手册](https://astreox.github.io/uBotDocs/sensor/)。下面文字里的“下表”“下一节”指旧版 README 的“传感器一览”和“通用用法”，现在对应文档站的[传感器概述](https://astreox.github.io/uBotDocs/sensor/)和[传感器通用用法](https://astreox.github.io/uBotDocs/sensor/basics/)。

## 在 uBot 家族中的位置

uBotSensor 是 **Capability**（能力）层，只依赖 UE 引擎模块和 `UBotCore ^0.1.0`（`.uplugin` 的 `"Plugins"` 与 `"UBot"` 块都声明了这个依赖）。上层的 `UBotROS` 适配器包依赖它，并负责把这些传感器发布为 ROS 2 话题。

| 模块 | 类型 | 加载阶段 | 说明 |
| --- | --- | --- | --- |
| `UBotSensor` | Runtime | Default | 传感器组件、帧对象、感知介质、数学辅助 |
| `UBotSensorEditor` | Editor | PostEngineInit | 2D / 3D 激光雷达和相机在视口里的 Component Visualizer |

`"UBot"` 块声明的能力键（`Provides`）：`Sensor.Pose`、`Sensor.Odometry`、`Sensor.IMU`、`Sensor.Lidar2D`、`Sensor.Lidar3D`、`Sensor.RGBCamera`、`Sensor.DepthCamera`、`Sensor.Medium`。整个插件只有一个日志类别 `LogUBotSensor`。

## 传感器组件与辅助类

所有传感器都继承 `UUBotSensorComponent`（`UStaticMeshComponent` 的子类）：组件本身就是传感器坐标系，并显示各自的传感器模型。它们都实现 Core 的 `IUBotSensor` 接口，`GetSensorType` 返回下表的类型键。

所有传感器还有通用事件 `OnSensorStarted`、`OnSensorStopped`、`OnSensorSampled`、`OnSensorError`（见下一节）。另外有两个不是传感器的辅助类：`UUBotCameraCaptureCoordinatorComponent`（显示名 uBot Camera Capture Coordinator，相机同步分组）和 `UUBotCameraCaptureSubsystem`（按世界保存分组注册表）。

每个传感器使用 `Content/Models/SM_UBotSensor_<Kind>` 网格作为模型，`<Kind>` 为 `Pose`、`Odometry`、`IMU`、`Lidar2D`、`Lidar3D`、`RGBCamera`、`DepthCamera`。`bHideSensorModelInGame`（默认开启）让模型在游戏中隐藏。

## 仿真时间戳与采样调度

时间戳取自 `UBot::Clock::GetSimTimeSeconds`，基准是 `UWorld::GetTimeSeconds()`（游戏时间，遵循暂停和时间膨胀）。同步分组中的相机帧使用分组的时间戳。

采样由组件 tick 驱动，不再使用循环定时器：

- 构造函数设置 `bCanEverTick = true`、`bStartWithTickEnabled = false`、`TickGroup = TG_PostPhysics`、`bTickEvenWhenPaused = false`。tick 只在传感器运行时开启，暂停时不 tick，所以暂停期间不采样。
- `StartSensor` 取立即样本后调用 `Scheduler.Reset(UpdateRateHz, Now + 1/UpdateRateHz)` 并开启 tick；`TickComponent` 里 `Scheduler.ConsumeDue(SimTime)` 到期才采样。
- **每个 world tick 最多一个样本。** 一帧跨过多个采样间隔时，多出来的间隔不会补采，而是计入 `GetDroppedSampleCount()`（`StartSensor` 时清零）。因此时间戳严格递增，并等于该帧的仿真时间。
- 运行中调用 `SetUpdateRateHz` 只改变间隔并保持相位：不重启、不取立即样本，也不重置 IMU 等的历史。
- 子类不能把 `PrimaryComponentTick.bCanEverTick` 清掉。

## 相机的渲染与读回

- 每次采样把场景渲染到 render target（`USceneCaptureComponent2D`），发布引用它的 `UUBotImageFrame`。**像素不会自动读回 CPU**：需要时调用 `UUBotImageFrame::RequestPixelReadback`（同步、开销高）。该帧必须仍是最新一次捕获，否则返回错误（捕获代数检查）。

## 感知介质（烟雾）

感知介质是 Core 的 `FUBotMediumSnapshot`：若干球形单元（`FUBotMediumCell`，位置和半径单位厘米，`Density` 与 `EdgeFactor` 为 0 到 1）。`UBotSensorMedium.h` 里的 `UBot::SensorMedium` 是 Core `UBot::Medium` 之上的薄层，激光雷达和相机通过 `CollectMedia` 取得介质，交点计算走 `UBot::Medium::FindFirstIntersection`。介质从两条路径发现，最后都是快照：

1. **接口路径**：实现了 `IUBotPerceptionMedium` 的 Actor（C++ 或蓝图），由 `UBot::Medium::CollectSnapshots` 收集。

   ```cpp
   UCLASS()
   class AMyFogVolume : public AActor, public IUBotPerceptionMedium
   {
       GENERATED_BODY()

   public:
       virtual bool GetMediumSnapshot_Implementation(FUBotMediumSnapshot& OutSnapshot) const override
       {
           OutSnapshot.Tint = FLinearColor(0.7f, 0.7f, 0.75f, 1.0f);
           FUBotMediumCell& Cell = OutSnapshot.Cells.AddDefaulted_GetRef();
           Cell.WorldPositionCentimeters = GetActorLocation();
           Cell.RadiusCentimeters = 300.0f;
           Cell.Density = 0.8f;
           return true;
       }
   };
   ```

2. **旧版标签路径**：带有 Actor 标签 `AgentSensor.SmokeVolume`（`UBot::SensorMedium::LegacySmokeVolumeTag()`）的 Actor，通过反射读取 `IsSmokeFieldActive()`（可选，默认激活）、`GetSmokeCellSnapshots()`（返回含 `WorldPosition`、`Density`、可选 `EdgeFactor` 的结构数组）以及半径属性 `CellVisualRadiusMeters` 或 `CellSizeMeters`（都没有时半径取 150 cm）。**这个标签是有意保留的旧名称**，AgentCombatCore 的烟雾 Actor 仍然使用它，不要改成 `UBot.*`。

同时实现了接口的 Actor 不会被收集两次。两条路径都会按 `SmokeCellRadiusScale` 缩放单元半径，并丢弃有效密度（`Density * EdgeFactor`）低于 `SmokeDensityThreshold` 的单元。

## CoreRedirects 的维护

重定向列表**只有一份**：`Config/DefaultUBotSensor.ini` 的 `[CoreRedirects]` 段，共 47 条。

| 类型 | 条数 | 内容 |
| --- | --- | --- |
| `PackageRedirects`（脚本包） | 1 | `/Script/AgentSensorCore` 到 `/Script/UBotSensor`，兜住没有单独重定向的内容（委托引脚的成员父级、重定向类的 `Default__` 对象等） |

- 改动重定向时只编辑这个 ini；测试 `UBotSensor.Redirects.Resolve` 会读取它并逐条检查。
