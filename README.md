# uBot Sensor

## 简介

uBotSensor 是 uBot 家族的传感器插件，为 UE 5.5 提供一组仿真传感器组件：位姿（Pose）、里程计（Odometry）、IMU、2D / 3D 激光雷达、RGB 相机、深度相机，以及烟雾等感知介质对雷达和相机的干扰。它由旧版 `AgentSensorCore`（0.5.0）重命名迁移而来，建立在 [uBotCore](https://github.com/AstreoX/uBotCore) 之上。

- **蓝图优先**：每种传感器都是可以直接在 Add Component 菜单里添加的组件，属性、函数和事件都暴露给蓝图。
- **没有 ROS 依赖**：插件不含 ROS 头文件或 rclcpp，输出是 UE 原生结构体和 UObject 帧（米、秒、弧度）。把数据发布到 ROS 是上层适配器的事。
- **采样确定**：采样由组件 tick 加 Core 的 `FUBotSampleScheduler` 驱动，样本用仿真时间打时间戳。

## 在 uBot 家族中的位置

uBotSensor 是 **Capability**（能力）层，只依赖 UE 引擎模块和 `UBotCore ^0.1.0`（`.uplugin` 的 `"Plugins"` 与 `"UBot"` 块都声明了这个依赖）。上层的 `UBotROS` 适配器包依赖它，并负责把这些传感器发布为 ROS 2 话题。

| 模块 | 类型 | 加载阶段 | 说明 |
| --- | --- | --- | --- |
| `UBotSensor` | Runtime | Default | 传感器组件、帧对象、感知介质、数学辅助 |
| `UBotSensorEditor` | Editor | PostEngineInit | 2D / 3D 激光雷达和相机在视口里的 Component Visualizer |

`"UBot"` 块声明的能力键（`Provides`）：`Sensor.Pose`、`Sensor.Odometry`、`Sensor.IMU`、`Sensor.Lidar2D`、`Sensor.Lidar3D`、`Sensor.RGBCamera`、`Sensor.DepthCamera`、`Sensor.Medium`。整个插件只有一个日志类别 `LogUBotSensor`。

## 安装

面向 UE 5.5。插件以源码形式分发，工程需要能编译 C++。先安装 uBotCore。

1. 把仓库克隆到 uBotCore 旁边，即 `<Project>/Plugins/uBot/uBotSensor`：

   ```powershell
   git clone https://github.com/AstreoX/uBotSensor.git <Project>/Plugins/uBot/uBotSensor
   ```

   也可以用 uBotCore 的 **uBot Packages** 窗口（`Window` 菜单）安装，它会先装好缺失的依赖；命令行等价于 `UnrealEditor-Cmd.exe <Project>.uproject -run=UBotPackage -Install=UBotSensor`。

2. 启用插件：在 `Edit > Plugins` 中搜索 uBot Sensor（分类 uBot）并勾选，或在 `.uproject` 中加入

   ```json
   "Plugins": [
       { "Name": "UBotCore", "Enabled": true },
       { "Name": "UBotSensor", "Enabled": true }
   ]
   ```

3. 重启编辑器。传感器组件在 Add Component 菜单的 **uBot** 分类下，显示名形如 `uBot IMU Sensor`。

C++ 工程使用时，在自己模块的 `Build.cs` 中加入 `"UBotCore"` 和 `"UBotSensor"`。

## 传感器一览

所有传感器都继承 `UUBotSensorComponent`（`UStaticMeshComponent` 的子类）：组件本身就是传感器坐标系，并显示各自的传感器模型。它们都实现 Core 的 `IUBotSensor` 接口，`GetSensorType` 返回下表的类型键。

| 显示名（类） | 类型键 | 主要输出 | 事件 | 默认频率 |
| --- | --- | --- | --- | --- |
| uBot Pose Sensor<br>`UUBotPoseSensorComponent` | `Pose` | `GetLatestPoseData()` 返回 `FUBotPoseSensorData`：位置（米）、四元数、可选速度 | `OnPoseDataUpdated` | 30 Hz |
| uBot Odometry Sensor<br>`UUBotOdometrySensorComponent` | `Odometry` | `GetLatestOdometryData()` 返回 `FUBotOdometrySensorData`：相对起点的位姿、机体系速度、协方差 | `OnOdometryDataUpdated` | 30 Hz |
| uBot IMU Sensor<br>`UUBotImuSensorComponent` | `IMU` | `GetLatestIMUData()` 返回 `FUBotImuSensorData`：朝向、角速度、线加速度、方差 | `OnImuDataUpdated` | 100 Hz |
| uBot 2D LiDAR Sensor<br>`UUBot2DLidarSensorComponent` | `Lidar2D` | `GetLatest2DLidarFrame()` 返回 `UUBot2DLidarFrame`：距离、命中标志、强度 | `On2DLidarFrameReady` | 10 Hz |
| uBot 3D LiDAR Sensor<br>`UUBot3DLidarSensorComponent` | `Lidar3D` | `GetLatest3DLidarFrame()` 返回 `UUBotPointCloudFrame`：点、环号、强度 | `On3DLidarFrameReady` | 10 Hz |
| uBot RGB Camera Sensor<br>`UUBotRgbCameraSensorComponent` | `RGBCamera` | `GetLatestImageFrame()` 返回 `UUBotImageFrame`（`BGRA8`） | `OnImageFrameReady` | 30 Hz |
| uBot Depth Camera Sensor<br>`UUBotDepthCameraSensorComponent` | `DepthCamera` | `GetLatestImageFrame()` 返回 `UUBotImageFrame`（`Depth32FC1`，米） | `OnImageFrameReady` | 30 Hz |

所有传感器还有通用事件 `OnSensorStarted`、`OnSensorStopped`、`OnSensorSampled`、`OnSensorError`（见下一节）。另外有两个不是传感器的辅助类：`UUBotCameraCaptureCoordinatorComponent`（显示名 uBot Camera Capture Coordinator，相机同步分组）和 `UUBotCameraCaptureSubsystem`（按世界保存分组注册表）。

每个传感器使用 `Content/Models/SM_UBotSensor_<Kind>` 网格作为模型，`<Kind>` 为 `Pose`、`Odometry`、`IMU`、`Lidar2D`、`Lidar3D`、`RGBCamera`、`DepthCamera`。`bHideSensorModelInGame`（默认开启）让模型在游戏中隐藏。

## 通用用法

### 启动、停止与手动采样

- `bAutoStart`（默认开启）：`BeginPlay` 时自动 `StartSensor`，`EndPlay` 时 `StopSensor`。带 `CaptureGroupId` 的相机不自动启动（见相机一节）。
- `StartSensor()`：已在运行时返回 `true`；`bSensorEnabled` 为假、没有 World，或相机属于采集分组时返回 `false`。启动时先触发 `OnSensorStarted`，取一个立即样本，再按 `UpdateRateHz` 继续采样。
- `StopSensor()` 停止采样并触发 `OnSensorStopped`。
- `SampleSensor()` 随时可以手动取一个样本（各传感器另有 `SamplePose`、`SampleIMU`、`Sample2DLidar` 等便捷函数）。只有采到有效帧时才返回 `true`。`bSensorEnabled` 为假时得到状态为 `Disabled` 的帧头。
- `SetUpdateRateHz()` 把频率限制在 [0.1, 240] Hz。

### 事件

- `OnSensorSampled(Header)`：每次采样都会触发，包括无效样本，请检查 `Header.bValid` 和 `Header.Status`。
- `OnSensorError(ErrorMessage)`：采样出错时触发，例如 3D 激光雷达射线数超过上限、Pose 的参考组件无效。
- 数据事件：Pose、Odometry、IMU 的 `On...DataUpdated` 同样每次采样都触发（包括 `WarmingUp` 等无效样本）；激光雷达和相机的帧事件只在产生帧时触发。
- 蓝图中用 Bind / Assign 绑定事件。`IUBotSensor` 的函数（类别 `uBot|Sensor`）可以通过接口在任意传感器上调用。

### 帧头

每个样本都带 Core 的 `FUBotFrameHeader`：

| 字段 | 说明 |
| --- | --- |
| `SensorId` | 传感器 ID；属性为空时使用组件名 |
| `FrameId` | TF frame id；属性为空时使用传感器 ID |
| `Sequence` | `int64` 序号，每个组件独立，每个样本加一（含无效样本） |
| `TimestampSeconds` | 取样时的 uBot 仿真时间 |
| `bValid` | 本帧是否有效 |
| `Status` | `EUBotSampleStatus`：`Valid`、`WarmingUp`、`InvalidDeltaTime`、`TeleportDetected`、`Disabled`、`Error` |

`GetLatestFrameHeader()` 返回最新一帧的帧头，`IUBotSensor::GetLatestSensorHeader` 返回同一个值。

### 仿真时间戳与采样调度

时间戳取自 `UBot::Clock::GetSimTimeSeconds`，基准是 `UWorld::GetTimeSeconds()`（游戏时间，遵循暂停和时间膨胀）。同步分组中的相机帧使用分组的时间戳。

采样由组件 tick 驱动，不再使用循环定时器：

- 构造函数设置 `bCanEverTick = true`、`bStartWithTickEnabled = false`、`TickGroup = TG_PostPhysics`、`bTickEvenWhenPaused = false`。tick 只在传感器运行时开启，暂停时不 tick，所以暂停期间不采样。
- `StartSensor` 取立即样本后调用 `Scheduler.Reset(UpdateRateHz, Now + 1/UpdateRateHz)` 并开启 tick；`TickComponent` 里 `Scheduler.ConsumeDue(SimTime)` 到期才采样。
- **每个 world tick 最多一个样本。** 一帧跨过多个采样间隔时，多出来的间隔不会补采，而是计入 `GetDroppedSampleCount()`（`StartSensor` 时清零）。因此时间戳严格递增，并等于该帧的仿真时间。
- 运行中调用 `SetUpdateRateHz` 只改变间隔并保持相位：不重启、不取立即样本，也不重置 IMU 等的历史。
- 子类不能把 `PrimaryComponentTick.bCanEverTick` 清掉。

### 单位与坐标

位置、距离、速度使用米和秒，角度使用弧度；名字里带 `Degrees` 的属性除外。点云和图像帧里的 `SensorTransformWorld` 仍是 UE 世界变换（厘米）。坐标轴保持 UE 约定（X 前、Y 右、Z 上），数据不做 ROS 转换，转换由 Core 的 `UBot::Ros`（如 `YawUEToROS`）或上层适配器完成。

### 通用属性

| 属性 | 说明 |
| --- | --- |
| `SensorId`、`FrameId` | 帧头里的 ID，见上表；各传感器有自己的默认值 |
| `UpdateRateHz` | 采样频率，限制在 [0.1, 240] |
| `bAutoStart`、`bSensorEnabled` | 自动启动；总开关 |
| `bNoiseEnabled`、`RandomSeed` | 噪声开关；每次 `StartSensor` 用它重置随机流 |
| `bDebugVisualization` | 仅激光雷达使用：运行时绘制调试射线 |
| `bHideSensorModelInGame` | 游戏中隐藏传感器模型 |

这些属性在细节面板的 `uBot Sensor` 类别（及其 `Debug`、`Model`、`Noise` 子类别）下。蓝图函数的类别是 `uBot Sensor` 或 `uBot Sensor|<传感器>`，例如 `uBot Sensor|IMU`。

### 在 C++ 中枚举传感器

```cpp
#include "UBotSensorInterface.h"

TArray<UObject*> Sensors;
UBot::Sensors::FindSensors(Robot, Sensors);
for (UObject* Sensor : Sensors)
{
    const FName Type = IUBotSensor::Execute_GetSensorType(Sensor);   // 例如 "IMU"
    const FString Id = IUBotSensor::Execute_GetSensorId(Sensor);
}
```

## 每种传感器的要点

**Pose**（默认 ID `pose_sensor`）

- 输出传感器组件自身的位姿。`ReferenceFrame` 为 `World`、`Parent`（所附着的父组件）或 `CustomComponent`（`CustomReferenceComponent`）。
- `ReferenceFrameId` 默认 `world`；为空时依次回退为 `world`、父组件名、参考组件名。
- `bIncludeVelocity` 开启时用相邻样本差分得到线速度和角速度；首个样本没有速度（`bHasVelocity` 为假）。父组件或参考组件无效时样本状态为 `Error`。

**Odometry**（默认 ID `odometry_sensor`，`FrameId` 为 `odom`）

- 跟踪 `CustomTrackedComponent`，为空则跟踪所属 Actor 的根组件。输出相对起始原点的位姿（米）和机体系下的线速度、角速度。`OdometryFrameId`（`odom`）和 `ChildFrameId`（`base_link`）写入输出数据。
- 噪声：位置、朝向、线速度、角速度噪声，位置和朝向的随机游走漂移。协方差 `PoseCovariance`、`TwistCovariance` 是 6x6 对角阵（36 个 `double`），由噪声参数填充。
- 首个样本为 `WarmingUp`。相邻样本间隔不在 [`MinimumValidDeltaSeconds`, `MaximumValidDeltaSeconds`] 内给出 `InvalidDeltaTime`，位移或旋转超过 `TeleportDistanceMeters` / `TeleportAngleDegrees` 给出 `TeleportDetected`；这两种情况下原点会重设为当前位置。`ResetOdometry()` 手动重设原点。

**IMU**（默认 ID `imu_sensor`，默认 100 Hz）

- 由位姿差分得到机体系下的角速度和线加速度。线加速度是比力：先减去世界重力再转到机体系，静止时读数约为 +9.81 m/s²，方向是世界向上，在传感器坐标系下表示。
- 加速度计 / 陀螺仪噪声、偏置和偏置随机游走（需要 `bNoiseEnabled`），可选低通滤波（`LowPassCutoffHz`），可选输出限幅（`MaxLinearAcceleration`、`MaxAngularVelocity`）。
- 前两个样本只填充历史，状态为 `WarmingUp`，第三个样本起才是 `Valid`。间隔超过 `MaxSampleDeltaSeconds`、瞬移（`TeleportDistanceMeters`、`TeleportAngleDegrees`）会重置历史。`ResetIMUBias()` 和 `ResetIMUHistory()` 可从蓝图调用。

**2D LiDAR**（默认 ID `lidar_2d`）

- 在传感器本地 XY 平面扫描：`StartAngleDegrees` 到 `EndAngleDegrees`（默认 -135° 到 135°），`HorizontalSamples`（默认 541）。角度向右（+Y）为正，与 UE 偏航一致。
- 射线检测参数：`TraceChannel`、`bTraceComplex`、`bIgnoreOwner`、`IgnoredActors`。量程 `MinimumRangeMeters` 到 `MaximumRangeMeters`（默认 0.05 到 30 m），`RangeNoiseStdDevMeters` 是距离噪声。
- `UUBot2DLidarFrame`：角度范围与步长（弧度）、量程、`ScanPeriodSeconds`、逐样本距离（`GetRangeByIndex`、`CopyAllRanges`）、命中标志（`CopyAllHitFlags`）和强度（`GetIntensities`）。未命中的样本距离等于最大量程。

**3D LiDAR**（默认 ID `lidar_3d`）

- 水平视场 `HorizontalFovDegrees`（默认 360°）、垂直视场 `VerticalFovUpDegrees` / `VerticalFovDownDegrees`（各 15°）、`NumberOfChannels`（16）、`HorizontalSamples`（360）。`MaximumRaysPerScan`（默认 65536）限制单次扫描的射线数，`NumberOfChannels * HorizontalSamples` 超过它时该次扫描报错，不会截断。
- `UUBotPointCloudFrame` 只包含有效命中的点：传感器坐标系下的点（米）、环号、时间偏移（扫描是瞬时的，目前恒为 0）。`GetPointByIndex` 一次取出一个点；`CopyAll...` 函数复制整个数组，开销高。`HitPointCount` 是点数，`TotalRayCount` 是发射的射线数。

**强度**：两种雷达的帧都有 `GetIntensities()`，范围 0 到 255。命中表面时为 `255 * Clamp(Dot(-RayDirectionWorld, ImpactNormal), 0, 1)`；2D 帧里未命中的样本以及被介质截断的样本为 0。3D 点云里强度数组与点数组一一对应。

**RGB 与深度相机**（默认 ID `rgb_camera`、`depth_camera`）

- 公共基类 `UUBotCameraSensorComponent`：`ResolutionWidth` / `ResolutionHeight`（640x480）、`HorizontalFovDegrees`（90°）、`PreviewDistanceMeters`（10 m，视口视锥长度，也是 RGB 烟雾叠加的最大距离），可选 `ExternalRenderTarget`（格式必须与编码匹配）。
- 每次采样把场景渲染到 render target（`USceneCaptureComponent2D`），发布引用它的 `UUBotImageFrame`。**像素不会自动读回 CPU**：需要时调用 `UUBotImageFrame::RequestPixelReadback`（同步、开销高）。该帧必须仍是最新一次捕获，否则返回错误（捕获代数检查）。
- 帧里带内参（`Fx`、`Fy`、`Cx`、`Cy`）、视场、`SensorTransformWorld`、`CaptureGroupId`、`CaptureSequence`、`CaptureTimestampSeconds`、`bSynchronized`。`GetBGRAPixelByCoordinates`、`GetDepthMetersByCoordinates` 读取读回后的像素。
- 深度相机：`Depth32FC1`，单位米，0 表示无效；`MinimumDepthMeters` / `MaximumDepthMeters`（0.1 到 100 m）之外的深度记为 0。`UUBotImageFrame::UpdateDepthPreview` 生成灰度或伪彩色预览纹理，`GeneratePointCloud` 从已读回的帧生成相机坐标系下的点云（没有强度，`HasIntensityData()` 为假）。

**相机同步分组**

- `CaptureGroupId` 非空的相机不会自己采样（`StartSensor` 返回 `false`），而由同组的 `UUBotCameraCaptureCoordinatorComponent` 统一触发。一个分组 ID 只能有一个协调器；没有协调器时，相机会在下一个 tick 通过 `OnSensorError` 报告。
- 协调器同样用组件 tick 和 `FUBotSampleScheduler`（`UpdateRateHz`，默认 30 Hz）。`StartCaptureGroup` 后**第一次自动捕获在一个间隔之后**，不会立即捕获；`bAutoStart` 开启时 `BeginPlay` 自动启动。
- 每个 world tick 最多捕获一组。同组相机按路径名排序依次捕获，共用同一个组序号（从 1 递增）和同一个组时间戳。被跳过的间隔计入 `GetDroppedCaptureCount()`（`StartCaptureGroup` 清零）。
- 其他函数：`StopCaptureGroup`、`CaptureGroupOnce`（手动触发一次）、`GetRegisteredCameraCount`、`IsCaptureGroupRunning`；事件 `OnCaptureGroupError`。

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

传感器上的属性（名称和类别与旧版相同）：`bEnableSmokeInterference`、`SmokeVolumeActorTag`（默认就是旧标签；设为 `None` 只关闭旧标签路径，不影响接口路径）、`SmokeDensityThreshold`（0.05）、`SmokeCellRadiusScale`；相机另有 `SmokeInterferenceTint`、`SmokeInterferenceOpacityScale`、`MaximumSmokeInterferenceOpacity`。

- **激光雷达**：射线进入有效密度不低于阈值的单元时，在入口处被截断，按命中处理，该样本强度为 0。
- **RGB 相机**：像素射线遇到介质时，像素向介质颜色混合，不透明度为 `Clamp(Density * OpacityScale, 0, MaximumOpacity)`，保留原 alpha。颜色用相机上的 `SmokeInterferenceTint`（旧版行为）；只有来自 Core 接口的快照才使用快照自己的 `Tint`。
- **深度相机**：把深度替换为介质入口距离；入口距离超出 [`MinimumDepthMeters`, `MaximumDepthMeters`] 时记为 0（无效）。
- 相机的烟雾叠加发生在像素读回时（`RequestPixelReadback`），不影响 render target 本身。

## 从 AgentSensorCore 迁移

UFUNCTION 与 UPROPERTY 的**名称保持不变**，所以蓝图引脚在类重定向之后继续有效。类、结构、枚举、动态委托签名和网格都有 CoreRedirect。

### 重命名

- 类 `UAgentXxx` 改为 `UUBotXxx`（如 `UAgentImuSensorComponent` 改为 `UUBotImuSensorComponent`，`UAgentImageFrame` 改为 `UUBotImageFrame`，`UAgentCameraCaptureSubsystem` 改为 `UUBotCameraCaptureSubsystem`）；文件 `AgentXxx.h/.cpp` 改为 `UBotXxx.h/.cpp`。
- 数据结构 `FAgentPoseSensorData`、`FAgentImuSensorData`、`FAgentOdometrySensorData` 改为 `FUBot...`；枚举 `EAgentPoseReferenceFrame`、`EAgentImageEncoding`、`EAgentDepthPreviewMode` 改为 `EUBot...`；动态委托 `FAgent...Event` 改为 `FUBot...Event`。
- **移入 Core**：`FAgentSensorFrameHeader` 变为 `FUBotFrameHeader`，`EAgentSensorSampleStatus` 变为 `EUBotSampleStatus`（字段和枚举值不变，头文件 `UBotTypes.h`）。
- 命名空间：`AgentSensorMath` 变为 `UBot::SensorMath`（其中 `WorldCentimetersToMeters` 已删除，改用 `UBot::Units::CentimetersToMeters`）；`AgentSensorSmoke` 变为 `UBot::SensorMedium`；编辑器里的 `AgentLidarVisualizer` 变为 `UBot::LidarVisualizer`。
- 模块与导出宏：`AgentSensorCore` / `AgentSensorCoreEditor` 变为 `UBotSensor` / `UBotSensorEditor`，`AGENTSENSORCORE_API` 变为 `UBOTSENSOR_API`；`LogAgent*` 日志类别合并为 `LogUBotSensor`。
- 网格：`SM_AgentSensor_<Kind>` 变为 `SM_UBotSensor_<Kind>`，路径从 `/AgentSensorCore/Models/` 变为 `/UBotSensor/Models/`。
- 蓝图中的显示信息：`ClassGroup` 为 `uBot`；组件显示名由 `Agent IMU Sensor` 变为 `uBot IMU Sensor`（其余同理）；`Agent Sensor` 类别变为 `uBot Sensor`，函数类别加前缀 `uBot Sensor|`；插件分类由 Simulation 变为 uBot。

### CoreRedirects

重定向列表**只有一份**：`Config/DefaultUBotSensor.ini` 的 `[CoreRedirects]` 段，共 47 条。

| 类型 | 条数 | 内容 |
| --- | --- | --- |
| `PackageRedirects`（脚本包） | 1 | `/Script/AgentSensorCore` 到 `/Script/UBotSensor`，兜住没有单独重定向的内容（委托引脚的成员父级、重定向类的 `Default__` 对象等） |
| `ClassRedirects` | 14 | 9 个组件、协调器、子系统和 3 个帧对象 |
| `StructRedirects` | 4 | 其中 `AgentSensorFrameHeader` 指向 `/Script/UBotCore.UBotFrameHeader` |
| `EnumRedirects` | 4 | 其中 `EAgentSensorSampleStatus` 指向 `/Script/UBotCore.EUBotSampleStatus` |
| `ObjectRedirects`（动态委托签名） | 10 | `*__DelegateSignature`，让 Bind / Assign / Create Event 节点里序列化的签名引用能解析 |
| `PackageRedirects` 加 `ObjectRedirects`（网格） | 7 加 7 | `SM_AgentSensor_<Kind>` 到 `SM_UBotSensor_<Kind>` |

注意：

- 引擎在配置已启用的插件时加载这份 ini，所以重定向**只在编辑器启动时生效**。在运行中的编辑器里新启用插件，旧资源不会被重定向，请重启编辑器。
- 重定向只处理序列化数据。工程里引用 `UAgent*` 等旧名称的 C++ 代码要按上面的映射手工改名。
- 旧资源第一次打开后请重新保存，让它们用新名称序列化。迁移完成后不再需要旧插件，建议从工程中移除 `AgentSensorCore`。
- 改动重定向时只编辑这个 ini；测试 `UBotSensor.Redirects.Resolve` 会读取它并逐条检查。

### 行为差异

除了上面的重命名，行为与旧版一致，有意的差异只有：

1. **采样改为组件 tick 加 `FUBotSampleScheduler`**（旧版是循环 `FTimerManager` 定时器）：一帧耗时超过采样间隔时，旧定时器会在同一帧触发多次，同一个仿真时间被采样多次，IMU 和里程计因此报告 `InvalidDeltaTime`。现在每个 world tick 最多一个样本，不再出现同帧重复样本。新增 `GetDroppedSampleCount()` 和 `GetDroppedCaptureCount()`。
2. **时间戳取自 uBot 时钟** `UBot::Clock::GetSimTimeSeconds`。时间基准与旧版相同（`UWorld::GetTimeSeconds()`）。
3. **新增 `Intensities`**：`UUBot2DLidarFrame` 和 `UUBotPointCloudFrame` 各有 `GetIntensities()`，范围 0 到 255。
4. 所有传感器实现 `IUBotSensor`，可以用 `UBot::Sensors::FindSensors` 统一枚举；C++ 里新增虚函数 `GetSensorTypeName()`。
5. 烟雾代码基于 Core 的 `UBot::Medium`，并新增 `IUBotPerceptionMedium` 发现路径；旧标签路径的行为不变。

## 测试

测试名都以 `UBotSensor.` 开头，共 29 项，分类为 `Camera`、`Depth`、`Interface`、`Lidar`、`Lidar2D`、`Lidar3D`、`Math`、`Medium`、`Odometry`、`Redirects`、`Scheduling`、`SmokeInterference`。其中 14 项从旧版迁移（`AgentSensorCore.` 前缀改为 `UBotSensor.`，`SmokeInterference.*` 保持原名），其余为新增：

- `Scheduling.*`：在临时 Game 世界里手动 tick，验证 100 Hz IMU 在 0.05 s 帧里每帧恰好一个样本、时间戳严格递增、没有 `InvalidDeltaTime`；运行中改频率；相机协调器的一个间隔延迟、每 tick 至多一组、组序号与组时间戳、丢弃计数；无分组相机的 tick 采样和暂停行为。测试用相机只记录捕获、不渲染，不需要 GPU。
- `Interface.Implements`：每个具体传感器都实现 `IUBotSensor` 并返回预期的类型键。
- `Lidar2D.IntensityRange`、`Lidar3D.IntensityRange`：强度范围与对齐。
- `Medium.LegacyAndInterfacePaths`：两条发现路径对等价单元产生等价快照；`Medium.MatchesLegacyMath`：把随机介质场和倾斜相机同时交给内嵌的旧版实现和新实现，比较结果。
- `Redirects.Resolve`：读取 `Config/DefaultUBotSensor.ini`，逐条检查 `FCoreRedirects` 的结果，并确认每个旧类型、委托签名和网格都映射到存在的对象。
- `Camera.ImageFrameBGRA`、`Camera.ImageFrameDepth`、`Camera.TriggerConfiguration`、`Camera.CaptureGroupRegistry`。

测试需要在启用了 UBotCore 和 UBotSensor 的工程中用带编辑器上下文的 `UnrealEditor` 运行。编辑器内：`Tools` 菜单打开 Session Frontend 的 Automation 标签页，过滤 `UBotSensor`。命令行：

```powershell
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Project>\<Project>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotSensor; Quit' '-TestExit=Automation Test Queue Empty'
```

把 `UBotSensor` 换成更具体的前缀（如 `UBotSensor.Scheduling`）可以只跑一类。UBotCore 自己的测试用 `UBotCore` 前缀。

## 版本与许可

当前版本 **0.1.0**（Beta，`IsBetaVersion`），仓库 <https://github.com/AstreoX/uBotSensor>。

Copyright 2026 AstreoX。本项目以 [Apache License 2.0](LICENSE) 发布，可以自由使用、修改和再分发（包括商业用途），再分发时需保留 `LICENSE` 与 `NOTICE` 文件，并注明你修改过的文件。
