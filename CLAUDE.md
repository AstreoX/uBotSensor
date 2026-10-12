# uBotSensor

UBotSensor is the sensor plugin of the uBot family (UE 5.5). It provides simulated sensors as Blueprint-first scene components: pose, odometry, IMU, 2D/3D lidar, RGB camera and depth camera, plus perception media (smoke) that disturb lidar and cameras. It is built on UBotCore (frame header, sim clock, sample scheduler, sensor interface) and has no ROS dependency; UBotROS publishes this data to ROS 2.

- UBotCore is required: `Plugins` in `UBotSensor.uplugin`, and `UBot.Requires` pins `^0.1.0`.
- Do not add ROS headers or rclcpp here.

## Layout

- `Source/UBotSensor/` - Runtime module. `Public/` is the API, `Private/` the implementation, `Private/Tests/` the automation tests.
- `Source/UBotSensorEditor/` - Editor module: lidar and camera component visualizers, registered in `UBotSensorEditorModule.cpp`. No tests.
- `Content/Models/SM_UBotSensor_<Kind>.uasset` - sensor meshes (binary, editor-only to change).
- `Config/DefaultUBotSensor.ini` - the plugin's `[CoreRedirects]`. `Config/FilterPlugin.ini` - extra files staged with the plugin.
- `docs/api.json` - generated Blueprint API. `docs/brand/` - logo used by the README.
- `README.md` (Chinese) documents each sensor's defaults and behaviour; update the matching section when you change them.

## Sensors and shared pieces

Headers are in `Source/UBotSensor/Public/`. All sensors derive from `UUBotSensorComponent`.

| Type key | Component class | Header | Output |
| --- | --- | --- | --- |
| `Pose` | `UUBotPoseSensorComponent` | `UBotPoseSensorComponent.h` | `FUBotPoseSensorData` |
| `Odometry` | `UUBotOdometrySensorComponent` | `UBotOdometrySensorComponent.h` | `FUBotOdometrySensorData` |
| `IMU` | `UUBotImuSensorComponent` | `UBotImuSensorComponent.h` | `FUBotImuSensorData` |
| `Lidar2D` | `UUBot2DLidarSensorComponent` | `UBot2DLidarSensorComponent.h` | `UUBot2DLidarFrame` |
| `Lidar3D` | `UUBot3DLidarSensorComponent` | `UBot3DLidarSensorComponent.h` | `UUBotPointCloudFrame` |
| `RGBCamera` | `UUBotRgbCameraSensorComponent` | `UBotRgbCameraSensorComponent.h` | `UUBotImageFrame` (BGRA8) |
| `DepthCamera` | `UUBotDepthCameraSensorComponent` | `UBotDepthCameraSensorComponent.h` | `UUBotImageFrame` (Depth32FC1) |

Data structs and enums are in `UBotSensorTypes.h`; frame objects in `UBot2DLidarFrame.h`, `UBotPointCloudFrame.h`, `UBotImageFrame.h`. Cameras share `UUBotCameraSensorComponent` (render target capture, readback, capture groups) with `UUBotCameraCaptureCoordinatorComponent` and `UUBotCameraCaptureSubsystem`.

Shared by every sensor:
- Base class `UUBotSensorComponent` (`UBotSensorComponent.h`): `StartSensor`/`StopSensor`/`SampleSensor`, `UpdateRateHz`, per-sensor sequence numbers, `CreateFrameHeader`/`MarkHeader`. Subclasses implement `CaptureSensorData` and `GetSensorTypeName`, and may override `OnSamplingStarted`.
- Frame header and sample state: `FUBotFrameHeader` and `EUBotSampleStatus` (`Valid`, `WarmingUp`, `InvalidDeltaTime`, `TeleportDetected`, `Disabled`, `Error`) from UBotCore `UBotTypes.h`.
- Scheduling: `FUBotSampleScheduler` (UBotCore) polled in `TickComponent`, driven by `UBot::Clock::GetSimTimeSeconds` (UBotCore `UBotClock.h`).
- Noise: base properties `bNoiseEnabled` and `RandomSeed`; each sensor keeps its own `FRandomStream`. Helpers in `UBotSensorMath.h` (`UBot::SensorMath`: Gaussian draws, covariance, lidar and camera geometry).
- Media: `UBotSensorMedium.h` (`UBot::SensorMedium`) over UBotCore `UBot::Medium` and `IUBotPerceptionMedium`.
- Log category `LogUBotSensor` (`UBotSensor.h`).

## Adding a sensor

Follow the existing sensors. Sensors are found through the `IUBotSensor` interface (`UBot::Sensors::FindSensors`), so there is no registry to update.
1. Add `Public/UBot<Name>SensorComponent.h` and `Private/UBot<Name>SensorComponent.cpp`. Derive from `UUBotSensorComponent`, or from `UUBotCameraSensorComponent` for image sensors (override `GetImageEncoding`, `GetRequiredRenderTargetFormat`, `GetCaptureSource`).
2. In the constructor load `/UBotSensor/Models/SM_UBotSensor_<Kind>` with `ConstructorHelpers::FObjectFinder` (check `Succeeded()`), and set default `SensorId`, `FrameId` and `UpdateRateHz`.
3. Override `GetSensorTypeName()` to return the static type key (the value `IUBotSensor::GetSensorType` reports).
4. Implement `CaptureSensorData(Header, OutErrorMessage)`: fill the output, set the status with `MarkHeader`, store `Header` in the output, broadcast the data event, return true only for a valid sample. Set `OutErrorMessage` to raise `OnSensorError`. Reset history and `RandomStream.Initialize(RandomSeed)` in `OnSamplingStarted`.
5. Declare output as a `USTRUCT(BlueprintType)` carrying `FUBotFrameHeader Header` (`UBotSensorTypes.h`) or a `UCLASS(BlueprintType)` frame object, a `DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam` event, a `BlueprintPure` `GetLatest<X>...()` and a `BlueprintCallable` `Sample<X>()` that calls `SampleSensor()`.
6. Add a visualizer in `UBotSensorEditor` only if the sensor needs viewport drawing.
7. Tests: add the class and type key to `ExpectedSensors` in `UBotSensor.Interface.Implements` (`UBotSensorComponentTests.cpp`); add `UBotSensor.<Name>.*` tests for the sensor's logic.
8. Add a `Sensor.<Name>` key to `UBot.Provides` in `UBotSensor.uplugin`; it must agree with the entry in the UBotCore package index.
9. Update the README section and regenerate `docs/api.json`.

## Behaviour rules

- Sampling runs from the component tick (`TG_PostPhysics`) through `FUBotSampleScheduler`, at most one sample per tick; skipped intervals count in `GetDroppedSampleCount()`. Never sample from looping timers and never clear `PrimaryComponentTick.bCanEverTick`.
- Timestamps are uBot sim time (`GetSensorTimestampSeconds()`, i.e. `UWorld::GetTimeSeconds`). Do not use wall-clock time for headers or deltas (`FPlatformTime` is used only for the diagnostic `CaptureDurationMilliseconds`).
- Velocities and accelerations (Pose, Odometry, IMU) are finite differences of component poses over header timestamps, with `UBot::SensorMath::QuaternionDeltaToAngularVelocity`. Do not read physics velocity.
- Invalid samples (`WarmingUp`, `InvalidDeltaTime`, `TeleportDetected`, `Disabled`, `Error`) still carry a header with `bValid == false`, and `OnSensorSampled` fires for every sample. Pose, Odometry and IMU also broadcast their data event on every captured sample, valid or not; lidar and cameras broadcast a frame event only when a frame was produced.
- IMU linear acceleration is specific force: world gravity (`GetGravityZ`) is subtracted before rotating into the sensor frame.
- Units: public data is SI (meters, seconds, radians). Names carry the unit (`...Meters`, `...RadPerSecond`, `...Degrees`, `...Centimeters`). UE transforms are centimeters (`SensorTransformWorld`, medium cell positions); convert with `UBot::Units`.
- Axes stay UE (X forward, Y right, Z up); no ROS conversion in this plugin.
- Determinism: random draws come only from the component's `FRandomStream`, seeded with `RandomSeed` at start, and only when `bNoiseEnabled`. No `FMath::Rand*`.
- Cameras render into a render target; CPU pixels exist only after `UUBotImageFrame::RequestPixelReadback`, and only for the latest capture. A camera with `CaptureGroupId` never self-starts; its coordinator captures the group with a shared sequence and timestamp.
- `SmokeVolumeActorTag` defaults to `AgentSensor.SmokeVolume` (`UBot::SensorMedium::LegacySmokeVolumeTag`); existing smoke actors rely on it. Do not change it.

## Build and test

The plugin builds inside a host UE 5.5 project that loads it together with UBotCore: clone both under `<Host>/Plugins/uBot/` (or list their parent folder in the host `.uproject` `AdditionalPluginDirectories`) and enable `UBotCore` and `UBotSensor` in its `Plugins`. Close running editors before rebuilding C++.

```powershell
& '<UE_5.5>\Engine\Build\BatchFiles\Build.bat' <Host>Editor Win64 Development '-Project=<Host>.uproject' -WaitMutex -NoHotReloadFromIDE
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Host>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotSensor; Quit' '-TestExit=Automation Test Queue Empty'
```

- All tests start with `UBotSensor.` (`UBotSensor.Scheduling`, `UBotSensor.Interface`, `UBotSensor.Camera`, `UBotSensor.Lidar2D`, `UBotSensor.Medium`, `UBotSensor.Redirects`, ...). Narrow the filter to one prefix to run a subset.
- Tests use `EditorContext | EngineFilter`, so they need `UnrealEditor-Cmd` with the host project. They run with `-NullRHI`; scheduling tests use a recording test camera that never renders.
- Cloud sessions have no Unreal Engine: you can read and edit code, but cannot compile or run tests. Say so in the PR ("not compiled, tests not run"); never claim they passed. Mirror existing patterns closely, keep `.generated.h` as the last include, and check every new reflected symbol against the headers.

## Conventions

- Unreal C++: four spaces, Allman braces, `U`/`A`/`F`/`E`/`I` prefixes (class `UUBotImuSensorComponent` lives in `UBotImuSensorComponent.h`), `b`-prefixed booleans, `UBOTSENSOR_API` on exported types. Public headers in `Public/`, implementations in `Private/`.
- Tests: `IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensor<Thing>Test, "UBotSensor.<Category>.<Name>", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)`, in `Private/Tests/*Tests.cpp` inside `#if WITH_DEV_AUTOMATION_TESTS`. Reflected test-only types go in a header there (UHT cannot see types declared in a `.cpp`); see `UBotSensorSchedulingTestTypes.h`.
- Blueprint API: sensors are `UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot <X> Sensor"))`. Properties are `EditAnywhere, BlueprintReadOnly` with `Category = "<Group>|<Sub>"` (for example `"IMU|Noise"`; shared ones use `"uBot Sensor"`) and `ClampMin`/`ClampMax`/`EditCondition` meta. Functions use `Category = "uBot Sensor|<Group>"`; getters are `BlueprintPure`. Events are `BlueprintAssignable`. Runtime state is `UPROPERTY(Transient)`. Functions that copy whole arrays get `DisplayName = "Copy ... (High Cost)"`.
- Do not add a `UFUNCTION` that shadows an `IUBotSensor` function (`IsSensorRunning` is deliberately native-only on the component).
- Doc comments on reflected declarations become tooltips and are exported to `docs/api.json`; keep them accurate.
- Blueprint-visible names (classes, structs, enums, enumerators, properties, functions, delegate signatures) are what user Blueprints bind to. Renaming or removing one breaks user assets: prefer additive changes. If a rename is unavoidable, add the redirect to `Config/DefaultUBotSensor.ini`; `UBotSensor.Redirects.Resolve` checks every line.

## docs/api.json

- It is the Blueprint API description exported by UBotCore's `UBotDocExport` commandlet. It is generated; never edit it by hand. Keep LF line endings.
- Regenerate after changing reflected API (classes, properties, functions, categories, tooltips, defaults) and before tagging a release. This needs a local UE install with the host project.

```powershell
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Host>.uproject' -run=UBotDocExport -Plugin=UBotSensor -Out=Saved/Docs/UBotSensor.zh.json -culture=zh-Hans -NullRHI
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Host>.uproject' -run=UBotDocExport -Plugin=UBotSensor -Out=<UBotSensor>/docs/api.json -MergeZh=Saved/Docs/UBotSensor.zh.json -culture=en -NullRHI
```

- Relative `-Out` and `-MergeZh` paths resolve against the host project directory; `<UBotSensor>` is the absolute path of this repo. Output is deterministic, so an unchanged API gives an empty diff.
- Cloud sessions cannot regenerate it; say so in the PR if reflected API changed.

## Versioning and commits

- `VersionName` in `UBotSensor.uplugin` is the version; the release tag is `vX.Y.Z` with the same number. The `version` field of `docs/api.json` mirrors it, so regenerate after a bump.
- The package index lives in the uBotCore repo (`Index/index.json`): a release adds a version entry whose `ref` is the tag, and `requires`/`provides` must match the `.uplugin` at that tag. Cloud sessions must not bump the version or create tags.
- Never commit `Binaries/`, `Intermediate/`, `Saved/` or `DerivedDataCache/` (they are git-ignored).
- Conventional Commit messages, imperative, scoped by sensor or area: `fix(imu): reset history after teleport`, `feat(lidar): ...`, `docs: ...`.
