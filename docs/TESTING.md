
# 测试

测试名都以 `UBotSensor.` 开头，共 36 项，分类为 `BodyState`、`Camera`、`Depth`、`Interface`、`Lidar`、`Lidar2D`、`Lidar3D`、`Math`、`Medium`、`Odometry`、`Redirects`、`Scheduling`、`SmokeInterference`。其中 14 项从旧版迁移（`AgentSensorCore.` 前缀改为 `UBotSensor.`，`SmokeInterference.*` 保持原名），其余为新增：

- `Scheduling.*`：在临时 Game 世界里手动 tick，验证 100 Hz IMU 在 0.05 s 帧里每帧恰好一个样本、时间戳严格递增、没有 `InvalidDeltaTime`；运行中改频率；相机协调器的一个间隔延迟、每 tick 至多一组、组序号与组时间戳、丢弃计数；无分组相机的 tick 采样和暂停行为。测试用相机只记录捕获、不渲染，不需要 GPU。
- `Interface.Implements`：每个具体传感器都实现 `IUBotSensor` 并返回预期的类型键。
- `Lidar2D.IntensityRange`、`Lidar3D.IntensityRange`：强度范围与对齐。
- `Medium.LegacyAndInterfacePaths`：两条发现路径对等价单元产生等价快照；`Medium.MatchesLegacyMath`：把随机介质场和倾斜相机同时交给内嵌的旧版实现和新实现，比较结果。
- `Redirects.Resolve`：读取 `Config/DefaultUBotSensor.ini`，逐条检查 `FCoreRedirects` 的结果，并确认每个旧类型、委托签名和网格都映射到存在的对象。
- `Camera.ImageFrameBGRA`、`Camera.ImageFrameDepth`、`Camera.TriggerConfiguration`、`Camera.CaptureGroupRegistry`。
- `Math.RigidPointKinematics`：`RigidPointVelocity` 和 `RigidPointAcceleration` 的方向与符号（ω 为 +Z、点在 +X 时速度指向 +Y，向心加速度为 −ω²r），以及它们与刚体上一点的有限差分一致。
- `Math.QuaternionDeltaSmallAngles`：`QuaternionDeltaToAngularVelocity`（差分模式的角速度）在转角很小时的转轴。0.0001 到 3 rad/s 的角速度，绕几个不是 X 的轴，施加在倾斜的姿态上（世界系：当前 = 增量 * 上一个），采样间隔 0.01 s 和 0.1 s（0.0199 与 0.0201 rad/s 跨过旧的 2e-4 rad 界限），上一个和当前四元数的符号（W 的正负）各取两种；结果与角速度在相对 1e-6 内一致（容限由舍入误差推出，写在测试里）。零转动得到的角速度在舍入误差内为零（1e-12 rad/s 以内），零时间恰好为零。
- `BodyState.*`（刚体状态源，测试里的状态源是一个只提供一个组件、状态可随意设置的测试组件）：
  - `ImuMatchesDifferences`：刚体沿解析轨迹运动（平移加速，绕倾斜轴转动并有角加速度），IMU 装在偏离原点、带旋转的安装位置上。状态模式与解析的比力和陀螺仪一致（误差 1e-6 以内）；200 Hz 的差分模式与状态模式相差不超过有限差分误差（用轨迹的加加速度上界计算，测试里写明了容限）；状态不带加速度时由状态历史推算，误差在一个状态步以内。
  - `ImuStatuses`：无状态源，以及有状态源但 `bUseBodyState` 关闭，输出相同，状态依次为 `WarmingUp`、`WarmingUp`、`Valid`；有状态源时 `WarmingUp`、`Valid`；刚体没有状态时 `WarmingUp`；推算加速度（状态时刻前进才推算、不前进则保持、推算不出来是 `WarmingUp`）；太旧或来自未来的状态给出 `InvalidDeltaTime`；切换模式回到 `WarmingUp`；`IsUsingBodyState` 和 `GetBodyStateAgeSeconds` 的取值。
  - `ImuNoise`：噪声开启时，相同种子和相同状态序列的两个 IMU 输出相同，噪声、滤波和限幅与独立写出的参考实现一致（两种模式都检查）；噪声关闭时输出等于无噪声的参考值。
  - `Odometry`：被跟踪组件偏离刚体原点时，速度等于 `P.Rot⁻¹ * (v + ω × r)` 和 `P.Rot⁻¹ * ω`，位姿与差分模式相同；暖机、年龄检查、状态消失和模式切换。
  - `Pose`：`World` 参考系从首个样本起就有带杠杆臂的速度；`Parent` 和 `CustomComponent` 参考系即使有状态源也用差分；刚体没有状态时没有速度；不检查年龄。

测试需要在启用了 UBotCore 和 UBotSensor 的工程中用带编辑器上下文的 `UnrealEditor` 运行。编辑器内：`Tools` 菜单打开 Session Frontend 的 Automation 标签页，过滤 `UBotSensor`。命令行：

```powershell
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Project>\<Project>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotSensor; Quit' '-TestExit=Automation Test Queue Empty'
```

把 `UBotSensor` 换成更具体的前缀（如 `UBotSensor.Scheduling`）可以只跑一类。UBotCore 自己的测试用 `UBotCore` 前缀。
