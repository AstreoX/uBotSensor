# 测试

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
