# 高频 IMU 与 3D LiDAR 执行方式

本页描述未发布的源码变更。请统一重新编译 Core、状态提供者、Sensor 和 ROS；资产仍可加载，旧 DLL 不保证兼容。

## IMU

`SamplingMode` 默认为 FrameTick，保留启动立即采样和每个世界 tick 最多一次采样的行为。

在 Play 前显式选择 PhysicsTimeline，同时保持 Use Body State 开启，并使用支持历史的状态提供者。没有该能力时，启动失败；运行中失去提供者时停止并报告错误，不回退到伪造历史样本。

新模式使用固定仿真时间网格：例如 200 Hz 的间隔为 0.005 秒。20 FPS 时允许一帧生成多个样本；只消费已经完成的物理状态，未来时刻留待后续帧。启动时等待有效状态，首个测量仍可能为预热状态。手动请求最多消费一个到期样本，没有新数据时不重复生成。

挂载点运动、重力、滤波、噪声和偏置随机游走按对应采样时刻和间隔计算。数据源、挂载和连续段变化会重置测量历史；新挂载不能追溯应用到旧采样时刻。随机偏置不会仅因运动不连续就重新归零。

诊断计数在 Start Sensor 时清零：

| 接口 | 含义 |
| --- | --- |
| GetDroppedSampleCount | 调度器直接跳过的网格时刻；包含历史耗尽和重新绑定，不包含查询失败或无效测量 |
| GetTimelineHistoryDroppedSamples | 早于提供者保留窗口的时刻 |
| GetTimelineUnavailableSamples | 不连续区间等无法查询的到期时刻 |
| GetTimelineRebindDroppedSamples | 挂载或提供者变化后不能追溯恢复的时刻 |

GetTimelineUnavailableSamples 单独统计已经消费但不能查询的时刻，不要与调度器计数混为同一总数。预热、无效时间间隔等状态通过测量的 Header.Status 和 Header.bValid 判断；这些时刻可能生成事件，但默认不发布到 ROS。

高仿真采样率不表示墙钟消息均匀到达，也不表示仿真实时因子达到 1。历史状态不包含历史场景，LiDAR 和相机不会因此补采过去的环境。

## 3D LiDAR

快照扫描模型保持不变：同一帧的射线对应同一采样时刻，点的时间偏移仍为零。点序、ring、强度、固定种子噪声和量程判定保持一致。已发布点云拥有自己的存储，后续扫描只复用内部临时数据。

`QueryMode` 提供 Auto、Serial、Parallel。Auto 按已验证的规模门槛选择；Serial 可用作对照；Parallel 是请求，仍受安全条件限制。标准 UE 5.5 下，查询使用引擎公开场景查询接口及其读锁，工作结束后才在游戏线程整理结果和广播事件。

当前 Auto 的保守界限为 23,040 条射线（32×720），默认最多 8 个连续通道查询块，只有两个以上可执行块才报告并行。单个及四个传感器在原生采集、bridge 和完整 SLAM 三阶段各完成三轮配对比较后才启用此默认值。128 条射线的小扫描比较中出现了中位数退化，收益也未超过重复波动，因此更小规模继续串行。这个界限代表已验收范围，不声称是所有硬件和场景的最佳交叉点；需要控制 CPU 用量时可显式选 Serial。

冻结的物理求解器、存在 Cluster Union 的世界、引擎查询调试绘制开启以及碰撞分析器正在录制时，保持串行查询。Cluster Union 的组件扩展查询尚未通过并行等价验收，因此不默认启用该路径。

`GetLastCaptureTimings` 分别报告准备、碰撞查询、结果整理、事件分发耗时和实际是否使用并行。原点云 CaptureDurationMilliseconds 仍只计采集，不包含后续事件回调和调试绘制。

查询阶段包含构造每条世界射线和公开引擎查询的开销；它不是引擎碰撞函数的独占 CPU 时间。Dispatch 在广播返回后才完整，在该广播的回调内部读取会得到尚未完成的值。

可通过 UBotSensor.Lidar3D.* 和 UBotSensor.IMU.Timeline.* 运行回归。原生性能基准 UBotSensor.Performance.LidarPipeline 需要显式设置 UBOT_SENSOR_BENCH_OUTPUT 为 CSV 文件路径；默认每组预热 10 秒、测量 60 秒，执行三轮串行对照、缓存串行与并行比较。UBOT_SENSOR_BENCH_LIDARS 支持 1 或 4。短时冒烟运行不能作为性能验收。
