# X650 无臂悬停验证

本目录提供 X650 无臂正常起飞、20 s 悬停测量及降落协议。历史运行证据单独归档；本源码发布不包含其原始模型、视频或日志，不能用本文代替一次新运行的验收。使用者需提供外部源 URDF 与全部 mesh，并保存本次实际结果、参数和来源哈希。
场景使用已明确绑定的 X650 动力模型与 PX4 airframe，不加载原 x500 两关节 PID＋前馈控制包。

在平台环境中先执行不启动 ROS、PX4 或 Gazebo 的检查：

```bash
PYTHONPATH=src /root/autodl-tmp/amsim-venv/bin/python3 simulations/x650_hover_v1/run_case.py --dry-run
```

缺少 flight model 时会列出需要补齐的动力信息并以退出码 2 结束。
模型存在时还会验证 URDF 来源哈希、资源完整性、空机械臂关节列表及 PX4 airframe 是否已构建。
配置中的 `expected_urdf_sha256` 已固定为审核后的 `x650_mn4014_v1` 派生 URDF 哈希；更改旋翼坐标、连续转动关节或速度上限后，必须重新检查派生模型并更新哈希。
`original_urdf_sha256` 独立记录最初 CAD 导出文件的来源参考；它不代替派生模型校验，也不单独证明模型转换链。

参数和飞行配置确认后，通过平台执行：

```bash
./scripts/amsim run x650_hover_v1
```

协议：从正常解锁前检查开始，起飞 2 m，稳定 3 s，额外稳定 5 s，测量悬停 20 s，保持 5 s，正常降落并确认自动上锁。
不强制解锁，不直接发送电机命令。定点由选定 airframe 的 PX4 飞行控制器执行。

`flight_samples.csv` 按目标 20 Hz 仿真时间间隔记录起飞前、起飞、稳定、测量、后保持、降落、落地全过程，保存原始 Gazebo 世界位姿（位置、xyzw 四元数）、固定悬停目标、位置误差、欧拉角、PX4 NED 位置、解锁/落地/失效保护状态、当前阶段和模型 SHA256。
该文件是视频的真实位姿来源；采样时间保留原消息时间戳，间隔丢失不会补造或插值。PX4 状态是同次回调中最新接收的值，另外保留其原始微秒时间戳说明异步关系。
采集器是现有单线程 ROS executor 中独立的 Odometry 订阅：起飞和降落命令内部继续处理回调，因此不依赖后台采样线程、不修改平台 API、不争用 MAVLink 端口。CSV 持续写入并定期 flush，失败时也保存已收到数据。
`flight_samples_manifest.json` 保存 CSV SHA256、模型 SHA256、坐标和四元数定义、实际采样间隔及阶段计数。四元数为 base_link 到 Gazebo world 的主动旋转，未进行 ENU/NED 或 FLU/FRD 转换；原始 `frame_id` 和 `child_frame_id` 同时保留。
`hover_samples.csv` 是其中稳定、测量、后保持区间的兼容子集；
测量目标固定为起飞前模型基座世界位置加 2 m 高度，不会随实际悬停漂移重置目标。
`result.json` 保存质量、模型与动力配置 SHA256、起降事件及位置/姿态统计。
位置 RMSE 0.25 m、最大误差 0.50 m、最大倾角 15° 是本次初始接入的验收门槛，属于实验要求而非硬件性能声明。
超过验收门槛仍正常降落，结果标为失败；出现通信过期、估计器重置、失效保护或超出试验范围会尝试降落并记录结果。
完整飞行 CSV 与原始 rosbag 都覆盖起飞与降落。固定目标只代表悬停测试点，不能当作起飞和降落阶段 PX4 的时变轨迹参考。
平台在场景结束后归档完整 PX4 ULog 和输入模型快照。

## 运行中的参数读回

起飞前和确认落地后，场景通过原有 `VehicleAPI.link` 逐项发送 `PARAM_REQUEST_READ`，保存 `px4_parameters_before.json` 和 `px4_parameters_after.json`。
请求范围是已安装 airframe 的全部字面量 `param set` 项、`SYS_AUTOSTART`、`MPC_USE_HTE`，以及代码中明确列出的 MC/MPC 控制增益和限值；这不是全量参数导出。
`px4_airframe.sh` 保存实际安装的 airframe；参数记录与 `result.json` 绑定其 SHA256，结果另保存两份 JSON 的 SHA256。采集辅助代码也随结果保存。

物理分配、电机输出和 airframe 参数必须在起飞前读回并匹配，否则保留失败证据且不解锁。核心 MC/MPC 项若在当前固件不存在，会标注不可用，不能用源码默认值填补。
`MPC_USE_HTE` 只读回记录，不修改开关；落地后的悬停推力估计或其他参数变化逐项记录，不把前后不同的数据宣称为固定参数结果。`parameters_unchanged` 仅比较这两次快照，不证明飞行途中没有短暂改动。

临时 `message_hooks` 观察同一连接中的 `PARAM_VALUE`，正常的 VehicleAPI 仍负责 ROS、心跳和 ACK；读回完成或失败时移除自己的 hook，不影响其他消费者。
整数按 PX4 的字节编码从原始 payload 解码，避免把 INT32 位模式当作普通浮点数；也处理 MAVLink 2 的尾部零截断。
相关协议依据：[pymavlink 接收 hook](https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py)、[固定 PX4 提交的参数实现](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_parameters.cpp)、[MAVLink 参数编码](https://mavlink.io/en/services/parameter.html#parameter-encoding)。
服务器实际 pymavlink MAVLink 1/2 编解码测试 11/11 通过，真实 PX4 起飞前和降落后各读取 101 项参数，均匹配所需配置且两次值未变化。为避开该 pymavlink 版本的 `get_payload()` 偏移问题，解码器从 `get_msgbuf()` 的完整帧按实际版本头长截取参数数据。

真实飞行后可使用 `scripts/render_x650_hover.py` 将完整真实 X650 网格与 `flight_samples.csv` 合成视频，详见 [回放说明](../../docs/X650_HOVER_REPLAY.md)。画面明确注明 Gazebo 记录位姿回放，非原生录屏；不能把旧 x500 动画标为 X650 视频。
