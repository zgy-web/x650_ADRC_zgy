# X650 TS20 新关节预测链与末端保持

这个场景为新条件执行器重新取得物理和控制证据，只允许 `hover_folded`、`joint_probe`、`tip_hold`。没有快速八字入口。X650 与原二连杆的刚体参数保留，两个理想关节速度控制器替换为 TS20 条件自由运动响应。详细边界和模型公式见 [TS20 接入](../../docs/X650_TS20_INTEGRATION.md)，模型准备见 [Gazebo 执行器说明](../../docs/X650_TS20_PLANT_README.md)。

## 准备和运行顺序

1. 用已有编译器构建数值加速器，记录实际库和源码哈希。
2. 以固定上游资料为独立参考，完成全部 8 个预先定义的数值试验。
3. 准备完整的独立 flight 模型与对应 airframe，保留原机体/臂几何及惯性，只继承几何证据。当前阻尼修正条件为 `x650_ts20_damped_hold_v1`／4996；原 4997 条件保留作历史记录。
4. 冻结已有 42 个 PID 数值，显式绑定新模型，并使用同一原始世界的精密 EV 模式。
5. 新地面稳定检查通过后，完成新悬停并审计 ULog、参数恢复、全时段实际输入和物理记录。
6. 用该悬停证书准入新的空中关节探针；验证实际方向、运动、完整首态自由预测以及飞机稳定性。
7. 用新悬停和关节证书准入 MPC 末端保持；真实滚动求解、下发、末端误差与时序均须独立验收。

软件试验通过、场景退出成功或读取到旧成绩都不等于取得本场景的完整通过证书。旧机型的飞行证书不能直接转用。

## 构建与独立数值验收

以下命令需要已有外部源文件、系统依赖与未使用的输出目录，不会下载依赖或连接硬件：

```bash
/usr/bin/python3 scripts/build_ts20_prediction.py \
  --output-dir generated/ts20_prediction_build_fresh

env -u PYTHONPATH OPENBLAS_NUM_THREADS=1 /usr/bin/python3 \
  scripts/validate_x650_ts20_prediction.py \
  --source artifacts/ts20_integration_20261022/upstream_snapshot \
  --manifest artifacts/ts20_integration_20261022/upstream_snapshot/source_manifest.json \
  --native-library generated/ts20_prediction_build_fresh/libts20_prediction.so \
  --native-sha256 ACTUAL_LIBRARY_SHA256 \
  --output artifacts/ts20_prediction_oracle_fresh
```

`ACTUAL_LIBRARY_SHA256` 取构建清单的 `library_sha256`。上游源码不随本仓库分发；清单校验完整源树的 12 个文件。验收报告绑定本次 Python 预测器、C++ 数值核、库字节和固定试验协议。只有构建成功而未通过独立数值验收的库不能用于飞行准入。

## 场景配置和验收

当前条件调用 `task.build_damped_task_config(stage)`，生成 `x650_ts20_damped_hold_v1`／`amsim_urdf_x650_2link_ts20_damped_v1_flight`／4996 的冻结协议，有效 MIT 增益为 Kp = 6、Kd = 0.25，总线周期 5 ms。原 `task.build_task_config(stage)` 和 airframe 4997 示例只对应历史 Kd = 3276/32768 条件；它们的模型、oracle、悬停和关节探针证书不能用于新条件准入。

调用者显式补齐绝对输入路径、已通过的当前 oracle 报告、相同库字节的 SHA256、原世界及 PID 资料；`tip_hold` 还需要匹配 PID 的旧基体响应先验，以及同一新条件的 `hover_folded`、`joint_probe` 证书。配置由 `task.validate_task_config` 和 `task.preflight` 再次校验。预测器的 Kp、Kd 和总线周期由哈希绑定的 plant manifest 显式读取，不能沿用构造器默认增益。

新增益的独立数值验收沿用原 8 个夹具、相同初态、输入和误差门槛，显式选择独立增益协议与候选条件文件：

```bash
env -u PYTHONPATH OPENBLAS_NUM_THREADS=1 /usr/bin/python3 \
  scripts/validate_x650_ts20_prediction.py \
  --source artifacts/ts20_integration_20261022/upstream_snapshot \
  --manifest artifacts/ts20_integration_20261022/upstream_snapshot/source_manifest.json \
  --native-library generated/ts20_prediction_build_fresh/libts20_prediction.so \
  --native-sha256 ACTUAL_LIBRARY_SHA256 \
  --gain-profile mit_kp6_kd025_v1 \
  --operating-condition configs/actuators/ts20_50_24v_damped_v1_candidate.json \
  --operating-condition-sha256 9d23be903984c38373e1ea0488fb3c36f09d89504e4b9c78ae85820a415dcb97 \
  --output artifacts/ts20_prediction_oracle_damped_fresh
```

前面的无 `--gain-profile` 命令保留原始历史增益协议。Kd = 0.25 是仿真条件的阻尼候选，既没有改写约 20 V 台架摩擦参数，也不代表 24 V 真舵机的增益读回或带载验证。关节探针 0.2 rad/s 和发布前总计 50 ms 的验收门槛均保留。

```bash
/usr/bin/python3 simulations/x650_ts20_hold_v1/launch.py \
  --task /absolute/path/to/task.json --dry-run

/usr/bin/python3 simulations/x650_ts20_hold_v1/launch.py \
  --task /absolute/path/to/task.json

/usr/bin/python3 simulations/x650_ts20_hold_v1/audit.py \
  --run-dir /absolute/path/to/new_native_run \
  --launch-dir /absolute/path/to/owned_launch \
  --output /absolute/path/to/new_audit
```

实际运行应复用服务器已有兼容的 NumPy/SciPy、ROS 和数据盘模块环境。新 launcher 配置本次进程的依赖顺序并在起飞前记录实际 Python、NumPy、SciPy 路径，不全局修改原虚拟环境。运行进程、参数恢复与着陆由平台生命周期管理。

2026-10-23 的 FFI 生命周期修正采用新的 `launch_chain_v4.py`，支持相同的悬停、关节探针和安静保持阶段。它额外要求显式的软件证明及 SHA，逐文件核对实际部署与新 oracle，并归档本次实际平台来源；不改变 task／审计或控制参数。使用未占用的输出和对应阶段的完整 task 配置，例如先进行只读准入检查：

```bash
/usr/bin/python3 simulations/x650_ts20_hold_v1/launch_chain_v4.py \
  --task /absolute/path/to/new_stage_task.json \
  --software-proof /absolute/path/to/reviewed_software_acceptance.json \
  --software-sha256 REVIEWED_SOFTWARE_SHA256 --dry-run
```

检查通过后，去掉 `--dry-run` 执行该阶段。应先取得同一新条件的悬停与关节证书，再运行保持；不能直接沿用修正前的旧证书。[软件结果与边界](../../docs/X650_TS20_INTEGRATION.md#10-ffi-生命周期修正与新的实际阶段准入)说明了本次修正没有改变数值输出，历史 53.14 ms 超时仍未确立原因。`launch_quiet_v3.py` 保持历史字节，用于解释对应失败档案。

新证书 kind 为 `x650_ts20_conditional_stage_certificate_v1`。审计包括整个声明窗口、独立 native 末端与关节物理状态、ULog 中实际飞控命令/状态/参数、MIT 包完整性与单次延迟、真实消费历史、全时段关节约束以及计算/发送时序。关节整段自由预测只从首态初始化，分块时携带上块的预测终态，不用后续真实状态重置。

运行时使用同时间戳的物理 q 和条件模拟的舵机端点速度回报；Gazebo 区间平均速度另作审计。接触或约束导致内部与物理位置差超过 0.1 mrad 时，实验拒绝继续，并进入独立恢复/降落流程。最新发送包、当前生效包及完整包流必须与每个观测的序号、时间和内容一致；静止情况下缺失的零输入记录也不能通过。

## 因果关节入口与当前验证状态

后续的[实际诊断保持](../../docs/validation/ts20_prediction_chain_20261022/diagnostic_quiet_actual_20261023.json)在 386 次 measure 发布后被旧 `EstimatedFeedback.read` 的未来 ROS JointState 保护拒绝，TS20 替换尚未发生，失败轮也未调用 allocator。独立 native partial 窗仅 13.020 s，不能作为完整 30 s 保持通过。一次 25.323905 ms 自然 generation 2 GC callback 区间位于最后一次 measure PVA 返回与首次恢复发布之间；[时段证据](../../docs/validation/ts20_prediction_chain_20261022/gc_gen2_timeline_overlap_v1.json)没有失败 read 的精确边界，不能据此确认 GC 引发该异常，也不能归因另一轮历史 56 ms 失败。

新版只在 upper PX4 estimated feedback 活跃时，先以固定 estimator `/clock` 调用一次 `select_joint_observation`。它选择不在未来、年龄不超过 25 ms 的完整 TS20 记录，并冻结同源 q／端点 dq／r、已读包、消费历史和总线原点。公开数值不可变，generation 与 runtime 持有的 opaque 对象身份共同绑定本轮；新选择开始即撤销旧 token，选择失败也不能沿用旧快照。`after_state_read` 只绑定该次观测，不二次读 CSV，也不先投影 q/dq 后再次传播。机体位置、姿态、速度、角速度和加速度仍由 PX4 提供，独立 truth 仅用于监督和验收。

新 upper 路径的 `feedback_snapshot.control_state` 关节值就是实际 TS20 观测，原 ROS 值单独记在 `raw_ros_joint_diagnostic`；历史旧 run 的该快照则在 TS20 替换前读取，须按实际源码身份解释。上层调度与任务计时不由原 ROS joint header 推进，非 upper 悬停／关节 commissioning 保持原时钟入口。默认 legacy 的未来测量保护、实际 TS20 的未来／过龄／序号／缺包／物理失配保护不变；恢复禁用实验、使观测失效，不靠 TS20 日志读取才能降落。

循环 observer 仍不主动 collect／disable GC 或改阈值；新版以三代各两个完整峰值事件、共固定 6 槽补足最近 64 个事件环的长期峰值时段记录。11 项 observer 及 provider／默认／恢复路径组成的 42 项本地合并回归保留为开发证据；随后[服务器 fresh 软件证明](../../docs/validation/ts20_prediction_chain_20261022/causal_joint_software_acceptance_20261023.json)绑定 86 项实际源，新执行 288 项测试（124 package＋134 interface／audit＋30 shared-loop）全部通过、0 skip，显式提供 native 测试库。原 30 个矩阵夹具重新运行全部通过，最大 26.684829965 ms；full/view 的矩阵、计划、状态、编码包和非线性输出等价，独立 timing 全部保留，最大 full／view 分别为 37.344370／26.682980 ms。证明 JSON 原始字节 SHA 为 `dfc55fea230a857829a5a109219db1c15e18f46f9f7f54b76d8feb4e95359201`。

首个 `software_v1` 保留 248 项、0 failure／4 个同源 ImportError 的失败输出，缺失的是服务器部署中已有跟踪的 `test_x650_precision_stage_audit.py` helper，没有飞行。补齐该文件精确字节（SHA `36c4d9f24185dff7a714ba7b48b282d909c3006cf02f9d014effe06dbea94699`），不改控制或 driver，在全新 `software_v2` 重跑上述软件验收；不继承旧测试／矩阵成绩。新版 fresh folded hover 与关节探针已通过完整独立验收；因果输入修正后的首次保持在 64 ms 控制间隔保护处失败，只有 6.412 s partial 记录，不能认证完整 30 s，快速轨迹未运行；软件通过不能升级旧飞行证书，旧记录和源码归档保持原身份。更完整的机制与边界见 TS20 接入[第 6 节](../../docs/X650_TS20_INTEGRATION.md#6-新预测链和单独的仿真工况)和[第 11 节](../../docs/X650_TS20_INTEGRATION.md#11-循环层时序诊断)。

本轮保持的[精确失败记录](../../docs/validation/ts20_prediction_chain_20261022/causal_quiet_actual_20261023.json)保存了失败轮状态与循环边界。31.200 ms 自然 gen2 callback 区间完全位于 35.103 ms 的诊断转换／编码间隙，实际 write 仅 0.071 ms；下一轮在 allocator 之前拒绝 64 ms。安全收尾、部分 ULog／因果输入及源码档案核验通过，不等于保持通过。诊断序列化和记录在线聚合候选目前仅本地评估，未改 GC 策略或控制／模型／50 ms 门槛。

## 结果解释

本场景验证的是来自约 20 V 台架模型的条件闭环。用户实际选型为 TS20-50／24 V；实际臂加载、轴端力矩比例和真舵机传感器仍待标定。原始质量和惯性没有被命令归一化参数替换。

水平二阶、垂向一阶响应在这里是旧机型的基体先验，元数据明确 `base_response_is_prior=True`。未来上层指令的计算和 ROS 传输耗时尚未主动补偿；3.346 ms 仅在 MIT 总线发送后应用一次。实际末端保持结果须按新记录解释，不能从离线 oracle 误差推断整机或真机精度。

开启外力保持之前还需要可核对的物理步外力记录及加载身份；当前未接通的外扰配置会硬拒绝。快速轨迹须在新的保持验收和相应新协议之后另行开放。
