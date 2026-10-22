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

新证书 kind 为 `x650_ts20_conditional_stage_certificate_v1`。审计包括整个声明窗口、独立 native 末端与关节物理状态、ULog 中实际飞控命令/状态/参数、MIT 包完整性与单次延迟、真实消费历史、全时段关节约束以及计算/发送时序。关节整段自由预测只从首态初始化，分块时携带上块的预测终态，不用后续真实状态重置。

运行时使用同时间戳的物理 q 和条件模拟的舵机端点速度回报；Gazebo 区间平均速度另作审计。接触或约束导致内部与物理位置差超过 0.1 mrad 时，实验拒绝继续，并进入独立恢复/降落流程。最新发送包、当前生效包及完整包流必须与每个观测的序号、时间和内容一致；静止情况下缺失的零输入记录也不能通过。

## 结果解释

本场景验证的是来自约 20 V 台架模型的条件闭环。用户实际选型为 TS20-50／24 V；实际臂加载、轴端力矩比例和真舵机传感器仍待标定。原始质量和惯性没有被命令归一化参数替换。

水平二阶、垂向一阶响应在这里是旧机型的基体先验，元数据明确 `base_response_is_prior=True`。未来上层指令的计算和 ROS 传输耗时尚未主动补偿；3.346 ms 仅在 MIT 总线发送后应用一次。实际末端保持结果须按新记录解释，不能从离线 oracle 误差推断整机或真机精度。

开启外力保持之前还需要可核对的物理步外力记录及加载身份；当前未接通的外扰配置会硬拒绝。快速轨迹须在新的保持验收和相应新协议之后另行开放。
