# X650 控制异常快照与指令发布时序

新运行在原有 CSV、反馈和控制诊断之外保存 `control_execution_trace.jsonl`。这项记录修复不改变模型、控制参数、任务协议、控制指令、消息处理次数或失败后的恢复策略；原来的记录与审计文件保留原样。

## 记录的时刻是什么

每次 controller `step()` 前后，以及每个关节 ROS publish／PX4 MAVLink send 调用前后，都保存本地单调时钟、已经处理的 `/clock` 值、JointState 原始 header 时间、关节回调接收时间和序号。读取这些值时不额外 spin，也不向 Gazebo 询问最新物理状态。

`local_call_returned=true` 只表示本地发布／发送函数正常返回。**它不是 Gazebo 接收或应用指令的确认**，所以始终保留 `downstream_application_confirmed=false`。计算期间没有处理新回调时，前后的仿真时钟与关节 header 可以相同；不得将本地计算耗时按未经验证的实时率换算成物理执行延迟。

每个关节单独保存发布返回记录，飞机 PVA 单独保存。若第一个关节成功发布、第二个关节发布失败，历史记录准确保留这种部分完成状态，不把两关节误写成一次原子执行。

`control_trace.csv` 另增加六列：计算开始／结束的 `wall_monotonic_s`、`processed_estimator_clock_s`、`joint_header_stamp_s`，前缀分别为 `compute_before_` 和 `compute_after_`。这些列描述本地计算边界。原有 `sim_s`、`joint_stamp_s`、指令和反馈列的语义保持不变。

## 控制计算失败时保存什么

调用控制器前先复制当次完整状态、目标、任务时间、dt、阶段和反馈原始诊断快照。`controller.step()` 抛异常时立即向事件流写入 `controller_step_exception` 并 flush，同时将第一次失败保存在 `control_failure_snapshot.json`。快照包括：

- 机体位置、旋转矩阵、速度、角速度、实测加速度、两关节角度／速度，以及已有的状态、控制和关节源时间字段。
- 末端目标位置、速度、加速度；实际使用的 PX4／关节反馈快照。
- 计算开始与失败返回边界的本地时钟及已处理消息时间。
- 各通道最后一次正常返回的本地发布指令和时间；不会将本次未生成的指令当成已发送。
- 原始异常类型、message、repr、traceback 和控制器提供的诊断。

优化失败中的 `inf`、`-inf`、`NaN` 用明确的 `{"nonfinite":"+inf"}`、`{"nonfinite":"-inf"}`、`{"nonfinite":"nan"}` 对象保存，不将它们变成 0。这样严格 JSON 序列化不会掩盖原求解异常。控制器异常仍原样抛出，由现有流程恢复机械臂、降落和恢复参数。

如果异常记录本身遇到 I/O 错误，记录器保留内存中的首次失败快照及记录错误，仍抛出原控制异常；外层 metadata 会尽可能保存这些信息。此机制不能保证磁盘不可用时成功落盘，也不会把不完整记录当成成功实验。

## 归档和验收兼容

关闭后生成 `control_execution_trace_manifest.json`，保存事件文件 SHA-256、成功写入数、事件序号尝试数、异常数、记录错误、最后本地发布、记录源码哈希以及首次异常快照哈希。`recording_closed` 与 `experiment_complete` 分开，且固定 `eligible_as_stage_gate=false`。

事件流、manifest 和存在时的异常快照加入本次原始 `stage_result.json` 的 artifact 映射。现有独立审计通过其通用 `verified_artifacts` 入口校验这些额外文件，并纳入最终证书的哈希链；不需要改写旧分析脚本。旧运行没有这些文件仍维持旧格式，但不能凭空补成具备新时序记录的实验。

这是后续分析指令、反馈持有和执行过程的观测证据。需要把本地发布与实际 Gazebo native 状态变化对照，才能判断端到端延迟；事件记录本身不产生延迟模型或执行器动力学验收结论。
