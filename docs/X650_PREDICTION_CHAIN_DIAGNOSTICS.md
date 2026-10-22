# 关节执行与姿态几何的开发诊断

在服务器原项目中运行，原始 CSV/JSONL/日志不需要转移：

```bash
python3 scripts/analyze_x650_prediction_chains.py \
  --run /absolute/mpc_tip_hold_run --output /absolute/new_diagnostic_directory
```

若原 task 指向的 expanded world 不在本机，或没有显式 gravity，可另传 `--gravity-m-s2 9.80665`；该值会明确标记为人工指定的诊断假设。默认读取并绑定原 world 的 gravity，不估计或拟合重力。

脚本接受 MPC `tip_hold`。失败 run 使用真实测量起点到最后成功发送控制记录；完成 run 使用真实 `task_end_sim_s`。两种情况都只输出 `diagnostic_only: true`、`eligible_as_stage_gate: false` 的开发报告，不代替独立验收。失败窗口不能与完整 PID 窗口正式排名。

`prediction_chain_diagnostic.json` 是紧凑统计，`joint_command_effect_events.json` 保留逐条命令的匹配或未辨识原因，`prediction_chains.png/.pdf` 展示原始时序。所有原始输入和输出都有 SHA256；没有优化器执行、formal 模型重新拟合、减均值或最佳相位对齐。

关节部分分别检查：

- native q 是否等于从首次 q 出发、对同一步 dq 的积分，分别报告右端和梯形积分误差；这是离散化一致性，不是新增执行器模型。
- 控制记录的 q/dq 与 feedback JSONL 是否一致，再按真实 JointState header 时刻与独立 native 比较。另报告消费时刻误差和 header 年龄，避免把旧样本当同时刻状态。
- 每个变化超过两倍容差的速度命令，寻找之后 120 ms 内原始 native dq 的新值跃迁，载荷容差为 1e-6 rad/s。拒绝事件先于命令消费、区间跨越消费时刻、重复值身份歧义、多个候选以及重复使用同一物理事件。保持命令和未匹配事件都保留计数，不静默丢弃。

命令日志时间在 MPC 求解之前，Float64 指令没有源时间戳，因此匹配得到的是**求解前消费时刻到物理效果的合成时间区间**，包含计算、发布、桥接与物理执行；不能叫纯执行器延迟。现有记录不能唯一拆出这些成分，新增源码记录也必须与旧证据区分。

姿态部分在同一步 native 状态上比较机体上轴 `R[:,2]` 和 `a+[0,0,g]` 的归一化方向，并用世界 X/Y 投影倾角避免把有 yaw 的 Euler roll/pitch直接误当水平加速度轴。这只是推力主导近似的诊断：base_link 原点不必是质心，转动运输、机械臂内力、阻力等都会影响关系，不据此宣称唯一根因。

固定末端误差被精确分解为：机体位置偏差、在 q0 下的姿态贡献、在 R0 下的关节几何贡献、姿态与关节交互项、独立 native tip 与 FK 的闭合残差。逐项之和应重建原末端误差；不通过平移目标消除任何偏差。

每次预测使用原 `node_intervals_s` 投影到原时间轴。旧 MPC 按原角速度衰减规则重建姿态；任务包含 `prediction_chain` 的新版必须读取每次原诊断保存的 `predicted_rotations_flu_to_enu`，并校验配置一致、矩阵有限、正交且行列式为 +1，以及 `predicted_tip = predicted_p + predicted_R * r(predicted_q)`。新协议缺失姿态、姿态不合法或与 FK 不一致都会拒绝，不能静默退回旧模型外推。报告的 `attitude_source` 区分两条路径。

新版另读取原 `prediction_initial_joint_position`，在同一控制消费 `sim_s` 与 native q 比较，并并列报告原始反馈 q 的误差、传播前后 q 的差值。字段位于 `original_prediction_errors.prediction_initial_joint_position`；该时间是控制消费时刻，不是发送时刻或未来预测节点。旧版没有这个字段时明确报告 unavailable，不补零或用真值构造预测初值。该结果只衡量记录中的时间传播误差，不触发新的验收门槛或证书。

随后分解预测误差为基体位置、关节几何、姿态几何和 FK 残差。超出部分窗口的节点明确计数；不混入恢复数据。后续节点预测的未来命令会被闭环重新规划，故长时域误差并非纯粹动力学误差，各条重叠预测也不是独立统计样本。

可选 `--formal-train-audit /path/upper_stage_audit.json --formal-holdout-audit /path/upper_stage_audit.json` 读取已通过 formal response 审计绑定的原 run，另画速度指令、姿态与原始几何关系。该附加诊断不改变已经接受的参数或 HOLDOUT 身份。
