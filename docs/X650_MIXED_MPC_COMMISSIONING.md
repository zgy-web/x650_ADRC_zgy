# X650 混合阶模型接入与上层 MPC 分阶段验证

本版将已经通过新训练、独立验证的 **X/Y 二阶、Z 一阶基体速度响应模型**接入上层协同控制，使用 `precision_ev_mixed_mpc_v3` 工况和 `mixed_response_zero_ff_v1` 冻结协议。当前模型响应验收已经通过；**MPC 在新 X650＋二连杆上的末端保持、八字轨迹和性能对照仍须逐项完成真实仿真与独立审计**。实现代码、通过单元测试、预检成功均不能替代这些飞行验收。

本页记录截至 2026-10-22 的模型接入状态。后续是否实际通过，以对应新运行目录的独立证书为准。旧模型开发记录和旧 v1/v2 证书保留原有版本与含义。

## 已完成的正式响应实验

实验在服务器的 PX4 SITL＋Gazebo 物理平台运行。训练与独立验证各有一条全新 60 秒记录，使用事先固定的不同脉宽与零速度段；先完成训练和参数冻结，再启动独立验证。最后的 6 秒是位置恢复段，模型从原始初态连续预测 0–54 秒，没有逐脉冲重置、移动时间轴、拟合增益或偏置。

| 轴 | 冻结模型参数 | 训练速度 RMSE，m/s | 独立验证速度 RMSE，m/s | 独立验证最大速度误差，m/s |
| --- | --- | ---: | ---: | ---: |
| X | 自然频率 2.8471958701784543 rad/s；阻尼比 0.695123475648465 | 0.00218239 | 0.00229085 | 0.00615718 |
| Y | 自然频率 2.7956770526957433 rad/s；阻尼比 0.69189021293641 | 0.00223862 | 0.00252910 | 0.00585585 |
| Z | 时间常数 0.19588596825267549 s | 0.00243571 | 0.00252471 | 0.01002991 |

训练记录为 `runs/x650_wholebody_mpc_v1_20261022_121127_4071`，独立验证记录为 `runs/x650_wholebody_mpc_v1_20261022_121717_5093`。独立验证中三轴共 **24 个脉冲／零速度分段全部通过**，最终报告的 `complete`、`passed`、`identified` 为 `true`。上表是速度模型预测误差，不能换算成已实现的 MPC 位置跟踪精度。

服务器正式模型和证书位于：

```text
/root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/x650_formal_response_20261022/holdout_validation/response_model.json
/root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/x650_formal_response_20261022/holdout_validation/response_identification.json
```

模型原始文件 SHA-256 为 `4b96a8ec27be5278e82c7d7ee8d627f78f95c1451c1578c5e61e02930f688c53`。参数接入读取这份已绑定 JSON，不从表格中重新抄写。上述结果摘要可以在实验工作区的 `artifacts/x650_formal_response_20261022/execution_status.json` 查阅；该摘要不充当阶段证书，原始日志与证书不随源码发布。

首个启动尝试曾因慢速仿真的状态消息监督超时，在正式测量前停止；它已降落、恢复参数并关闭运行环境，失败记录仍保留。监督修复后先重新预注册，再取得上述训练与独立验证记录。协议、拟合方法和误差门槛没有借重试调整。激励和辨识方法见 [正式响应协议](X650_FORMAL_RESPONSE_PROTOCOL.md)，原始记录绘图见 [证据绘图说明](X650_FORMAL_RESPONSE_PLOTTING.md)。

## 控制范围与反馈语义

本轮 MPC 位于 PX4 的位置、速度、姿态控制环之上，联合输出三轴逻辑基体速度与两个关节速度。它使用末端几何关系、当前基体与关节状态以及已辨识的闭环速度模型预测末端运动。它没有直接输出旋翼推力或关节力矩，也没有完成浮动基体全身动力学 NMPC 的验证。

水平模型为 `v_dot = a`、`a_dot = wn² × (u − v) − 2 × zeta × wn × a`；垂向为 `v_dot = (u − v) / tau`。静态增益固定为 1，纯时延与偏置固定为 0。MPC 状态为 `[px, py, pz, vx, vy, vz, q1, q2, dq1, dq2, ax, ay]`，输入为 `[ux, uy, uz, dq1_cmd, dq2_cmd]`，采用精确零阶保持离散化。

新反馈协议为 `px4_estimator_acceleration_receipt_enu_v2`，命令接口继续使用 `raw_ev_world_pva_v2`。每次控制使用实际 PX4 估计位置、速度、姿态、角速度、位置消息中配对的地固系加速度，以及实测关节状态；先按固定 NED／ENU 约定转换，再初始化预测。加速度来自所记录的位置回调，没有用 Gazebo 真值替换，也没有把持有的旧消息当作新样本。正式响应辨识中“整条记录一次初始化且 a0=0”的约定，不用于替代 MPC 运行中的实际加速度初始化。

精密 EV 测量仍由理想 Gazebo 观测辅助 PX4 EKF，因而保留 `ground_truth_aided=true`。独立物理记录用于验收；本轮不提供 GPS 或硬件性能结论。新增审计核验加速度来源、回调时效和同负载 ULog 记录；物理加速度代理误差作为报告指标保存，不临时增加验收阈值，也不改变已通过的响应模型验收。

**两组均使用零加速度前馈。** 任务标签仍叫 `pid_ff` 和 `mpc`，但此协议明确设置 `feedforward_enabled=false`，adapter 的 `acceleration_feedforward_policy="zero"`，实际发送的加速度前馈必须为零。已有响应证据只覆盖折叠臂、零加速度前馈条件下的基体速度响应；活动关节的执行动态、机械臂惯性补偿开启后的模型适用性，仍需另行验证。

## 模型证据与条件映射

[`accepted_response_model.py`](../simulations/x650_wholebody_mpc_v1/accepted_response_model.py) 在起飞前校验：

- 模型类型、schema、三轴参数、稳定性与预先固定的参数边界；拒绝训练候选、无证书 JSON、旧 `base_time_constant_s` 模型及未批准的增益／延时／偏置。
- 最终证书绑定的模型原始字节、完整物理身份、全部 42 个 PID 参数身份、工况、训练报告、冻结候选和预注册文件。
- 当前实际 PX4 二进制与正式响应实验中的二进制一致；训练完成早于候选冻结，候选冻结早于独立验证起飞。
- 两条独立原始记录、物理审计与证书、启动任务和源码 ZIP、原始日志及参数恢复文件的哈希链。

历史控制／平台源码只有在预注册 ZIP 中保留完全一致的原路径和 SHA-256 时，才可以用归档字节核验其历史身份。分析源码、原始测量和报告不因此获得替换或改写许可。v3 的每次启动另外归档模型、最终证书、预注册、预注册源码 ZIP、冻结候选和训练报告这六项原始输入；独立审计保存自己的 `audit_sources.zip`。

旧证据通过明确角色进入新流程：v1 的地面、悬停、关节证据仅作为 `preparation_only`；正式 v2 响应证据仅作为 `nominal_response_only`。它们不会被改写成 v3，也不会自动证明 MPC 或摆臂条件通过。v3 后续阶段要求同协议、同模型、同 PID、同 EV 条件的独立证书。

## 冻结配置与阶段时序

完整配置由 [`mixed_mpc_protocol.py`](../simulations/x650_wholebody_mpc_v1/mixed_mpc_protocol.py) 生成，不接受自由填写的 `mpc_config`。协议规范 JSON 的 SHA-256 为：

```text
52f4758648836eb6654554d85201fb447132869b53392bb23ed90a08c89920d9
```

| 项目 | 固定值 |
| --- | --- |
| 控制周期 | 0.02 s |
| MPC 节点 | 15；首段使用实际控制间隔，随后 14 段各 0.04 s；典型预测时长 0.58 s |
| 姿态角速度外推衰减时间 | 0.15 s |
| 目标权重：末端位置／速度 | 400／4 |
| 目标权重：基体位置／关节姿态 | 0.25／0.02 |
| 目标权重：名义输入／输入变化 | 0.01／0.01 |
| 最大迭代次数／求解 ftol | 100／1e-9 |
| 原始约束违反容差／KKT 容差 | 2e-7／2e-3 |
| 求解预算／失败策略 | 20 ms／抛出失败并停止；不沿用旧计划回退 |
| 名义机械臂贡献 | 0.2；名义飞机贡献 0.8；实际纠偏仍由反馈分配 |
| 基体速度限值 X/Y/Z | 0.55／0.35／0.45 m/s |
| 关节速度限值 1/2 | 0.45／0.35 rad/s |
| 基体指令变化率 X/Y/Z | 0.45／0.30／0.45 m/s² |
| 关节指令变化率 1/2 | 0.60／0.60 rad/s² |
| 关节软角度范围 | q1：−0.75～0.45 rad；q2：−0.35～0.85 rad |

两组共享相同的名义分配、限幅与关节软边界，以及固定 PID 实际参数。完整公共参数还包括关节位置增益 3、末端误差增益 2、最大末端速度修正 0.20 m/s 等，均包含在协议哈希中。修改任何数值需要新的评审版本和新实验，不能在验证后覆盖当前 preset。

| 阶段 | 参考轨迹 | 完整测量时长 |
| --- | --- | ---: |
| `tip_hold` | 固定世界坐标系下的末端位置；本阶段不施加外部扰动力 | 30 s |
| `small` | X±0.075 m、Z±0.03 m，16 s/圈，两圈 | 40 s |
| `normal` | X±0.25 m、Z±0.10 m，12 s/圈，两圈 | 32 s |
| `fast` | X±0.25 m、Z±0.10 m，8 s/圈，两圈 | 24 s |

八字为 XZ 平面参考，开始时间为 2 s，使用 2 s 平滑起止过渡。表中的测量时长还包括静止／回稳段；审计覆盖整个阶段，不只截取轨迹看起来较好的部分。当前安排先完成 `tip_hold`、`small`、`normal`，`fast` 保留为后续独立阶段，不自动连跑。

验收固定要求末端三维误差 RMS ≤0.05 m、峰值 ≤0.10 m、倾角 ≤8°，并保留完整 native／ULog 采样、实际 PVA 收发、坐标与时效、地面与关节物理限位、降落解除武装和参数恢复检查。MPC 每条求解记录必须 `accepted_optimum=true`、`previous_plan_fallback=false`、原始约束违反 ≤2e-7、KKT 指标 ≤2e-3，而且不能超过 20 ms 求解预算。末端误差根据原始时间和预先声明的参考独立计算，不拟合常数偏差或轨迹相位。

## 逐项推进的证书要求

每一次运行都先要求 `native_ground`、`hover_folded`、`joint_probe` 和 `response_identification` 四项基础证据。推荐执行顺序如下，每行只在上一行完成独立审计后单独启动：

| 顺序 | 本次运行 | 新增前置证书 |
| --- | --- | --- |
| 1 | PID `tip_hold` | 无；使用四项基础证据 |
| 2 | MPC `tip_hold` | PID `tip_hold` 的 `pid_baseline.json` |
| 3 | PID `small` | 已通过的同协议 `tip_hold`；可用上一步 MPC 证书 |
| 4 | MPC `small` | 同协议 `tip_hold`，以及 PID `small` 的 `pid_baseline.json` |
| 5 | PID `normal` | 同协议 `tip_hold` 和 PID `small` 的 `research_stage.json` |
| 6 | MPC `normal` | 同协议 `tip_hold`、MPC `small` 的 `research_stage.json`，以及 PID `normal` 基线 |
| 后续 | PID／MPC `fast` | 同控制器的 `small`、`normal`；MPC 另外要求 PID `fast` 基线 |

`pid_baseline` 必须写明 `controller="pid_ff"`、`source_stage` 等于本次阶段，并且协议哈希相同；普通八字基线不能替代快速八字基线。`small` 前置的末端保持可以来自任一已通过的同协议控制器；`normal`／`fast` 的前置轨迹证书必须与本次控制器相同。

## 任务生成和单次运行命令

以下从服务器仓库根目录执行，使用现有环境，不重新安装依赖。`FORMAL_TASK` 应指向已接受的正式响应运行所归档的原任务，借用它的模型准备、PID、世界与物理准备证据路径；不会修改原任务。`EXTRA_GATES_JSON` 如有值，应指向只包含本次额外阶段证书映射的 JSON，例如 `{"pid_baseline":"/absolute/pid_tip_hold/audit/pid_baseline.json"}`。所有证书必须实际存在且通过哈希检查。

```bash
export FORMAL_TASK="$PWD/runs/x650_wholebody_mpc_v1_20261022_121717_5093/controller_task.json"
export RESPONSE_ROOT="$PWD/artifacts/x650_formal_response_20261022/holdout_validation"
export STAGE=tip_hold
export CONTROLLER=pid_ff
export TASK_OUT="$PWD/artifacts/x650_mixed_mpc_20261022/tasks/pid_tip_hold.json"
export EXTRA_GATES_JSON=""

PYTHONPATH="$PWD/src:$PWD/simulations/x650_wholebody_mpc_v1" /usr/bin/python3 - <<'PY'
import json, os
from pathlib import Path
from task import build_task_config, validate_task_config
from mixed_mpc_protocol import build_mixed_mpc_protocol

seed = json.loads(Path(os.environ['FORMAL_TASK']).read_text())
cfg = build_task_config(os.environ['STAGE'], os.environ['CONTROLLER'],
                        mixed_mpc_protocol=build_mixed_mpc_protocol())
for key in ('preparation_report', 'airframe_source', 'native_recorder_library',
            'pid_profile', 'hover_reference_profile'):
    cfg[key] = seed[key]
cfg['research_profile'] = {'name': 'precision_ev_mixed_mpc_v3',
                           'world_sdf': seed['research_profile']['world_sdf']}
cfg['response_model'] = str(Path(os.environ['RESPONSE_ROOT'])/'response_model.json')
cfg['stage_evidence'] = {name: seed['stage_evidence'][name]
                         for name in ('native_ground', 'hover_folded', 'joint_probe')}
cfg['stage_evidence']['response_identification'] = str(
    Path(os.environ['RESPONSE_ROOT'])/'response_identification.json')
extra = os.environ.get('EXTRA_GATES_JSON')
if extra:
    cfg['stage_evidence'].update(json.loads(Path(extra).read_text()))
cfg = validate_task_config(cfg)
output = Path(os.environ['TASK_OUT'])
output.parent.mkdir(parents=True, exist_ok=True)
with output.open('x', encoding='utf-8') as stream:
    json.dump(cfg, stream, indent=2, allow_nan=False)
    stream.write('\n')
print(output)
PY
```

先做无飞行预检：

```bash
PYTHONPATH="$PWD/src" /usr/bin/python3 simulations/x650_wholebody_mpc_v1/launch.py \
  --task "$TASK_OUT" --dry-run
```

检查输出中的 **`preflight.ready` 必须为 `true`**；`--dry-run` 本身返回 0 不代表预检通过。缺少前置证书时先完成对应阶段，不填写假的成功标志。正式运行建议在已有 `screen` 后台会话中执行以下单次命令；每次使用新的输出目录：

```bash
env -u PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 scripts/run_x650_mixed_mpc_stage.py \
  --task "$TASK_OUT" \
  --output /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/x650_mixed_mpc_20261022/pid_tip_hold_job \
  --launch-python /root/autodl-tmp/amsim-venv/bin/python3 \
  --analysis-python /usr/bin/python3 \
  --pyulog-site /root/autodl-tmp/amsim-venv/lib/python3.10/site-packages
```

分析进程使用系统 NumPy／SciPy；`--pyulog-site` 只将服务器已有 PyULog 所在目录追加到分析搜索路径末尾，避免替换系统数值库。路径须按现有环境核实。飞行进程使用原仿真虚拟环境，并将数值库线程数限制为 1。

driver 每次只飞一个任务，要求恰好产生一个独立 run 与一个 launch，确认降落、运行环境关闭、参数恢复后，再执行独立审计。失败、运行身份歧义、证据变化或审计不通过都会停止，不自动重飞、不调参数、不推进下一阶段。输出目录已存在时直接拒绝覆盖。

## 输出与独立审计

单次 job 保存 `task.json`、`job_status.json`、`launch.log`、`audit.log` 和 `audit/`。物理与飞控原始文件仍位于独立 `runs/` 目录，启动目录保留控制源码 ZIP 和六项响应证据快照。起飞前执行完整模型证据校验，进入控制循环后只检查小型模型文件及实际 PID，避免在 Offboard 发送环中重复散列大型原始日志。

正常成功的 `audit/` 包括：

- `mixed_mpc_stage_audit.json`：完整独立审计与末端误差、求解时间／质量、反馈来源指标。
- `research_stage.json`：当前阶段的成功证书。
- `research_ground.json`：本次地面检查记录；新 v3 证书不会替换任务要求的既有 v1 物理准备角色。
- `pid_baseline.json`：只在 PID 组成功时生成，绑定当前 `source_stage`。
- `audit_sources.zip`：审计依赖源码的不可变快照。

若需对一个已经完成的原始 run 单独审计，应使用新的审计输出目录：

```bash
env -u PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 scripts/audit_x650_mixed_stage.py \
  --run-dir /absolute/runs/x650_wholebody_mpc_v1_TIMESTAMP \
  --launch-dir /absolute/runs/x650_wb_launch_TIMESTAMP \
  --output /absolute/new_independent_audit
```

该直接命令要求分析解释器已经能导入 PyULog；否则优先使用上面的单次 driver 及其现有包目录参数。原 `stage_result.json`、`controller_metadata.json` 是运行记录，保留 `eligible_as_stage_gate=false` 和待审计语义，不会被成功证书覆盖。

完成 PID 与 MPC 的同阶段独立验收后，才比较同参考、同物理模型、同 PID 和零加速度前馈条件下的末端误差、机体运动、关节运动及求解耗时。此时可以报告该阶段的实际 MPC 结果；在此之前，本版只表示模型接入和实验门槛已准备好。
