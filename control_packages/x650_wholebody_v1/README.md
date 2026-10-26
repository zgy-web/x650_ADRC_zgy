# X650 + 二连杆：PID/前馈与上层全身 MPC，0.1.0.dev0

本包提供 X650 组合模型的显式运动学/质量参数、PID 协同基线、上层全身 MPC，以及共用的前馈与 PX4 参考编码。后续 `precision_ev_mixed_mpc_v3` 协议已在服务器完成保持、小幅与普通八字，见 [阶段结果](../../docs/X650_FIGURE8_RESULTS_20261022.md)；这不表示真机或严格 50 Hz 整环已验收。PX4 仍执行级联 PID，历史实验机械臂使用 Gazebo 关节速度接口。

本 README 中 `precision_ev_upper_v2` 的接口及工况限制保留为历史版本说明，不能替代 v3 的任务门禁。新 `ts20.py` 是独立的离线 MIT 参考适配和外部模型加载模块，详见 [TS20 接入](../../docs/X650_TS20_INTEGRATION.md)；它不自动替换已冻结 MPC 的关节预测或 Gazebo 执行器。真机前和 NMPC 路线见 [验证框架](../../docs/X650_REAL_FLIGHT_READINESS_AND_NMPC.md)。

TS20 新工况由显式的 `Ts20MpcController`、`Ts20PredictionModel` 和 `Ts20Runtime` 接入，[独立场景](../../simulations/x650_ts20_hold_v1/README.md)管理新的悬停、摆臂和末端保持。该入口使用实际积分参考、实际 MIT 包与测量年龄，名义关节响应保留静摩擦和换向；原生数值核只作计算加速。关节局部信赖域上限 0.02，发布前检查完整非线性候选包络，最多 3 次局部 QP 并限制总计算 50 ms；外层还要检查适配器及首条发布前的完整耗时。它仍是上层速度接口，机体响应使用旧先验，不能称为已验证的 24 V 带载全身动力学 NMPC。

完整 C++ 在线 Core 的 ABI 2 增加了 [可信中心重建与重新线性化恢复](../../docs/X650_MPC_TRUSTED_CENTRE_RECOVERY.md)。正式 QP 仍保留 0.02 半径及原约束，内部中心通过检查后才允许重新求解；原失败快照与实际 ROS 工作进程回放已通过。这个分支位于 `native/mpc_core.cpp`，没有同步修改历史 Python `Ts20MpcController`。新后端的受扰物理闭环需另行验收，不能把离线恢复称为完整飞行通过。

当前源码进一步升级到 **Core ABI 3**，新增[节点间硬约束与指令编码余项恢复](../../docs/X650_MPC_INTERVAL_HARD_CONSTRAINTS.md)。不增加松弛变量，不放宽 ±0.42／±0.32 rad/s 内界；429 条固定原输入及匹配 ROS 接口测试通过。必须重新编译匹配 worker/header/core，并在新 startup 绑定 `canonical_interval.hpp`、`joint_safety.hpp` 等实际源码；旧 ABI 2 的飞行资格不适用于新库。该分支仍仅位于 C++，历史 Python 控制器没有这项区间恢复实现。软件结果见[本次验证记录](../../docs/validation/cpp_interval_hard_20261026/README.md)。

旧 `control_packages/pid_ff_wholebody_v1` 的冻结参数包和历史实验保持独立。旧 x500 的 PID 选择结果不能作为新 X650 + 臂已经充分整定的依据；本包要求调用者传入当前实验实际读回的 PX4 增益。

## 模型与坐标

`ArmModel.from_report(report_path)` 读取组合模型准备报告，并校验其 URDF、运动学、机身/桨盘净距、自碰撞等来源文件的哈希与通过状态。该报告是几何与质量的接入依据，不是一次成功飞行的证明。实际尺寸与质量来自原验证模型，不使用可编辑示例模板的默认值。

本包只分发源码、接口和合成测试，不包含私有 X650 CAD、URDF/STL、生成模型、准备报告或实验数据。报告及其相邻 `robot.urdf`、`kinematics.json`、净距/自碰撞证书和 mesh 必须由使用者从外部模型准备链提供。X650 预设绑定特定源 SHA256、安装构型、物理模型和 PX4 参数；通用 URDF 导入支持另一个模型，并不意味着另一个模型可以直接通过本包的 X650 证据门禁。输入准备见 [场景 README](../../simulations/x650_wholebody_mpc_v1/README.md)。

- 世界位置/线速度：ENU，m / m/s；`rotation`：机体 FLU 到 ENU 的旋转矩阵。
- `omega_body`：机体 FLU 角速度，rad/s；关节 q / dq：相对折叠零位的 rad / rad/s。
- 原二连杆：0.30/0.25 m、0.25/0.20 kg，第一杆零位 150 度，两个关节轴沿 +Y。
- 当前 v2 组合模型：无臂 X650 3.02 kg + 两杆 0.45 kg + 假设安装座 0.025 kg = 3.495 kg。安装根坐标约为 FLU `[0,0,-0.18]` m；第二杆沿共同关节轴 +Y 错层 0.04 m，消除原理想方杆同平面自穿。偏移进入正运动学、质心和惯量计算。
- 动力学前馈使用新组合模型的总质量和机体原点/质心偏移。关节和机械臂重量不重复叠加到名义重力补偿。

## 默认 MPC 配置

| 项目 | 当前实现 |
|---|---|
| 状态 | `[p_W(3), v_W(3), q(2), dq(2)]` |
| 优化输入 | 三维机体目标执行速度与两关节速度，共 5 维 |
| 节点 | 15 个，75 个凝聚优化变量 |
| 节点时间 | 首段为调用者传入的实际 `dt`，后续 14 段为 0.04 s |
| 实际预测时域 | `dt + 14 × 0.04 s`，例如 dt=0.02 时为 0.58 s |
| 基座响应 | 精确离散的一阶闭环速度响应，时间常数 `[0.20,0.20,0.25]` s；**未辨识工程初值** |
| 关节响应 | 当前按理想速度执行，时间常数 0；原生响应仍需测量 |
| 姿态 | 以实测 R 和机体角速度作衰减外推，角速度衰减时间常数 0.15 s；姿态不是自由优化输入 |
| 求解器 | SciPy SLSQP；精确 Cholesky Hessian 白化、解析目标梯度、线性约束；每次只线性化一次 |
| 最大迭代 / ftol | 100 / 1e-9 |
| 验收 | primal violation ≤ 2e-7，KKT stationarity infinity norm ≤ 2e-3，同时要求求解器报告成功 |
| 速度限幅 | 飞机 `[0.55,0.35,0.45]` m/s；关节 `[0.45,0.35]` rad/s |
| 指令变化率限幅 | 飞机 `[0.45,0.30,0.45]` m/s²；关节 `[0.60,0.60]` rad/s² |
| 关节软界 | 下界 `[-0.75,-0.35]`、上界 `[0.45,0.85]` rad，位于实际物理限位内 |
| 终端余量 | `q_N + (vmax/(2*amax)+max_dt)*dq_N` 位于软界内；是保守线性制动余量，不是鲁棒不变集证明 |
| 名义分配 | 同基线，机械臂 20%；实时纠偏不固定按该比例分摊 |
| 权重 | 末端位置 400、末端速度 4、基座位置 0.25、关节姿态 0.02、名义输入 0.01、输入变化 0.01 |
| 默认失败策略 | 抛出带诊断的 `MpcSolveError`，交上层停止任务和正常降落 |
| 可选失败策略 | `failure_policy="shift"`：只有上一条移位序列重新通过当前约束检查后才使用，明确记录 fallback，不记为本次最优解 |
| 时间/延迟状态 | 不自动补偿未测量的状态/命令延迟；调用者需声明每项反馈的时钟、来源与异步保持协议，不能把最新消息拼接称为采样同步 |

位置预测沿前次移位轨迹线性化 `p_tip=p+R*r(q)`；速度预测使用各节点冻结的关节雅可比及姿态转动项。`accepted_optimum` 只针对本次凸 QP 的质量检查，不代表原非线性整机问题的全局最优。

`solve_budget_ms=20` **目前只产生超预算诊断，不会中断求解器，也不会自动拒绝所有耗时超过 20 ms 的可行解**。外层 runner 必须测量完整计算时间及仿真时钟推进，决定返回值是否仍及时。`realtime_verified` 默认且当前仍为 false。

## 可调用接口

从项目根目录将本目录加入 `PYTHONPATH`，使用服务器已有 NumPy/SciPy 环境即可；不需要安装或重新下载依赖。下例是调用接口，不启动仿真：

本地验证环境是 Python 3.11、NumPy 2.0.1、SciPy 1.17.0。新容器现成的系统栈 NumPy 1.21.5 / SciPy 1.8 也已实际通过同一控制包 40 项测试，因此 `pyproject.toml` 最低版本声明同步为这组实测版本。本模块不读取新版 SLSQP 才提供的 multipliers，KKT 由独立 `nnls` 估计。服务器采用在单次进程中把 `/usr/lib/python3/dist-packages` 加入 `PYTHONPATH` 的方式，优先复用匹配的系统 NumPy/SciPy，同时保留 amsim-venv 的其他依赖；没有全局改写 venv，也不需要下载。服务器上的实际闭环时延仍需测量，单元通过不等于原生飞行通过。

```python
from x650_wholebody.api import ControlState, EndpointTarget
from x650_wholebody.model import ArmModel
from x650_wholebody.mpc import MpcController
from x650_wholebody.adapter import ControllerAdapter

def make_controller(report_path, verified_pid_gains, p0, R0, q0, trajectory):
    model = ArmModel.from_report(report_path)
    allocator = MpcController(model, verified_pid_gains, trajectory=trajectory)
    controller = ControllerAdapter(allocator, model, verified_pid_gains)
    controller.reset(p0, R0, q0)
    return controller

# 在真实仿真循环中，每步重新读取实际状态，并传入真实控制间隔：
# target = EndpointTarget(*trajectory.sample(task_t))
# command = controller.step(task_t, actual_dt, measured_state, target)
# 编码必须与场景反馈协议配套：旧入口使用 command.px4_setpoint(...)；
# precision_ev_upper_v2 使用下文冻结原点的 encode_world_command(...)。
# 同时发送 px4.position / velocity / acceleration，type_mask=2048。
# 两关节直接发送 command.joint_velocity，不再加关节位置 P。
```

`trajectory.sample(t)` 应返回 `(position, velocity, acceleration)`；也支持 `.preview(t)` 或可调用对象返回同样三元组或 `EndpointTarget`。采样时间是同一个任务相对仿真时间。没有提供轨迹源时，只能对当前 target 作常加速度外推，诊断会明确标记，不能称为完整已知八字预览。

低层 `MpcController.reset(p0,R0,q0)` / `step(t,dt,state,target)` 返回与 `BaselineController` 相同的分配 dict。`ControllerAdapter` 为两组共用：维护实测角加速度滤波，计算机械臂相对运动的平动惯性前馈，合成加速度并作相同限幅，然后根据实际 PX4 位置 P 增益构造误差一致的 p/v/a 参考。反作用力矩只记录，当前没有关节力矩前馈或机体力矩注入。

`ControlCommand` 保存计算当时的反馈快照。上面的 MPC 构造例是数值接口示意，不是当前精密 v2 允许执行的飞行任务。旧 `command.px4_setpoint(...)` 会依据传入的 PX4 估计量保留状态快照的误差；它不是 v2 的编码器。不能将旧 command 与新 PX4 状态反复组合，声称执行了新的上层反馈计算。

## 精密 v2 的估计反馈与直接 P/V/A 编码

场景层 [estimated_feedback.py](../../simulations/x650_wholebody_mpc_v1/estimated_feedback.py)负责 PX4 输入与冻结世界原点；控制包保持纯数值接口。ApiNode 默认订阅不变，显式 `enable_estimator_feedback()` 后增加姿态、角速度、融合标志及 `/clock`。下面只展示已审核 PID 场景中的调用关系，不负责启动、起飞或阶段放行：

```python
from simulations.x650_wholebody_mpc_v1.estimated_feedback import (
    Px4EstimatedFeedback, encode_world_command,
)

# reference 已在实际 raw_ev_world 条件下绑定；先等待全部输入有效。
feedback = Px4EstimatedFeedback(reference, node)
state, joint_stamp_s, snapshot = feedback.read(node, node.estimator_clock_s)
command = pid_controller.step(task_t, actual_dt, state, target)
px4 = encode_world_command(command, reference)
# 发送 P/V/A 及 command.joint_velocity，并归档 snapshot 与实际发送命令。
```

机体位置、线速度、姿态和角速度均取自 PX4 估计；关节 q/dq 取自测量。世界 ENU 位置由 `[local_y,local_x,frozen_ref_alt-local_z]` 得到，旋转采用 `C·R(q)·D`，其中 `C` 为 NED→ENU、`D=diag(1,-1,-1)` 为 FRD/FLU 转换。状态取最新异步回调并检查接收时龄，原点/重置/有效性/融合标志改变即拒绝，不回退到 Gazebo truth。

`state.stamp_s` 固定为角速度回调对应的 `/clock` 接收时刻，保持的旧角速度不会因另一 topic 更新而被重复求导。位置/姿态/角速度限 0.10 仿真秒，关节 0.25 秒，flags 1.25 秒；flags 墙钟限 10 秒、其他输入 4 秒。flags 按固定 PX4 每 1 秒或状态变化的源发布周期设限，DDS 5 Hz 限速不等于源固定 5 Hz。该接收时龄协议不是跨 topic 采样同步或延迟补偿。

命令由冻结原点直接变换：`p_NED=[p_y,p_x,ref_alt-p_z]`，速度前馈和加速度只做 ENU→NED 旋转；不再读取最新估计误差平移参考。快照允许独立重建 `u=K(p_ref-p_est)+v_ff`，PX4 实际接收/执行另由完整 ULog 验证。EV 本身来自理想 Gazebo 测量，所以整个工况仍标记 `ground_truth_aided=true`，不能宣称脱离理想定位或实机精度已验证。

v2 暂时只用于 PID 响应辨识与无外力注入的末端定点。每次 raw runner 结果保留 provisional；事后 native/ULog/反馈/命令/参数复原审计通过才另发证书。操作条件绑定完整 42 项实际 MC/MPC 参数、模型、世界、EV 和源码，不能把旧 GPS、真值直接反馈或不同 PID 的响应模型混入这一协议。

## 50 Hz sender 的接入顺序

初次原生接入优先采用单一仿真时间轴、同步求解与发送，保留 0.02 s 目标周期，并完整记录实际间隔。建议先在进程导入 NumPy/SciPy 前设置：

```bash
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
```

这只请求数值库线程数，不证明服务器后端一定采用该设置。先在地面完成原生回调、计算和 `/clock` 推进测量；若每 20 ms 仿真内没有足够墙钟预算，可以为两组使用相同的较低 Gazebo `real_time_factor`，保持物理积分步长与任务仿真时间不变，并记录世界文件哈希和实际 RTF。假设实测 RTF=0.20，则每 20 ms 仿真约有 100 ms 墙钟预算；这是测量和配置依据，不能据此跳过原生时间验证。起飞、稳定门槛等墙钟超时也需按实测 RTF 调整。

native runner 至少记录每步状态/关节时间戳、读取时刻、计算开始/结束、发送时刻、`control_dt_s`、`solve_ms`、总 `compute_ms`、返回后状态年龄、漏发次数及失败/fallback。已有 `max_dt_s=0.05` 只是异常门槛，满足该门槛不等于严格 50 Hz 已验证。

后续若拆为独立 solver 与 50 Hz sender，需要先实现以下配套，再用于正式对比：

1. solver 只接收最新且完整的状态快照，队列不堆积；输出带绝对仿真执行时刻、所用状态时间戳和序列编号的计划。
2. sender 依据当前仿真时刻执行已验证的计划，读取新反馈并重新编码 PX4 参考、共用前馈；过期计划不当作当前求解结果。
3. 已发送但尚未执行的命令进入预测首段，发送后实际使用的输入回灌 planner，保持 warm start 与执行一致。
4. 每个真实发送间隔都满足相同的指令变化率。不能直接把 20 Hz 的 `step(dt=0.05)` 结果在 50 Hz 发送端作大幅跳变，也不能把当前 0.04 s 的计划节点当成 0.02 s 节点执行。若引入插值/限速器，预测模型必须包含同样的执行规则，不能只在结果后静默平滑。
5. 持续发送 Offboard 心跳与“继续执行过期 MPC 任务”需要分开处理；连续过期/不可行时停止任务并正常降落，明确记录。

当前代码没有该异步执行器、延迟队列或硬截止取消机制，因此先进行同步低 RTF 的接入验证更直接。

## 离线测试与可复现计时

```bash
python -m unittest discover -s control_packages/x650_wholebody_v1/tests -v

python control_packages/x650_wholebody_v1/tests/benchmark_timing.py \
  --report generated/x650_2link_v2_20261021/preparation_report.json \
  --steps 100 --nodes 15 --blas-threads 1 \
  --output generated/x650_mpc_timing_new_run.json
```

计时程序拒绝覆盖已有输出，记录实际模型报告/控制源码哈希、Python/NumPy/SciPy 版本、请求的 BLAS 线程环境，以及 p50/p95/max。它用同一工程响应模型推进输入状态来覆盖 warm start，**只用于计算耗时，不能证明独立跟踪性能、物理稳定性或 PX4/Gazebo 飞行成功**。

2026-10-21 的本地 100 步、默认 75 变量、哈希绑定组合模型计时：

| 条件 | solve p50 / p95 / max，ms | 总 compute p50 / p95 / max，ms |
|---|---|---|
| 默认线程环境 | 7.45 / 15.64 / 24.09 | 26.63 / 42.86 / 65.73 |
| 新进程请求 1 个 BLAS 线程 | 3.36 / 7.64 / 35.29 | 18.36 / 24.47 / 52.82 |

两次均为每步 2 次 SLSQP 迭代，primal violation < 3e-16、KKT stationarity < 5e-16。运行先后、机器负载和后端调度影响尾部，不能把这两次测量当成受控硬件性能统计；全部时间也尚未包含 ROS/MAVLink/日志与前馈开销。单线程请求减少了本次多数开销，20 ms 仍未逐次满足。

目前测试覆盖状态/坐标接口、独立运动学导数、组合模型哈希门禁、旧数值行为兼容、共用前馈、实际 PID 增益编码、有限时域预览、真实 dt 指令变化约束、预测关节边界、QP 梯度、失败与显式 fallback。正式实验还需要新机原生响应辨识、带臂悬停及补偿检查，并在相同 PID/模型/前馈/时钟设置下对照普通和快速八字。完整设计见项目 `docs/X650_MPC_ARCHITECTURE.md`。

本地最近一次完整控制包单元检查为 40 项全部通过；这些检查没有启动原生仿真。

## 原生响应辨识与独立留出验证

`scripts/identify_x650_upper_response.py` 为本包生成可由 runner 读取的 `response_model.json`。必须先完成两次独立的 `response_probe`：一次 train，一次 holdout。源码与合成数据测试不提供已标定模型；应以本次完整飞行及独立审计的输出为准。

```bash
# 使用已经完成起飞、probe、降落和解除武装的两次独立 native run。
# 替换为实际运行目录；output 必须不存在。
PYTHONPATH=/usr/lib/python3/dist-packages \
  /root/autodl-tmp/amsim-venv/bin/python3 scripts/identify_x650_upper_response.py \
  --train /absolute/train/run \
  --holdout /absolute/holdout/run \
  --train-audit /absolute/train/audit/upper_stage_audit.json \
  --holdout-audit /absolute/holdout/audit/upper_stage_audit.json \
  --output generated/x650_upper_response_new_pair
```

上面的命令用于精密 v2，两份 `--*-audit` 必须同时提供，且分别来自对应独立 run 的 `audit_x650_upper_stage.py` 完整验收。它校验 PX4 估计反馈快照与实际接收 P/V/A，模型输出观测采用独立 native 物理速度；同时检查估计速度是否可以作为该物理速度的代理。响应段加速度前馈必须为零，此模型不标定主动前馈工况。

每轴使用 task t=2..6、6..10、10..14 s 的既定四秒段。只用 train 拟合正时间常数，静态增益固定为 1、偏置固定为 0、输出模型延迟固定为 0。使用原始逻辑发令仿真时间和独立 native 物理时间，以事件分段的精确一阶传播处理不等采样间隔。每段只用首个真实速度初始化一次，之后自由预测，不逐点重置状态，也不事后平移轨迹。脚本保留无 `--*-audit` 的旧协议读取路径，该路径的历史 `control_trace`/`truth_stamp_s` 结果不能作为 v2 的替代证据。

holdout 从不改写 tau、增益、偏置或相位。相同目录、相同 CSV 哈希，以及仅改变换行/绝对时间但具有相同规范化测量内容的数据，均不能作为独立验证。两次 stage 必须完整落地，全部列出原始文件的哈希一致可查，模型、URDF、airframe、flight profile、preparation 与实际 MC/MPC 参数哈希一致，且每次飞行前后实际 MC/MPC 值一致。读取文件名为原生 runner 实际输出的 `stage_result.json`、其指定 metadata、`control_trace.csv`、`px4_parameters_before.json` 和 `px4_parameters_after.json`。

以下是预先固定、随脚本源码哈希写入报告的**工程 acceptance**，不证明真实系统恰好一阶，也不证明该模型或时间常数为所有工况的最优选择：

- 每轴至少 120 个唯一真值样本，样本最大间隔不大于 0.06 s，完整覆盖声明的四秒段；正/负有效脉冲各至少 0.6 s，幅值 XY 至少 0.04 m/s、Z 至少 0.025 m/s；真实速度跨度至少为输入跨度的 50%。
- tau 搜索范围 0.01..1.5 s，不能贴搜索边界，且把 tau 改为一半/两倍后的预测变化须足够，避免无有效动态激励却得到一个数值。
- train 与 holdout 的自由预测 RMSE 均不大于 XY 0.015 m/s、Z 0.010 m/s，按实际命令半幅归一的 RMSE 不大于 0.20；另检查最大误差、持续偏差、响应峰值、跨轴运动，以及机械臂是否保持折叠配置。
- 仅作拒绝诊断：固定 train 得到的 tau，在原时间轴上模拟 0..120 ms 命令延迟，不移动观测。若大于 30 ms 的延迟使 holdout RMSE 下降至少 20% 且至少 0.001 m/s，则拒绝当前零延迟模型；该诊断不会把延迟或移相偷偷写入已接受模型。

脚本保存 `response_model.json` 和可用于 stage gate 的 `response_identification.json`。只有完整独立验收通过时，`identified` / `passed` 才为 true，CLI 退出 0。输入证据不完整、激励不足或一阶模型不合适时，仍保存明确失败原因，标为 false 并退出 2；应修改实验或升级响应模型，不能放宽阈值掩盖结果。输出目录拒绝覆盖。

MPC runner 使用前还需把 `pid_parameters_sha256` 与当前实际 MC/MPC 读回值的规范化哈希比较，并校验 response stage evidence 绑定的模型文件哈希就是配置中的 `response_model.json`。通过仅表示该台带臂机在折叠、小幅速度激励、同一 PID 下，指定零延迟一阶近似满足此次工程容差；机械臂执行响应、角动态、快速八字和实时性均是另行验证项。

## PID 工程基线冻结

本节保留后续 MPC/八字的工程基线路线；当前精密 v2 仅允许 response/tip，不因这段说明自动开放 small 或 MPC。精密工况的 42 项参数冻结与恢复由场景的研究协议核验，下面的旧工程冻结器不替代该证据契约。

`scripts/freeze_x650_pid_baseline.py` 可以从成功的原生 hover、joint_probe、PID+FF tip_hold、PID+FF small 及两次独立 response_probe 中冻结全部已捕获的实际 MC/MPC，并要求位置/速度/姿态/角速度 26 个增益名称完整；这不是全量 PX4 参数导出，数值也不抄旧机。它重查原始哈希、参数前后/跨次一致性、独立 native 采样和实测误差，不把默认增益复制成“最优 PID”。

默认工程验收通过时，可以输出 `passed=true` 的 `pid_baseline` 证据，用于上层 MPC 前置调试，同时明确 `tuning_claim=false`、`baseline_scope="engineering_stability_only"`、`formal_fully_tuned_comparison_ready=false`。正式声称 MPC 优于充分整定 PID，需要另外完成预声明候选比较和独立验证，当前冻结不会替代该任务。

预声明容差为 hover 位置 RMS/峰值 0.10/0.20 m、tip_hold 末端 0.05/0.10 m、small 末端 0.04/0.10 m，最大倾角 8°，并保留原关节/指令/采样边界。小八字静止在中心的理论 RMS 约 57 mm，因此 40 mm 至少能排除完全未追踪；这些仍是调试可用性指标，不是精密或最优性证明。完整命令、证据字段、严格模式、当前只读参数接口缺口和后续原生整定矩阵见项目 `docs/X650_PID_BASELINE.md`。
