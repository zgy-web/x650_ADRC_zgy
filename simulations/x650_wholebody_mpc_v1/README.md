# X650 + 二连杆：原生接入与分阶段 MPC 八字实验

本目录提供服务器上现有 **Gazebo / PX4 SITL / ROS 2** 平台的执行入口。新机体是导入的 X650 URDF，组合模型为 `amsim_urdf_x650_2link_v2_flight`，PX4 airframe 为 `4998`。原二连杆保留长度、质量、关节轴和折叠零位；第二杆沿共同关节轴 +Y 错层 40 mm，消除原理想方杆的同平面自穿。完整质量为 3.495 kg，包含显式假设的 25 g 安装座。

本文记录代码已经实现的执行流程和证据契约。**入口、离线数值测试、mock 生命周期通过，都不等于原生悬停或 MPC 八字已经通过。** 本页不替代某一次服务器运行的 `result.json`、`controller_metadata.json`、原生记录及通过证据。

当前 MPC 是保留 PX4 级联 PID 和 Gazebo 关节速度执行器的**上层全身运动学 MPC**：同时优化机体三维目标执行速度与两关节速度，预测末端位置和闭环基体响应。它没有直接优化各桨推力或关节力矩，尚未实现后续全身动力学、推力/力矩级 NMPC。设计见 [算法范围](../../docs/X650_MPC_ARCHITECTURE.md) 和 [控制包说明](../../control_packages/x650_wholebody_v1/README.md)。

## 源码与外部模型输入

本目录随源码发布的是场景、控制接口、参数事务、记录/分析工具和合成测试。用户提供的 CAD、原始/派生 URDF、STL/DAE/OBJ、资源压缩包、生成 SDF、准备报告、视频、ULog、rosbag 和原始遥测不包含在这份源码中。`generated/`、`runs/`、`artifacts/` 路径仅表示实验者本地的输入/输出布局。

平台的通用 [URDF 导入接口](../../docs/URDF_IMPORT.md)支持外部模型；本 X650 预设进一步约束具体来源与物理构型：

1. 为 [机架准备脚本](../../scripts/prepare_x650_flight.py)提供外部源 `robot.urdf` 及全部相对路径 mesh。脚本要求源 SHA256 为 `643f5c1d8a57eeffa822b044cba7b9ec6b1a0d72186b8e524fb0d081be3943bb`，与审核原件不符即拒绝。
2. 用 [二连杆组合工具](../../scripts/prepare_x650_arm.py)及 [带臂 airframe 工具](../../scripts/prepare_x650_arm_airframe.py)产生独立版本。控制包核验同目录 URDF、运动学、净距、自碰撞证书的内容哈希，以及总质量、机体坐标、关节约定和安装偏移。
3. 在目标 PX4/Gazebo 环境按 [原生记录器构建说明](../../benchmark/whole_body_closed_loop_v1/README.md)从源码构建插件。任务的 `native_recorder_library` 必须显式指向当地构建库的绝对路径；预编译 `.so` 不随源码分发。任务还需要实际安装的模型、flight profile、airframe 与原生导入 manifest。
4. 在 `--task` JSON 中填写这些实际输入及独立实验的证据路径。`task.py` 的默认本地路径是版本约定，不是缺少资产时的自动下载或默认替代。换几何、质量、PID 或世界文件后必须重新准备和核验，不能复用旧成功证书。

没有上述私有几何时仍可运行合成单元测试、阅读与调用纯数值接口；不能据此启动这台精确 X650 预设或声称复现了它的飞行。

```bash
# 仅构建和检查观察插件，不启动 PX4/Gazebo，不下载依赖。
python3 benchmark/whole_body_closed_loop_v1/build_recorder.py
gz plugin --info --plugin benchmark/whole_body_closed_loop_v1/build/libwholebody_recorder.so
```

任务默认仍保留历史库路径。新构建输出位于 `benchmark/whole_body_closed_loop_v1/build/`，应在新 task 中设置对应绝对路径，并保存构建 manifest 与二进制哈希；不要覆盖既有实验绑定的旧库。插件绑定原二连杆的 `arm_link_2` 与局部 `[0.25,0,0]` 末端点，不是任意 URDF 的通用末端传感器。

## 当前精密协议：先准备，再接上层 PID

当前正式接入按 `research_profile` 分为两个版本。**`precision_ev_upper_v2` 仅允许 `pid_ff` 的 `response_probe` 和 `tip_hold`；MPC、small/normal/fast 八字不在该版本的允许范围。** 下文的全 stage 目录描述源码能力，不能替代这里的放行范围或某次完整实验验收。

| 协议 | 允许的任务 | 控制与证据用途 |
|---|---|---|
| `precision_ev_selected_pid_v1` | `hover_folded`、`joint_probe`，均为 `pid_ff` | PX4 原生纯位置/yaw 闭环，验证精密悬停和关节运动时的物理准备状态 |
| `precision_ev_upper_v2` | `response_probe`、`tip_hold`，均为 `pid_ff` | PX4 估计状态作为上层反馈，直接 world→NED P/V/A 编码；先独立辨识响应，再验收末端定点 |

两个版本都要求同一个显式世界文件和 `fixed_world_position_v1` 参考配置、完整冻结的 **42 项 MC/MPC 实际参数**，以及预先选定 `velocity_high` 四项增益。参数事务在地面应用 7 项精密 EKF 参数与 4 项 PID 候选值，记录类型/字节及读回；完整降落解除武装后恢复原值并核验。操作条件哈希绑定物理模型、全部 42 项参数、世界文件、EV 配置、PX4 源码版本和控制接口。有限候选选择不等于 PID 全局最优。

精密 EV 由运行期 Gazebo 测量送入 PX4 EKF，50 Hz、无附加高斯噪声、测量点为 `base_link`，因此两版均保留 `ground_truth_aided=true`。v2 上层直接使用 PX4 `VehicleLocalPosition`、`VehicleAttitude`、`VehicleAngularVelocity` 与实测 `JointState`；独立 native 真值用于误差验收。该实验不支持 GPS 性能或实机定位精度的推论。

`estimated_feedback.py` 在已绑定的 `raw_ev_world` 原点下转换状态：世界位置为 `[local_y, local_x, frozen_ref_alt-local_z]`，旋转为 NED→ENU、PX4 FRD→NED 四元数、FLU→FRD 三项的乘积。原点和位置/速度/航向/四元数重置后拒绝继续，不重新拟合坐标原点。P/V/A 命令直接用这个冻结转换编码，不逐帧利用真值与估计误差平移命令。

反馈协议是**各 topic 最新异步样本的接收时龄检查**，并不声称源采样时刻同步。时间来自 ROS `/clock`；原始 DDS 时间保留作诊断。位置、姿态、角速度保持时间 ≤0.10 仿真秒，关节 ≤0.25 秒；flags ≤1.25 秒，因固定 PX4 的状态标志每 1 秒或状态变化时发布，DDS 5 Hz 是上限。flags 的墙钟上限为 10 秒，其余为 4 秒。`ControlState.stamp_s` 使用角速度回调的接收仿真时刻，旧角速度不会因位置回调更新而被再次求导。缺失、过期、无效或融合条件改变都报错，不回退到真值反馈。

地面话题检查用于确认流和初始姿态/航向对准，不等于 PX4 已完成飞行中的最终航向对准。上层控制开始前仍须在起飞后严格确认 `heading_good_for_control`；不能为通过地面探针而关闭这一运行期门禁。

v2 `response_probe` 必须有同模型、EV、PID、世界条件的 v1 地面/悬停/关节审计证书。v1 在这里只作为 `preparation_only`，不会被改写成 v2 的上层性能证据。先完成独立 train、holdout 两次 response，再分别进行完整 native/ULog/反馈与实际 PX4 P/V/A 审计；原始 runner 结果保持 provisional，不覆盖成通过。

```bash
# 在本机已有 ROS/PX4 环境和外部模型/证据齐备后执行；占位目录需替换。
python3 simulations/x650_wholebody_mpc_v1/launch.py \
  --task /absolute/reviewed_precision_v2_response_task.json --dry-run

# 完整运行并关闭 runtime 后，对每次独立 response 分别审计。
python3 scripts/audit_x650_upper_stage.py \
  --run-dir /absolute/completed_response_run \
  --launch-dir /absolute/its_matching_launch \
  --output /absolute/new_response_audit

python3 scripts/identify_x650_upper_response.py \
  --train /absolute/train_run --holdout /absolute/independent_holdout_run \
  --train-audit /absolute/train_audit/upper_stage_audit.json \
  --holdout-audit /absolute/holdout_audit/upper_stage_audit.json \
  --output /absolute/new_response_identification
```

审计成功才另写 `research_stage.json`、`research_ground.json`；它们绑定完整 ULog、native trace、源码快照、实际命令、反馈快照和参数复原。v2 `tip_hold` 还要求同 v2 条件的 `response_identification.json` 及其绑定的 `response_model.json`。两次响应使用零加速度前馈辨识折叠臂的基体闭环速度模型，不能据此声称机械臂动态或带前馈机动已标定。

v1 用 `audit_x650_precision_stage.py` 完成独立审计；精密悬停门槛是三维 RMS ≤2 cm、峰值 ≤5 cm，joint 保留原方向、跟踪和机体包络门槛并另报位置误差。v2 tip 定点门槛为末端 RMS ≤5 cm、峰值 ≤10 cm，属于接入验收。它没有注入外力，不能称为外力扰动补偿试验。源码、预检、话题连通或一次协议通过都不是 MPC 优于 PID 的证据。

## 全 stage 代码目录（不代表精密 v2 全部放行）

所有阶段均检查准备报告、实际物理模型、flight profile、airframe、原生 observer 和既有证据。每次新运行还会重新采集地面稳定窗口，然后正常起飞、执行任务、恢复折叠 q0、降落并等待解除武装。发生异常保留原始错误和恢复记录，不把失败任务改写成成功，也不把 MPC 静默切换成 PID。

| stage | 已实现的任务 | 测量时间 / 轨迹 | 控制入口 |
|---|---|---|---|
| `hover_folded` | NavTakeoff 上升 2 m，稳定后保持折叠臂悬停，独立原生位置/姿态验收 | 20 s 测量；起飞后先稳定 3 s、再保持 5 s | PX4 原生飞行 PID + q0 关节保持 |
| `joint_probe` | 空中依次运动 q1、q2、双关节，核验活动幅值、方向、跟踪和机体包络 | 30 s；单关节幅值 0.08 rad，双关节幅值 0.06 rad，C2 起止 | PX4 飞行 PID + 关节速度参考 |
| `response_probe` | 折叠臂状态下逐轴正负小速度脉冲，采集闭环平移响应 | 30 s；X/Y 幅值 0.08 m/s，Z 幅值 0.05 m/s，每个正负脉冲 1 s，中间 1 s 零指令；末段返回锚点 | 先发送 offboard 参考，再切模式；PX4 执行速度闭环 |
| `tip_hold` | 以实际机体位置、姿态、速度和关节状态闭合末端定点任务 | 30 s | `pid_ff` 或 `mpc` |
| `small` | 小幅慢八字 pilot | 世界 XZ；X ±0.075 m、Z ±0.03 m，16 s/圈、2 圈；任务 40 s | `pid_ff` 或 `mpc` |
| `normal` | 原尺寸普通八字 | 世界 XZ；X ±0.25 m、Z ±0.10 m，12 s/圈、2 圈；任务 32 s | `pid_ff` 或 `mpc` |
| `fast` | 原尺寸快速八字 | 世界 XZ；X ±0.25 m、Z ±0.10 m，8 s/圈、2 圈；任务 24 s | `pid_ff` 或 `mpc` |

八字使用原实验的 C2 相位速度起止平滑：任务第 2 s 开始，普通运动窗口为 2–28 s，快速为 2–20 s，小幅 pilot 为 2–36 s。名义飞机/臂比例保留 80%/20%；实时闭环纠偏由实际状态、目标和约束确定。

前三个 commissioning stage 只能选择 `pid_ff`。此处名称用于统一任务选择，它们尚未运行末端分配器：默认 `hover_folded`、`joint_probe` 使用 PX4 原生起飞后的飞行保持；显式选择下文固定世界参考后，改为 Offboard 纯位置/yaw 参考，仍由 PX4 原生 PID 执行。`response_probe` 使用参考激励。不能把它们描述成已完成上层 MPC 性能测试。

`tip_hold` 当前没有注入机体外力、外力矩或独立控制扰动，元数据为 `external_force_disturbance=false`。它先验证定点闭环；受扰主动补偿的定量对比仍需单独接入物理扰动及其审计。

## 起飞前的地面稳定检查

runner 动态向 Gazebo world 加载已有的 `wbbench::WholeBodyRecorder`，不修改已哈希的模型 SDF。它记录同一物理步的 `base_link` 世界状态、实际关节 q/dq 和 `arm_link_2` 局部 `[0.25,0,0]` 末端点。

每次运行先记录 5.5 s，在其中完整的 5 s 原生窗口检查：倾角 ≤2°、机体速度 ≤0.03 m/s、两关节绝对角度 ≤0.01 rad、角速度 ≤0.02 rad/s、原生字段有效且最大物理采样间隔 ≤8 ms。生成 `native_ground.csv`、`native_ground_manifest.json` 和 `native_ground.json`；失败时不解锁、不起飞。

这项检查验证**原生运动学静置、折叠构型和采样稳定**。它没有使用 ContactSensor，也没有直接核对每个接触对或接触力；不能据此宣称已经完成完整的原生接触/碰撞传感验证。静态网格/自碰撞证书、地面静置检查和后续飞行验证分别保留。

## 服务器环境与基础入口

以下命令从现有服务器仓库执行，使用数据盘已有程序。无需重新下载 Node、PX4、Gazebo、ROS 或 Python 数值库。

```bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
export AMSIM_VENV=/root/autodl-tmp/amsim-venv

# 此容器已验证的系统 NumPy / SciPy 与 Python 3.10 ABI 匹配。
# 仅给当前命令进程优先复用系统数值库，保留现有 venv 的其他依赖。
export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

"$AMSIM_VENV/bin/python3" simulations/x650_wholebody_mpc_v1/launch.py \
  --stage hover_folded --controller pid_ff --dry-run
```

`--dry-run` 不启动仿真、不解锁。检查输出中的 `preflight.ready` 是否为 `true`；命令返回码为 0 本身不表示模型和证据已就绪，失败原因也会以 JSON 返回。上面的默认 task 没有启用精密 research profile；继续精密实验时应使用带相应 profile 和已审计输入的 `--task` JSON，不把默认入口当成 v2。

确认已有 airframe/model 安装和输入哈希就绪后：

```bash
"$AMSIM_VENV/bin/python3" simulations/x650_wholebody_mpc_v1/launch.py \
  --stage hover_folded --controller pid_ff
```

launcher 使用共享飞行锁，每次创建独立 `runs/x650_wb_launch_*`，平台另外创建 `runs/x650_wholebody_mpc_v1_*`。长任务应从现有服务器后台会话启动，避免终端或 Remote 连接中断影响任务。不要并行运行两次飞行，也不要复用已有输出目录覆盖失败证据。

`launch.py` 会向子进程设置单线程数值库环境；这是线程数请求，不是实时性证明。系统 NumPy/SciPy 的 `PYTHONPATH` 复用只针对已检查过的当前容器栈，不修改全局 venv。换容器后先验证实际解释器和 import 路径。

## 后续任务配置与共享证据

预设的轨迹、时长、模型名和控制边界由 `task.py` 冻结。可编辑字段包括已审核输入文件路径、`pid_profile`、`response_model`、`stage_evidence`、`research_profile`，以及下文诊断 profile 和 `pid_tuning_candidate`。每个可选字段有独立白名单和 stage 限制，不接受任意参数/轨迹变体。输入 JSON 的所有路径使用服务器绝对路径。研究协议与诊断/整定 profile 互斥；以下旧诊断说明不替代上面的研究协议。

### 固定目标、定位诊断与 PID 候选

三个 profile 及候选字段默认均为 `null`，旧任务仍可加载：

| 字段 | 接受的配置 | 允许范围 |
|---|---|---|
| `hover_reference_profile` | `{"name":"fixed_world_position_v1","world_sdf":"/absolute/actual_world.sdf"}` | 默认允许 `hover_folded / joint_probe`；精密 v2 则要求 `response_probe / tip_hold`，均为 `pid_ff` |
| `diagnostic_profile` | `diagnostic_parameters.PROFILE`（水平 EV 辅助）或 `PRECISION_PROFILE`（EV XYZ+速度、EV 高度参考、关闭 GNSS/气压融合）的精确字典 | 仅 `hover_folded / pid_ff`；精密模式还要求固定世界参考 |
| `precision_test_profile` | `"hover20_steps36_v1"` | 仅固定世界参考下的 `hover_folded / pid_ff` |
| `pid_tuning_candidate` | `diagnostic_parameters.PID_CANDIDATES` 中九个名称之一，或完全相同的四参数字典 | 仅固定世界参考下的 `hover_folded / pid_ff`；不得同时使用正式 `pid_profile` |

固定目标由起飞前机体世界位置加 2 m 一次确定；测量前使用实际 `vehicle_local_position` 原点编码，之后不把飞机实际位置重新设为目标。发送掩码 `2552`，只保留位置与 yaw，速度/加速度/yaw-rate 字段忽略；不走原有按期望速度编码的上层入口。原 GPS 条件使用 `global_wgs84`，通过 Gazebo WGS84 世界基准、ECEF 和 PX4 MapProjection 转换；精密 EV 条件使用已核对源码的 `raw_ev_world`，不可混用高度基准。原点、有效性或位置/航向重置改变会终止参考，不自动重绑定。

`precision_test_profile` 保留前 20 s 主悬停窗口，之后按 X +0.15 m、归零、Y +0.15 m、归零、Z +0.10 m、归零，各保持 6 s，总测量 56 s。每段均从同一个世界锚点预编码，只改变一个轴；记录名义边界与首次实际发令时刻。该协议测位置指令响应，未注入外力，不叫外力扰动恢复。响应恢复判据为三维位置误差 ≤0.02 m 且原生速度 ≤0.03 m/s 连续 2 s；未在观察窗内达到则报告未稳定。

候选真实应用 PX4 的四项 P 增益：`MPC_XY_P`、`MPC_Z_P`、`MPC_XY_VEL_P_ACC`、`MPC_Z_VEL_P_ACC`。目录包括 `baseline`、`velocity_low`、`velocity_high` 及 `position_{low,high}_velocity_{baseline,low,high}`；具体数值和整定边界见 [PID 基线说明](../../docs/X650_PID_BASELINE.md)。`pid_profile` 仍只读核对，含义未改变。只传候选且 `diagnostic_profile=null` 时保留原 GPS/气压定位，不创建 EV 观测插件。

参数事务只在已确认 landed/disarmed 后写入。写前保存全部 42 项控制原值、类型和原始字节及待恢复日志；候选写入逐项读回，非候选控制参数受保护。落地解除武装后恢复原值、读回并同步保存 PX4 参数存储，成功后清除日志；失败留下日志并禁止下一次飞行。实际写入标记为 `parameter_writes=true`，参数证据采集器自身仍只读。

定位诊断通过运行期附加 OdometryPublisher 和 PX4 原生 GZBridge 提供理想测量，不改变物理 SDF；必须实际确认对应融合标志，不能把“已写 EKF 参数”当成“已经融合”。EV 数据条件标为 `ground_truth_aided=true`，应与原 GPS 条件分开报告。

启用 `diagnostic_profile`、`pid_tuning_candidate` 或 `precision_test_profile` 的结果均为诊断，`eligible_as_stage_gate=false`，不能用作正式 MPC 前置 gate。有限九项候选比较不代表充分整定或全局最优。`scripts/run_x650_precision_candidates.py` 只顺序运行明确目录候选，任务/物理/参数/恢复或分析任一失败即停；其自动完整分析当前要求精密 EV 基线。原 GPS 复核由独立任务执行并采用 WGS84 分析方法。

原生物理悬停/阶跃可分别用 `analyze_x650_hover_precision.py --native-only` 和 `analyze_x650_position_steps.py`；精密 EV 的完整 ULog 分解使用前者默认模式。原 GPS 使用 `analyze_x650_hover_chain.py --world-sdf ... --model-sdf ... --px4-source ...`，并用只读 `audit_x650_gps_crosscheck.py` 核验整个 56 s 的有效 PID、融合状态和复原证据。旧 GPS 分解器的 `selected_initial_parameters` 不包含起飞前所有候选变更，不能单独当作候选实际生效的证明。

取得首轮真实通过结果后，可用任务生成器创建后续 JSON。下面的 `HOVER_RUN`、`JOINT_RUN` 等必须替换成**实际完成且审计通过**的结果目录；这些名称只是命令示例，不声明任何运行已通过。

```bash
export HOVER_RUN=/root/autodl-tmp/safe-aerial-manipulator-testbed/runs/REPLACE_WITH_SUCCESSFUL_HOVER
export JOINT_RUN=/root/autodl-tmp/safe-aerial-manipulator-testbed/runs/REPLACE_WITH_SUCCESSFUL_JOINT_PROBE
export TASK_OUT=/root/autodl-tmp/safe-aerial-manipulator-testbed/generated/x650_joint_probe_task.json

"$AMSIM_VENV/bin/python3" - <<'PY'
import json, os
from pathlib import Path
from simulations.x650_wholebody_mpc_v1.task import build_task_config
cfg = build_task_config("joint_probe", "pid_ff")
previous = Path(os.environ["HOVER_RUN"])
cfg["stage_evidence"] = {
    "native_ground": str(previous / "native_ground.json"),
    "hover_folded": str(previous / "hover_folded.json"),
}
destination = Path(os.environ["TASK_OUT"])
if destination.exists():
    raise FileExistsError("Use a fresh task file: " + str(destination))
destination.write_text(json.dumps(cfg, indent=2) + "\n")
PY

"$AMSIM_VENV/bin/python3" simulations/x650_wholebody_mpc_v1/launch.py \
  --task "$TASK_OUT" --dry-run
"$AMSIM_VENV/bin/python3" simulations/x650_wholebody_mpc_v1/launch.py \
  --task "$TASK_OUT"
```

共享的 `stage_evidence` 映射示意如下，值都是已有证据 JSON 的绝对路径：

```json
{
  "native_ground": "/absolute/successful_hover/native_ground.json",
  "hover_folded": "/absolute/successful_hover/hover_folded.json",
  "joint_probe": "/absolute/successful_joint/joint_probe.json",
  "response_identification": "/absolute/identification/response_identification.json",
  "pid_baseline": "/absolute/frozen_pid_validation/pid_baseline.json",
  "small": "/absolute/successful_small/small.json",
  "normal": "/absolute/successful_normal/normal.json"
}
```

门槛依赖由 `task.required_gates(cfg)` 决定。下表是 `research_profile=null` 的旧任务链；精密 v1/v2 使用前文额外操作条件与独立审计门禁，不能直接套用下表的原始结果路径：

| 即将执行的任务 | 必需的前序证据 |
|---|---|
| 首次 `hover_folded` | 输入模型/质量/几何/airframe 审核；地面稳定证据由本次运行先采集 |
| `joint_probe`、`response_probe` | `native_ground`、`hover_folded` |
| PID `tip_hold`、PID `small` | 上述两项 + `joint_probe` |
| MPC `tip_hold`、MPC `small` | 上述三项 + `response_identification`、`pid_baseline`，并提供响应模型和 PID 参数文件 |
| `normal`，两种控制器均相同 | 上述五项 + `small` |
| `fast`，两种控制器均相同 | 上述六项 + `normal` |

证据格式为 `kind=x650_native_stage_evidence`、`simulation_backend=gazebo_px4`、相应 `stage`、`complete=true`、`passed=true`，并绑定本次实际 model/source URDF SHA 和 `artifacts: {文件路径: SHA256}`。相对 artifact 路径以证据文件所在目录解释。preflight 与实际 runner 都检查真实文件和哈希，传入 `true` 不能代替证据。

PID 与 MPC 各自保留自己的 pilot/正式结果；为某一控制器准备下一步 task 时，应传该控制器对应的 `small` 或 `normal` 结果。门槛表示工程前序条件已满足，不表示 MPC 必然优于 PID，也不是全局最优/稳定性证明。

## 后续 MPC/八字路线（需另行放行）

以下保留完整开发顺序，描述未来接入依赖，不表示当前精密 v2 允许执行这些 MPC 或八字任务。只有完成新的协议版本、模型/PID/反馈来源绑定和相应前置验收后，才能继续到对应阶段。

1. **首轮带臂悬停**：检查原生地面稳定，再执行 `hover_folded`。成功必须包含完整 20 s 测量、实际 q0 恢复、正常降落与解除武装、参数前后快照和原生采样。
2. **关节检查**：执行 `joint_probe`，核验两个关节各自正负方向、活动幅值、跟踪误差，以及机体位移/倾角。只飞完、无异常还不足以通过此 gate。
3. **两次独立响应采集**：在相同新机模型与 PID 下，分别执行两次 `response_probe`；一次固定作 training，另一次固定作 holdout。每次使用新 run，不把同一次记录复制成两次，也不根据最终误差挑最佳重复。该 stage 只提供数据，不自动声称已辨识。
4. **响应辨识**：使用下面的离线入口，用 training 拟合各轴闭环速度时间常数，再固定参数对 holdout 验证。单位增益、零偏置、零命令延迟的一阶模型若不符合原生记录，报告应失败；不要通过任意相移或拟合 holdout 来使它通过。
5. **新 X650 PID 基线确认与冻结**：结合悬停、小幅响应及 PID 定点/小八字 pilot，确定新机的实际位置/速度/姿态/角速度内环参数。旧 x500 的冻结 26 项不能作为新机已充分整定的证明。保存 `pid_profile` 参数文件和独立、绑定原始实验的 `pid_baseline` 验证证据；目前 runner 不自动整定，也不会仅凭写出 JSON 自动颁发这份证据。
6. **定点与小八字接入**：分别运行 PID 与 MPC 的 `tip_hold`、`small`。先解决坐标、参数、状态年龄、求解可行性和调度问题，再冻结比较配置。
7. **正式普通/快速八字**：两组使用同一新机、内环 PID、前馈、限制、反馈来源和仿真时钟设置，分别完成 `normal`，再进入 `fast`。正式比较建议保留至少 3 次独立重复，并预先固定视频所用重复编号。

响应辨识入口已实现；以下参数仍须替换为真实成功目录和新的输出目录：

```bash
"$AMSIM_VENV/bin/python3" scripts/identify_x650_upper_response.py \
  --train /absolute/response_training_run \
  --holdout /absolute/response_holdout_run \
  --output /absolute/new_response_identification_directory
```

只有 `response_identification.json` 通过且 `response_model.json` 为 `identified=true, passed=true`，才用作 MPC 的模型输入。`response_model` 的任务字段指向参数文件 `response_model.json`；`stage_evidence.response_identification` 指向证据文件 `response_identification.json`。此辨识范围是折叠臂的基体闭环平移速度响应，不包含完整浮动基动力学、关节动态辨识或实机实时性。

PID 参数文件的最低结构为：

```json
{
  "model_sha256": "实际新模型SHA256",
  "parameters": {
    "MPC_XY_P": "实际冻结数值",
    "MPC_Z_P": "实际冻结数值"
  }
}
```

这里用字符串占位强调它是结构示意，不能直接用于飞行；实际值必须是数值，并应保存实际选择的完整内环参数。`pid_profile` 的核对路径在起飞前通过现有 MAVLink 连接只读确认实际增益，与文件比较；独立候选写入仅由上述显式诊断事务执行，不在飞行中自动调参。`MPC_*` 在 PX4 参数名里表示多旋翼位置控制，不等于模型预测控制。

对已准备好所有证据的 task，运行方法统一为：

```bash
"$AMSIM_VENV/bin/python3" simulations/x650_wholebody_mpc_v1/launch.py \
  --task /absolute/reviewed_task.json --dry-run
"$AMSIM_VENV/bin/python3" simulations/x650_wholebody_mpc_v1/launch.py \
  --task /absolute/reviewed_task.json
```

## 时间、实时因子与指标

任务采用仿真秒，目标控制周期 0.02 s；runner 记录实际 `control_dt_s`，超过 0.05 s 会失败。真实反馈时间戳回退、状态/关节年龄超界、PX4 failsafe、非预期 offboard 退出等也会终止任务并尝试恢复降落。

**当前 RTF 配置及 50 Hz 真实运行能力尚未冻结。** MPC 计时需要包含 QP 构建、求解、质量检查、前馈、ROS/MAVLink 和日志；离线耗时不能直接当作原生闭环调度结果。若为仿真验证降低 `real_time_factor`，PID 与 MPC 必须使用相同世界文件、物理积分步长与仿真时刻轨迹，并归档实际 RTF/世界配置哈希；运行变慢不构成实机实时性通过。当前默认 `solve_budget_ms=20` 仅记录超预算，不能中断正在执行的 SLSQP。

原生位置/末端指标使用原记录仿真时刻，与同一时刻的固定参考或解析目标比较，按时间积分加权计算三维 RMSE；同时报告峰值、各轴误差和姿态/关节状态。没有事后平移、相位对齐或幅值缩放。

`hover_folded` 的位置验收采用独立 `native_trace.csv` 中的 `base_p` 与四元数，核验完整 20 s、首尾覆盖和最大 8 ms 采样间隔，门槛为 RMS ≤0.25 m、峰值 ≤0.50 m、倾角 ≤15°。这是首次工程接入门槛，不表示精度已优化。末端任务的主要指标来自同一原生插件测得的真实 tip；控制器自行 FK 只能作诊断，不能改名成独立原生真值。

## 每次运行的主要文件

| 文件 | 作用 |
|---|---|
| `controller_task.json` | 实际任务和输入路径 |
| `controller_metadata.json` | 阶段、模型与来源哈希、真实参数、控制器详情、时间、验收/错误和落地状态 |
| `flight_samples.csv` / manifest | 全过程真实机体位置/姿态、关节、各自时间戳、phase；不伪造独立 tip |
| `native_stream.csv` | Gazebo observer 持续追加的原始物理步记录；runtime 关闭前可能继续增长 |
| `native_trace.csv` / manifest | 从原生 stream 冻结的完整原始行和不可变哈希，用于独立物理指标 |
| `native_ground.csv` / manifest / `native_ground.json` | 起飞前的独立稳定窗口和证据 |
| `control_trace.csv` / `controller_diagnostics.jsonl` | 实际反馈、命令、状态年龄、dt、耗时及分配/前馈/求解诊断 |
| `feedback_trace.jsonl` | 精密 v2 的估计状态快照、回调序号、接收时龄、原始 DDS 时间、原点/重置/融合标志及直接 P/V/A 命令证据 |
| `px4_parameters_before.json` / `after.json` | 原有 MAVLink 连接上只读取得的原始参数值与类型/字节证据 |
| `hover_reference.json`、`position_step_probe.json` | 可选固定世界参考、原点/转换契约、六段声明目标；实际首次发令时刻另记录于 metadata |
| `diagnostic_parameter_session.json`、`diagnostic_parameters_before/applied/restored.json` | 可选参数事务的原值、写入、读回、复原与存储保存证据；待恢复日志在 `.amsim/` 中阻止未完成复原后的新飞行 |
| `<stage>.json`、`stage_result.json` | 完成/通过状态及下阶段可以核验的原始文件哈希 |
| `diagnostic_result.json` | 可选诊断任务结果，不能替代正式 `<stage>.json` gate |
| `precision_stage_audit.json` / `upper_stage_audit.json` | 独立事后审计输出；完整通过后另写 `research_stage.json` 和 `research_ground.json`，不覆盖原始 provisional/失败记录 |
| `model_snapshot.sdf`、ROS bag、`px4.ulg` | 平台归档的实际物理模型和原始记录；ULog 在 PX4 正常关闭后复制 |

记录视频时使用 X650 v2 的原 URDF 网格与测量机体/关节状态回放，保留“记录状态回放”的标识；没有记录的桨叶角度不补造旋转动画。科学指标以原始记录为准，视频帧插值不能进入指标计算。

## 本地回归

使用已有 NumPy/SciPy 的 Python 环境：

```bash
python3 -m unittest discover -s control_packages/x650_wholebody_v1/tests -v
python3 -m unittest discover -s simulations/x650_wholebody_mpc_v1/tests -v
```

runner 当前回归覆盖地面失败不解锁、地面通过后起飞、20 s 测量/折叠/降落、起飞异常恢复、关节原生跟踪验收、offboard 预发送、真实 artifact/hash 门槛、ENU/FLU 速度变换、不可变原生 CSV 与稀疏/缺段悬停拒绝；另有固定世界参考、原点重置保护、参数事务复原、PID-only 无 EV 观测器、20+36 s 阶跃和实际发令时刻的独立测试。这些测试使用明确标注的合成或 mock 数据，不能作为任何一次原生飞行的通过证据。
