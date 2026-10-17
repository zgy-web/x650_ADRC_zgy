# PID + 前馈 + 末端协同闭环，v1.0.0

本包将已验证的 `whole_body_closed_loop_v1` 数值控制器封装为独立 Python API，并提供现有 AMSIM / PX4 SITL / Gazebo 平台的调用入口。运行时包仅依赖 NumPy；仿真适配器使用已有的 ROS 2、MAVLink、PX4 和 Gazebo 环境。原始研究实验和大型日志独立保留，日常使用本包无需它们。

入口：[平台 README](../../README.md) · [接入与验证说明](../../docs/PID_FF_WHOLEBODY_V1.md) · [验证记录](../../docs/validation/pid_ff_wholebody_v1_20261017/) · [可执行 API 示例](examples/one_step.py)

## 从源码安装

控制包需要 **Python ≥3.10、NumPy ≥1.23**。仅调用控制器或运行接口测试不需要 ROS、PX4 或 Gazebo。在目标 Python 环境中，从仓库根目录执行：

```bash
python -m pip install ./control_packages/pid_ff_wholebody_v1
python control_packages/pid_ff_wholebody_v1/examples/one_step.py
```

若环境已有 NumPy、setuptools ≥61 和 wheel，可离线构建安装，不下载依赖：

```bash
python -m pip install --no-deps --no-build-isolation ./control_packages/pid_ff_wholebody_v1
```

若已取得发布 wheel，也可执行 `python -m pip install --no-deps /path/to/flyarm_pid_ff-1.0.0-py3-none-any.whl`；该方式要求环境已有 NumPy。Git 仓库提供源码，wheel 不作为运行的前置条件。当前仿真适配器直接加载仓库内的包，无需额外安装 wheel。

## 在仿真平台中运行

实际飞行在 Linux 上运行，需要完成[平台安装](../../docs/INSTALLATION.md)，并通过 `./scripts/amsim doctor`。已有服务器环境可直接复用。以下命令均从仓库根目录执行，使用已验证服务器的 Python 环境：

```bash
export AMSIM_VENV=/root/autodl-tmp/amsim-venv

# 查看配置，不启动仿真
python3 simulations/pid_ff_wholebody_v1/launch.py --dry-run

# 悬停，开启末端协同闭环
python3 simulations/pid_ff_wholebody_v1/launch.py --trajectory hover --mode ON

# 普通八字：12 秒/圈，2 圈；快速八字：8 秒/圈，2 圈
python3 simulations/pid_ff_wholebody_v1/launch.py --trajectory normal --mode ON
python3 simulations/pid_ff_wholebody_v1/launch.py --trajectory fast --mode ON

# 普通八字的配对基线
python3 simulations/pid_ff_wholebody_v1/launch.py --trajectory normal --mode OFF
```

在其他机器上，将 `AMSIM_VENV` 改为已有仿真 Python 环境的绝对路径，例如 `export AMSIM_VENV=/absolute/path/to/amsim-venv`；其中应有 `bin/python3` 和平台所需依赖。若 `.amsim/local.env` 也设置了该变量，保持两处一致，因为平台启动脚本会读取此文件。启动器本身仅用 Python 标准库；实际场景由平台使用所选环境执行。`--dry-run` 可在 Windows 或未安装仿真的机器上运行，但它不验证飞行依赖是否齐全。

`--mode OFF` 关闭末端协同反馈，保留相同 PID、名义分配和机械臂运动前馈，用于配对比较。ON/OFF 都不是关闭底层飞行闭环。新场景默认无外加扰动；悬停接入检查不等于重做原受扰对比实验。

启动器采用 headless 模式，检查共享飞行锁和运行状态，每次生成独立输出目录。适配器读回确认全部 26 项 PID 后起飞，读取实际机体与关节状态，执行任务，停止机械臂并降落；启动器同时检查场景成功、控制器完成和运行环境关闭。逐次执行实验，并保留平台默认的仓库 `runs/` 输出目录：本版本启动器按该位置查找结果，不支持把 `AMSIM_RESULTS_DIR` 改到其他位置。

## 在其他 Python 程序中调用

完成上面的源码安装后，可直接导入：

```python
import numpy as np
from flyarm_pid_ff import ControlState, EndpointTarget, PidFeedforwardController, tip_world

# 以下为接口示例。实际仿真循环必须每步读取新的测量状态。
initial = ControlState(
    position=[0., 0., 2.5], rotation=np.eye(3), velocity=[0., 0., 0.],
    omega_body=[0., 0., 0.], q=[0., 0.], dq=[0., 0.], stamp_s=10.,
)
controller = PidFeedforwardController(initial, mode="compensated")
target = EndpointTarget(tip_world(initial.position, initial.rotation, initial.q))
command = controller.step(t=0., dt=.02, state=initial, target=target)

# 飞机侧：与当前 PX4 位置、速度估计一起编码，得到 NED 指令。
setpoint = command.px4_setpoint([0., 0., -2.5], [0., 0., 0.])
# 同时发送 setpoint.position / velocity / acceleration，type_mask=2048；航向另传。
# 机械臂侧：直接发送 command.joint_velocity，单位 rad/s。
```

完整可执行示例见 [examples/one_step.py](examples/one_step.py)。它验证 API 调用，不代表一次物理仿真。

| 接口 | 含义 |
|---|---|
| `ControlState` | 实际机体位置、姿态、线速度、机体角速度、两关节角/角速度和仿真时间戳 |
| `EndpointTarget` | 世界系末端目标位置、速度和加速度 |
| `PidFeedforwardController.step(t, dt, state, target)` | 每次返回飞机位置/速度/加速度参考、两关节速度和诊断信息 |
| `command.px4_setpoint(p_ned, v_ned)` | 将同一状态快照下的控制误差转换为 PX4 NED 参考 |
| `controller.hold(state)` | 首次任务前的预发送/稳定悬停，积累前馈滤波历史 |
| `controller.reset(initial_state)` | 新任务/新仿真时钟重新初始化；默认清空历史 |
| `frozen_pid_parameters()` | 返回经过 SHA-256 校验的 26 项 PID 参数独立副本 |
| `controller.metadata()` / `provenance()` | 参数、范围、原始来源与版本信息 |

位置和线速度用世界 ENU，旋转为机体 FLU 到世界 ENU；姿态也可传 wxyz 四元数。角速度为机体 FLU。关节用 rad/rad/s；时间用仿真秒。每架飞机、每个独立实验各创建一个控制器实例。控制周期约 0.02 s，传入实际 dt，必须满足 `0 < dt <= 0.05 s`；任务时间递增，测量时间戳不得回退。

预发送期间调用 `hold()`，开始任务后直接调用 `step()`，这样保留原来的角加速度滤波历史。`hold()` 仅用于初始化，使用原关节位置 P 和速度限幅，不适用任务期的制动/指令加速度限制。仿真时钟重置后调用默认 `reset()`。

## 这版算法具体包含什么

底层位置、速度、姿态、角速度控制仍由 **PX4 内的 PID** 执行。本包保留当前已整定的 26 项参数，并统一计算其输入参考；没有在 Python 中再串联一套飞行 PID。

末端实际位置为 `p_tip = p_body + R_body @ r_arm(q)`。协同闭环使用实际末端误差、飞机实际平移速度、姿态角速度和机械臂雅可比，求解带边界的最小二乘分配问题，输出 3 个飞机速度量与 2 个关节速度量。默认末端反馈增益为 2/s，补偿速度范数上限为 0.2 m/s。当前参考轨迹的名义比例仍为飞机 80%、机械臂 20%；启用闭环后，纠偏指令根据实际误差、目标函数权重和关节余量计算，不固定按该比例分摊。

飞机加速度输入包含轨迹加速度前馈与机械臂相对运动的惯性补偿。机械臂侧包含名义关节轨迹速度，以及协同分配产生的主动补偿速度；关节加速度参考用于飞机侧惯性补偿。机械臂仍由 Gazebo 速度执行器驱动，本版不输出关节力矩前馈，计算出的反作用力矩只记作诊断量。

使用 `px4_setpoint()` 输出的三个参考一起发送，勿将 `base_velocity_target` 与位置参考直接叠加发送，否则重复加入 PX4 的位置反馈。`joint_velocity` 已经是最终关节速度，不再叠加额外的关节位置 P。

此版本固定用于当前 `x500_underfold` 两自由度机械臂模型。模型不符、目标不可行、时间异常会显式报错。它是基于仿真真值辅助的末端**位置**协同闭环，并保留实测姿态引起的几何补偿；它不等同于六维末端位姿控制或全动力学 MPC。

## 固定配置与输出

普通/快速八字的幅值均为 X ±0.25 m、Z ±0.10 m，包含起止平滑段。飞机速度边界为 [0.55, 0.35, 0.45] m/s，关节速度为 [0.45, 0.35] rad/s，保留原指令加速度、关节软限位和制动约束。机械臂加速度补偿范数上限为 1 m/s²，合成加速度上限为 3 m/s²。当前发布包有意冻结模型、PID 和上述控制配置；新模型或重新整定应作为后续独立版本验证。

场景目录 `runs/pid_ff_wholebody_v1_*` 保存：

- `controller_task.json`、`controller_pid_parameters.json`：任务与固定 PID。
- `controller_metadata.json`：模型/源码哈希、参数读回值、完成/降落状态。
- `control_trace.csv`：测量状态、末端误差、飞机参考、关节指令及时间间隔。
- `controller_diagnostics.jsonl`：分配、限幅、前馈等逐步诊断。
- 平台原有 `result.json`、ROS bag、PX4 ULog 等证据。

启动记录另在 `runs/pid_ff_launch_*` 中，包含任务配置、日志与最终状态。

## 验证和来源

原数值实现来自研究工作区的 `benchmark/whole_body_closed_loop_v1`；数值核心仅调整导入以实现独立调用，外层 API 管理输入检查、状态历史和输出编码。原始文件与封装后文件的 SHA-256 见 [provenance.json](flyarm_pid_ff/data/provenance.json)。固定 PID 文件 SHA-256 为 `935428d891b59e64b6a0190f0eac888fbf66cbb989578745adefd25eda395dd2`。

从完整仓库的根目录运行接口与适配器测试：

```bash
python -m unittest discover -s control_packages/pid_ff_wholebody_v1/tests -v
```

这 15 项测试包括 API 状态处理、两个模式共 640 个合成控制周期的数值等价性，以及适配器正常结束和异常清理。适配器测试使用 mock，无需启动 ROS 或仿真。旧实现的冻结参考文件随仓库放在 `tests/fixtures/legacy_v1/`，测试始终使用这份经过 SHA-256 校验的参考，因此新克隆可以完成回归测试，无需取得完整的原实验目录。完整测试需要 `control_packages/` 和 `simulations/` 保持仓库中的相对位置。

真实历史日志逐步重放另需 **PyYAML**，并需要服务器上原始实验数据。输入目录必须包含 `metadata.yaml`、完整的 `control_trace.csv` 和 `model_snapshot.sdf`；不能用绘图用的降采样数据或裁掉预发送行的 CSV。原始日志未纳入 Git。

```bash
# 仅当重放环境缺少 PyYAML 时安装；已有服务器环境无需重复安装
python -m pip install PyYAML

python control_packages/pid_ff_wholebody_v1/tests/replay_control_trace.py \
  --case /absolute/path/to/original/run --output /absolute/path/to/new/report.json
```

`--case` 可重复传入多个实验目录。重放从原控制日志的预发送行开始，保留角加速度滤波历史，再将新包的 15 类输出与原指令逐项比较；该操作只读旧记录，不启动飞行。

2026-10-17 已核对四类实验 ON/OFF 共 8 份原始记录、13,443 个控制周期，15 类输出最大绝对差为 0；新封装入口另完成了一次真实 PX4/Gazebo 悬停接入检查。普通与快速八字入口已配置，本次封装没有重飞整套八字对比。详见[接入与验证说明](../../docs/PID_FF_WHOLEBODY_V1.md)和[原始验证摘要](../../docs/validation/pid_ff_wholebody_v1_20261017/)。
