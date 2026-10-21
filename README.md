# 安全空中机械臂测试平台

一个可复用的 **PX4 SITL + Gazebo Harmonic + ROS 2** 空中机械臂仿真、可视化、控制与数据记录平台。

> **当前仿真开发分支：** `simulation`
>
> 仿真平台有意独立于 `main` 分支维护。

<p align="center">
  <img src="docs/images/simulation_reviewed_underfold.png" width="48%" alt="Gazebo 中的已验证两连杆空中机械臂">
  <img src="docs/images/simulation_3link_overview.png" width="48%" alt="Gazebo 中的可配置三连杆空中机械臂">
</p>

当前版本已在一台全新的 Ubuntu 22.04 服务器上完成 clean-room 验证。验证内容包括 PX4 SITL、Gazebo Sim 8.15、ROS 2 Humble、基于 VirtualGL 的 NVIDIA GPU 渲染、已验证机械臂模型、可配置 N 连杆机械臂、公共 Vehicle/Arm API、rosbag 记录以及 PX4 ULog 归档。

这份源码包含平台、控制算法、数值配置、模型准备工具与测试。**用户自有 X650 的 CAD、URDF、网格及其压缩包不随源码提供**；生成模型、准备报告、飞行数据、视频、ULog、rosbag 和预编译插件也由实验者另行保存。下文中的模型输入与结果路径是本地运行约定，不是克隆后自动存在的附件。合成单元测试可以脱离私有模型运行；精确复现某次 X650 飞行则必须取得该次模型及哈希绑定的证据。

## PID + 前馈 + 末端协同闭环

**`flyarm-pid-ff 1.0.0` 已接入本分支。** 它将当前已验证的控制器封装为独立 Python 包，并提供悬停、普通八字和快速八字场景。底层 PID 在 PX4 内执行；包根据实际机体与关节状态计算末端位置误差，联合输出飞机参考和两关节速度，同时计算机械臂运动引起的飞机侧惯性补偿。

| 入口 | 用途 |
|---|---|
| [控制包 README](control_packages/pid_ff_wholebody_v1/README.md) | 安装、Python API、坐标系、控制参数与日志说明 |
| [仿真启动器](simulations/pid_ff_wholebody_v1/launch.py) | 选择轨迹和 ON/OFF 模式，管理飞行锁及运行结果 |
| [接入与验证说明](docs/PID_FF_WHOLEBODY_V1.md) | 算法范围、历史指令重放和真实服务器悬停证据 |

仅调用控制 API 时，需要 **Python ≥3.10、NumPy ≥1.23**。在已激活的 Python 环境中，从仓库根目录安装并运行示例：

```bash
python -m pip install ./control_packages/pid_ff_wholebody_v1
python control_packages/pid_ff_wholebody_v1/examples/one_step.py
```

实际飞行需要先完成下方的平台安装与 `doctor` 检查。已有仿真环境可直接使用，无需重新安装；将 `AMSIM_VENV` 设为该环境的实际路径。以下使用已验证服务器的路径，从仓库根目录执行：

```bash
export AMSIM_VENV=/root/autodl-tmp/amsim-venv

# 检查任务配置，不启动仿真
python3 simulations/pid_ff_wholebody_v1/launch.py --dry-run

# 悬停；普通八字 12 秒/圈；快速八字 8 秒/圈
python3 simulations/pid_ff_wholebody_v1/launch.py --trajectory hover --mode ON
python3 simulations/pid_ff_wholebody_v1/launch.py --trajectory normal --mode ON
python3 simulations/pid_ff_wholebody_v1/launch.py --trajectory fast --mode ON

# 同一普通八字的对照：保留相同 PID 和前馈，关闭末端协同反馈
python3 simulations/pid_ff_wholebody_v1/launch.py --trajectory normal --mode OFF
```

这些命令逐次运行，启动器采用 headless 模式，每次结果写入独立的 `runs/pid_ff_wholebody_v1_*` 目录。`ON` 和 `OFF` 都保留底层飞行闭环。名义轨迹分配为飞机 80%、机械臂 20%；ON 模式的纠偏量根据实际误差和约束计算。

本控制版本固定用于 `x500_underfold` 两自由度机械臂，使用 Gazebo 真值辅助反馈，控制末端位置并输出关节速度。原算法已完成四类实验 ON/OFF 的历史指令重放核对；新封装入口已完成一次真实 PX4/Gazebo 悬停接入验证。普通与快速八字入口已配置，本次封装没有重飞整套八字对比。详细范围和证据见上面的验证说明。

## 导入自有 URDF 模型

平台新增 `.urdf` / `.zip` 导入入口，保存原始模型及资源哈希，使用服务器已有的 `gz sdf` 转换为 Gazebo SDF。支持显式根坐标转换、`package://` 资源映射，以及无机械臂模型。URDF 的质量、惯量和关节限位按原文件保留。

```bash
# 先检查；可在不安装 Gazebo 的电脑上运行
python -m amsim.cli model-inspect /path/to/robot.zip

# 以下在已有 Gazebo 环境的服务器执行
./scripts/amsim model-import /path/to/robot.zip --name my_robot
./scripts/amsim model-preview amsim_urdf_my_robot --check --iterations 100
./scripts/amsim model-preview amsim_urdf_my_robot --static
```

最后一条生成静态查看场景并返回 `open_command`；执行返回的完整命令即可打开 Gazebo，命令中的独立 `GZ_PARTITION` 必须保留。动态检查会在重力下加载原模型，无动力的机体会下落。

导入完成后可检查模型外观和物理加载。进入 PX4 飞行前，需要额外提供明确的电机映射、转向、动力系数和已构建的 PX4 airframe；`model-bind-flight` 负责绑定并检查这些参数。未绑定的模型不能直接 `start` 或 `run`。现有 PID＋前馈包仍固定于原来的 `x500_underfold` 两关节模型。

完整命令、支持范围、飞行配置模板和 X650 的坐标转换见 [URDF 导入说明](docs/URDF_IMPORT.md)。导入与物理加载检查、动力配置绑定、悬停闭环验收各自生成独立证据，前一项不会自动证明后一项已经通过。历史验证记录及原始日志单独归档，不作为本源码发布的附件。

X650 数值动力配置采用 **T-MOTOR MN4014 KV330＋15×5CF、6S 30000 mAh、无臂整机 3.02 kg**，包括仿真电机编号、五点推力拟合、质量／惯量守恒检查和 PX4 4999 airframe。反扭矩与响应时间仍有显式估计项。源码说明见 [X650 动力配置](configs/vehicles/x650_no_arm/README.md)、[无臂悬停协议](simulations/x650_hover_v1/README.md)和[实测状态回放方法](docs/X650_HOVER_REPLAY.md)。

## X650 + 二连杆与精密定位接入

[X650 控制包](control_packages/x650_wholebody_v1/README.md)提供新的 PID＋前馈协同基线与上层运动学 MPC；[分阶段仿真入口](simulations/x650_wholebody_mpc_v1/README.md)管理地面、悬停、关节、响应辨识与末端定点验收。当前精密上层协议 **`precision_ev_upper_v2` 只接受 `pid_ff` 的 `response_probe` 和 `tip_hold`**。MPC、普通/快速八字及全身动力学 NMPC 尚未在该协议下放行；有源码、预检通过或地面话题正常，不等于 MPC 飞行性能已验证。

独立物理记录器的 [源码构建入口](benchmark/whole_body_closed_loop_v1/README.md)随仓库提供；在目标 Gazebo 环境构建并把实际库路径写入任务。构建成功、插件可加载和完整飞行验收是不同步骤。

通用 `model-import` 接受用户提供的 URDF/ZIP 与完整 mesh 资源；X650 预设则绑定特定源 URDF 哈希、二连杆构型、物理准备报告、airframe、世界文件和实际 PID 参数。`prepare_x650_flight.py` 会拒绝不匹配的源模型。换一台同名 X650 或改动几何，必须建立新的模型准备与证据版本，不能只改路径或移除哈希检查。

精密模式使用理想 Gazebo EV 测量辅助 PX4 EKF，因此标记 `ground_truth_aided=true`；v2 上层控制的直接反馈来自 PX4 估计状态和实测关节，独立物理真值用于验收。它验证当前仿真测量条件下的闭环，不提供 GPS 或实机定位性能结论。原始结果按独立运行保存，源码不附带可直接充当 stage gate 的成功证书。

## 项目提供的能力

- 固定 PX4 commit 的 PX4 SITL + Gazebo Harmonic 仿真运行环境。
- 已验证的两连杆空中机械臂模型。
- 可配置串联 N 连杆机械臂生成器。
- 自有 URDF/ZIP 模型导入、资源归档、独立 Gazebo 预览与显式 PX4 动力配置绑定。
- 稳定的公共实验 API：`ScenarioContext`、`VehicleAPI`、`ArmAPI` 和观测接口。
- 明确区分 PX4 估计状态与可选的 Gazebo ground truth。
- 场景自动发现和用户自定义场景模板。
- 平台统一管理 rosbag + PX4 ULog 记录。
- 面向远程 Linux 服务器的 GPU Gazebo GUI。
- Ubuntu 22.04 clean-host 安装脚本。
- 已实现 Gazebo ↔ MAVLink PX4 HIL 软件后端，并保留可选 Micro-XRCE-DDS ROS 2 链路。
- 可独立调用的 PID + 前馈 + 末端协同闭环控制包，以及悬停和八字仿真入口。

## 快速开始

### 1. 克隆 `simulation` 分支

```bash
git clone -b simulation https://github.com/lilinanan/safe-aerial-manipulator-testbed.git
cd safe-aerial-manipulator-testbed
```

如果仓库仍为 private，需要先完成 GitHub 认证。例如可以使用：

```bash
gh auth login
gh auth setup-git
```

也可以继续使用你已经配置好的 Git HTTPS 或 SSH 凭据。

### 2. 安装主机级依赖

当前验证目标系统为 **Ubuntu 22.04**。

```bash
sudo ./deploy/host/install_ubuntu22.sh
```

该脚本会安装：

- ROS 2 Humble
- Gazebo Harmonic / Gazebo Sim 8
- ros_gz
- rosbag2
- 编译工具链
- Micro-XRCE-DDS Agent 2.4.2
- 远程可视化所需工具

该脚本**不会修改**：

- NVIDIA 驱动
- 防火墙规则
- SSH 配置
- 云平台自身的系统服务

如需 GPU 渲染，请先确保服务器已有可用 NVIDIA 驱动：

```bash
nvidia-smi
```

### 3. 准备项目工作区

```bash
./scripts/amsim setup --check-only
./scripts/amsim setup
./scripts/amsim doctor
```

安装正常时，`doctor` 最后会输出：

```text
READY
```

第一次完整执行 `setup` 可能需要几分钟，因为需要编译 PX4 SITL 和 ROS 消息包。

### 4. 在远程服务器上启动可视化

如果运行在没有物理显示器的远程服务器上：

```bash
./scripts/amsim desktop-start
./scripts/amsim desktop-status
./scripts/amsim render-status
```

在验证通过的 NVIDIA 路径下，`render-status` 应显示类似：

```text
selected_backend: virtualgl
ready: true
gpu: NVIDIA ...
```

VNC/noVNC 默认只监听 localhost。建议通过 SSH 隧道或云平台的端口转发访问，**不要直接把这些端口暴露到公网**。

如果本机已经有可用 X display，则不需要执行 `desktop-start`。

### 5. 运行已验证示例

悬停基线：

```bash
./scripts/amsim run hover
```

机械臂运动实验：

```bash
./scripts/amsim run arm_motion
```

面向用户的正式运行默认开启可视化。如需显式无界面批处理：

```bash
./scripts/amsim run hover --headless
```

每次完成的实验都会在 `runs/` 下生成独立结果目录，通常包含：

- 场景配置快照
- 模型快照
- 运行 manifest
- rosbag2 数据
- PX4 ULog
- recorder 日志
- 场景专属结果文件

## 常用运行命令

启动已验证的两连杆模型：

```bash
./scripts/amsim start
./scripts/amsim check
./scripts/amsim gui-start
```

查看状态：

```bash
./scripts/amsim status
./scripts/amsim gui-status
```

正常停止：

```bash
./scripts/amsim gui-stop
./scripts/amsim stop
./scripts/amsim desktop-stop
```

## 可配置机械臂

查看当前可用模型：

```bash
./scripts/amsim model-list
```

构建仓库中提供的三连杆示例：

```bash
./scripts/amsim model-build x500_serial_3link_example
```

启动三连杆模型：

```bash
./scripts/amsim start --model x500_serial_3link_example
./scripts/amsim gui-start
```

运行时会根据配置动态暴露全部机械臂关节，例如：

```text
/arm/arm_joint_1/cmd_vel
/arm/arm_joint_2/cmd_vel
/arm/arm_joint_3/cmd_vel
/arm/joint_states
```

详细说明见 `docs/CONFIGURABLE_MODELS.md`。

## 公共实验 API

实验代码不需要直接操作底层 PX4/Gazebo topic。

```python
from amsim import ScenarioContext

with ScenarioContext() as ctx:
    ctx.vehicle.takeoff(2.0)

    ctx.arm.move_position(
        "arm_joint_1",
        -5.0,
        unit="deg",
        max_speed=1.0,
        speed_unit="deg/s",
    )

    obs = ctx.observe()
    print(obs.vehicle.z)
    print(obs.arm)

    ctx.vehicle.land()
```

默认情况下，飞行器状态来自 **PX4 估计值**。

只有显式请求时才会返回 Gazebo ground truth：

```python
obs = ctx.observe(ground_truth=True)
```

详细 API 见 `docs/PUBLIC_API.md`。

## 创建自定义场景

查看已有场景：

```bash
./scripts/amsim scenarios
```

创建新的用户场景模板：

```bash
./scripts/amsim new-scenario my_experiment
```

系统会在 `scenarios/` 下生成 Python 实验文件和 YAML 配置文件。

运行：

```bash
./scripts/amsim run my_experiment
```

更多说明见：

- `docs/CUSTOM_SCENARIOS.md`
- `docs/RECORDING.md`

## 当前已验证的软件基线

| 组件 | 已验证版本 |
|---|---|
| 操作系统 | Ubuntu 22.04.3 LTS |
| Python | 3.10.x |
| ROS | ROS 2 Humble |
| Gazebo | Gazebo Sim 8.15.0 |
| ros_gz | 0.244.12-3jammy |
| rosbag2 | 0.15.17 |
| PX4 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Micro-XRCE-DDS Agent | 2.4.2 |
| clean-room GPU | NVIDIA RTX 3080 Ti |
| clean-room NVIDIA 驱动 | 580.142 |

经过验证的 `px4_msgs` 接口快照已经固定保存在：

```text
vendor/px4_msgs
```

该快照**不会**直接根据当前 PX4 的 `msg/` 目录重新生成，因为平台运行所需的 ROS 消息接口与固定 PX4 source tree 中现有消息目录并不完全一致。

## 故障排查

优先执行：

```bash
./scripts/amsim setup --check-only
./scripts/amsim doctor
./scripts/amsim desktop-status
./scripts/amsim render-status
```

### `doctor` 提示缺少 ROS / Gazebo / MicroXRCEAgent

重新执行：

```bash
sudo ./deploy/host/install_ubuntu22.sh
```

### 服务器有 GPU，但 `render-status` 没有 ready

先检查：

```bash
nvidia-smi
./scripts/amsim desktop-start
./scripts/amsim render-status
```

### `direct_renderer` 显示 `llvmpipe`

这表示 direct X/OpenGL 当前使用的是 Mesa CPU 软件渲染。

如果远程服务器有 NVIDIA GPU，并且：

```text
virtualgl_ready: true
```

则应优先使用自动选择的 VirtualGL GPU 路径。

### 上一次运行被异常中断

清理本平台拥有的进程：

```bash
./scripts/amsim gui-stop
./scripts/amsim stop
./scripts/amsim desktop-stop
```

## 文档

- [PID + 前馈控制包与调用接口](control_packages/pid_ff_wholebody_v1/README.md)
- [末端协同闭环接入与验证](docs/PID_FF_WHOLEBODY_V1.md)
- [X650 PID/前馈与上层 MPC 控制包](control_packages/x650_wholebody_v1/README.md)
- [X650 精密定位与分阶段实验协议](simulations/x650_wholebody_mpc_v1/README.md)
- [X650 上层响应辨识结果重放与绘图](docs/X650_UPPER_RESPONSE_ANALYSIS.md)
- [X650 水平响应链与二阶模型开发诊断](docs/X650_RESPONSE_DYNAMICS_DIAGNOSTICS.md)
- [详细安装指南](docs/INSTALLATION.md)
- [可移植性设计](docs/PORTABILITY.md)
- [Clean-room 验证报告](docs/CLEAN_ROOM_VALIDATION_2026-10-02.md)
- [公共 API](docs/PUBLIC_API.md)
- [自定义场景](docs/CUSTOM_SCENARIOS.md)
- [数据记录](docs/RECORDING.md)
- [可配置机械臂模型](docs/CONFIGURABLE_MODELS.md)
- [PX4 HIL 使用与验证](docs/HIL.md)
- [Docker 使用说明](deploy/docker/README.md)
- [Docker CI 验证报告](docs/DOCKER_VALIDATION_2026-10-02.md)

## 当前范围

### 已验证

- PX4 SITL
- Gazebo Harmonic
- ROS 2 Humble
- 远程 NVIDIA GPU 可视化
- 已验证两连杆机械臂
- 可配置 N 连杆机械臂
- 公共控制/观测 API
- 场景运行器
- rosbag2 + PX4 ULog 数据记录
- clean shutdown
- Docker headless 镜像构建与完整 hover 运行（GitHub Actions）
- HIL 软件数据链路回环：Gazebo sensors → MAVLink HIL → actuator → Gazebo

### 尚未宣称已验证

- Docker 内 NVIDIA GPU / VirtualGL GUI（Docker headless 已验证）
- 实体 PX4 HIL（软件 HIL bridge 已实现并通过回环测试，真实飞控仍需接入验证）
- 实体无人机硬件

`legacy/v3_reference` 是冻结的历史参考工作流，产品化过程中不要修改。
