# TS20-50 资料接入与 MPC 执行器验证

日期：2026-10-22。用户确认两个关节使用 **金刚科技 TS20-50，供电 24 V**。源资料固定为 [ZHUOZHUOO/TS20](https://github.com/ZHUOZHUOO/TS20) 的 `a97ef4f227ab778f9efdf86ff93ee309d7d35a22`，包括厂家数据手册、V1.1 通信手册及 `models/ts20_50` 条件响应模型。

本轮完成外部模型加载接口、MIT 参考适配、软件测试以及旧 MPC 命令的条件重放。它为新的执行器阶段提供诊断，不改写原八字成绩；尚未取得 24 V 实际连杆装配下的整机闭环证据。

## 1. 手册、实测模型与本机条件

| 资料/条件 | 核对结果 | 使用方式 |
|---|---|---|
| 型号 | TS20-50，与用户确认一致 | 不继承到 TS20-100 |
| 厂家额定电压 | 24 V DC | 与用户计划一致，不代表响应模型已在 24 V 验证 |
| 冻结辨识条件 | 约 20 V、2 A 电源限制、原 PLA 小输出盘 | 24 V、连杆重力/惯量下须重新验证 |
| 厂家规格 | TS20-50 额定 0.7 N·m，峰值 2 N·m/300 ms；空载最高 31.4 rad/s；35 g | 不是完整持续力矩—速度包络；不能拿编码范围作工作限值 |
| 最新通信依据 | V1.1，固件族 3.4.6；RS485 MIT 力位混合控制 | 设备版本与配置读回后才落成实机参数 |
| MIT 输入 | q_ref、dq_ref、tau_ff、Kp、Kd | 位置、速度和前馈分量独立；不等价理想速度执行器 |
| 发送周期 | 厂家要求 1–10 ms 重复指令；本轮候选 5 ms | 外层 MPC 周期与总线周期分离 |
| 波特率/增益 | 新手册出厂 1 Mbps，Kp/Kd 标度由设备 0x4B/0x4C 读回 | 不能把旧手册或单台台架配置当成所有设备默认值 |
| 角度 | MIT 回报为转子角按带符号减速比折算的接口坐标 | 本轮将它当作关节角只作敏感性研究；独立法兰角度仍需验证 |
| 实测证据 | 上游汇总 210 条历史完整轨迹，未发布原始序列 | 可以使用冻结模型，不能自行宣称已独立复现实测误差 |
| 力矩/温度 | 回报观察器和条件温度趋势模型 | 不用于本轮物理扭矩饱和、轴端 Kt 或热安全保证 |

资料位置：旧数据手册 `docs/servo-datasheet-v0.1.pdf`；更新的厂家手册位于上游 `docs/official/2026-09-24/`。通信结论以新手册为准。模型 API 及边界见 [上游模型说明](https://github.com/ZHUOZHUOO/TS20/blob/a97ef4f227ab778f9efdf86ff93ee309d7d35a22/models/ts20_50/README.md)。

## 2. 实际使用的模型

上游冻结模型不是旋翼或电机内部电磁模型，而是 MIT 命令到回报 q/dq 的事件摩擦接口模型：

```text
u_d = ZOH(Kp*q_ref + Kd*dq_ref + tau_ff)(t-d)
q_dot = v
Jbar*v_dot = u_d - Kp*q - (bbar+Kd)*v - offset
             - slip_sign*(Fcbar + delta*exp(-(abs(v)/vs)^2))
```

零速时若 `abs(u_d-Kp*q-offset) <= Fs` 则保持；严格越过阈值才起动。运动中通过真实零速事件切换分支，不人为把小速度强制清零。

冻结的有效延迟约 3.346 ms，包含合成的指令/控制/遥测影响。参数 `J_total_over_ka`、摩擦和偏置在命令归一化尺度内，不能直接作为 Gazebo 实体惯量或物理 N·m。尤其不能把连杆 kg·m² 直接加到 `J_total_over_ka` 上。

本轮不对参数做 24/20 电压缩放，也不加载 `conditional_candidates.json` 中尚未确认的物理尺度作为默认物理参数。原 MPC 的 24 ms 也不直接叠加到 3.346 ms；两者的计时口径不同。

## 3. 当前速度接口为何必须改

旧预测链中，关节命令经过管线延迟后直接成为实际 dq。TS20 模型的内部 PD、静摩擦与量化都会改变该关系。

按历史有效增益 Kp=0、Kd=3276/32768、tau_ff=0 估算，正向起动要求目标速度约大于 0.755 rad/s。原 MPC 两关节限制为 0.45/0.35 rad/s，纯速度输入可能一直不能从静止起动。这不是降低响应时间常数可以修复的问题。

本轮候选桥接使用：

```text
r(t) = q_initial + integral(u_joint_velocity(t) dt)
q_ref[k] = quantize(r(t_bus[k]))
dq_ref[k] = quantize(u_joint_velocity(t_bus[k]))
tau_ff[k] = 0
Kp = 6
Kd = 3276/32768
```

Kp/Kd 是上游历史条件下的有效值，并非本机已读取/批准的驱动配置。位置参考和速度参考分别采样保持，量化后的 dq_ref 不一定等于 held q_ref 的时间导数。

本轮采用 5 ms 固定总线网格；原非等距速度指令在总线时刻的积分按原始 ZOH 分段精确计算。不同控制切换点不额外插入总线包，避免制造过短发送间隔。位置和速度约束同时检查原参考与量化值，越界即拒绝，不静默限幅。

## 4. 可调用的软件接口

新增 `control_packages/x650_wholebody_v1/x650_wholebody/ts20.py`，仅用于离线计算：

```python
from x650_wholebody.ts20 import FrozenTs20Model, build_mit_commands

motor = FrozenTs20Model(ts20_source_root, source_manifest)
bridge = build_mit_commands(
    original_command_times, original_velocity_commands, q_initial,
    bus_period_s=0.005, kp=6, kd=3276/32768,
    qlimits=([-0.75, -0.35], [0.45, 0.85]),
    vlimits=[0.45, 0.35],
)
trial = {
    "input_time_s": bridge["input_time_s"],
    "observation_time_s": query_times,
    "initial_time_s": query_times[0],
    "initial_state": [q_initial[0], dq_initial[0]],
    "command": {key: values[:, 0] for key, values in bridge["command"].items()},
}
motion = motor.simulate_motion(trial)
```

TS20 上游保持外部依赖，不复制数值实现进本项目源码。manifest 记录源提交和 `models/ts20_50` 全部 12 个源码/参数文件的 SHA-256；加载和预测前校验身份。模块用唯一包别名加载已验哈希的 Python 字节，不会运行上游硬件脚本或示例。

q、dq、tau_ff 分别按 `2*pi/2^24`、`1/512`、`1/2048` 分辨率及 C 的半值远离零规则量化。Kp/Kd 未做假定设备标度的编码；真实 0x4B/0x4C 读回后另行处理。

## 5. 本轮验证与结果口径

新增 11 项测试覆盖非等距命令积分、总线端点、量化、限位拒绝、源文件身份与变更检测、独立包加载和计算预算传递。测试使用自有合成夹具，不代替电机实验。服务器新容器已完成该测试。

实际响应检查使用上游事件模型，比较 1 秒内正负 0.2 rad/s 参考：纯速度桥接与积分位置+速度桥接。具体输出记入 `interface_smoke.json`，属于模型计算，不是新实测。

现有 MPC 小幅、普通八字记录由 `analyze_x650_ts20_command_replay.py` 重放，流程为：

1. 重查完整原生验收与原始输入哈希。
2. 从各记录首个实际 q/dq 初始化 TS20，自由传播后续运动。
3. 把原 MPC 速度指令转成假定 MIT 指令序列。
4. 保持原机体位姿日志，只用 TS20 预测关节重建条件末端位置。
5. 报告关节差异、相对原末端的位置变化及相对原参考的条件误差。

**这不是重新闭环的 PX4/Gazebo 飞行。** 新关节运动没有回到原 MPC、机体或 EKF 中，因此不能把条件末端误差当成新的整机成绩，也不能将它与旧 PID 排名。该诊断只用来量化执行器假设改变的影响。

已通过的最终运行位于 `conditional_replay_v3/`，摘要见 [软件验证记录](validation/ts20_20261022/software_validation_summary.json)。保持原机体日志时，仅关节响应改变造成的末端位置变化 RMS 小幅为 **2.797 mm**、普通为 **2.988 mm**；峰值分别 **3.749 mm／3.754 mm**。它们是相对原末端的差异，不是新的闭环误差或性能改善量。两关节相对原日志的位置差异 RMS 小幅约 **8.227／10.066 mrad**，普通约 **10.353／10.409 mrad**。

正负 0.2 rad/s、1 秒模型检查中，纯速度配置两关节均保持静止；积分位置+速度配置末端关节角约 +0.19183／−0.19190 rad。该检查只验证当前假定增益与模型的行为，没有接硬件。

服务器工作区为：
`/root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/ts20_integration_20261022/`。
上游副本与逐项身份清单位于 `upstream_snapshot/`；有效重放目录、统计和曲线由本次执行报告记录。

新容器中的 amsim 虚拟环境 NumPy 与系统 SciPy 不兼容，本轮数值计算使用数据盘既有、兼容的 `/usr/bin/python3`（NumPy 1.21.5 / SciPy 1.8.0），没有下载或覆盖依赖。首次失败日志保留。

```bash
env -u PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 \
  scripts/analyze_x650_ts20_command_replay.py \
  --source artifacts/ts20_integration_20261022/upstream_snapshot \
  --manifest artifacts/ts20_integration_20261022/upstream_snapshot/source_manifest.json \
  --accepted-root artifacts/x650_figure8_prediction_chain_20261022 \
  --output NEW_CONDITIONAL_REPLAY_DIR
```

## 6. 如何进入新的 MPC 预测和物理仿真

下一版预测要包含关节位置、角速度、积分位置参考 r、在途命令历史与采样年龄。候选输入仍可为两关节上层速度，但预测必须经过同一 MIT 适配规则，再传播舵机状态。原历史协议不自动接受这个修改。

先从固定输入段检验 20、100、300、600 ms 等预报时域。可先建立适合 QP 的局部伺服近似，把摩擦残差和静止分支另作检验；用于 NMPC 的平滑近似也需要对上游事件模型及实际加载数据交叉验证，不能为了求解方便直接删除起动阈值。

最后需要完成两类物理证据：

- 24 V 和最终连杆装配下的 MIT 命令→实际关节运动，覆盖正负、换向、工作角度、停止和温度；建立独立验证记录。
- 若要用真实力矩驱动 Gazebo 刚体，标定可信的力矩映射和装配参数；或者明确采用经加载辨识的闭环关节响应插件，并验证它对机体耦合的适用性。额定峰值和归一化惯量都不能代替该标定。

完成后，在新的工况身份下重新验证悬停摆臂与末端保持，再推进轨迹。真机前总体路线及同接口 NMPC 的比较设计见 [验证框架](X650_REAL_FLIGHT_READINESS_AND_NMPC.md)。
