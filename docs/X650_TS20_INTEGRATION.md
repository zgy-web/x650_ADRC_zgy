# TS20-50 资料接入与 MPC 执行器验证

日期：2026-10-22。用户确认两个关节使用 **金刚科技 TS20-50，供电 24 V**。源资料固定为 [ZHUOZHUOO/TS20](https://github.com/ZHUOZHUOO/TS20) 的 `a97ef4f227ab778f9efdf86ff93ee309d7d35a22`，包括厂家数据手册、V1.1 通信手册及 `models/ts20_50` 条件响应模型。

第一阶段完成外部模型加载、MIT 参考适配以及旧 MPC 命令的条件重放。随后新增了事件关节预测、独立数值验收、Gazebo 条件执行器和单独的新机型。原八字成绩保持原来的工况身份；24 V 实际连杆装配下的整机闭环证据仍需取得。当前阶段按“新关节预测链 → 新悬停与摆臂 → 完整末端保持”推进，未开放快速八字。

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

TS20 上游源码保持外部依赖。manifest 记录源提交和 `models/ts20_50` 全部 12 个源码/参数文件的 SHA-256；加载和预测前校验身份。模块用唯一包别名加载已验哈希的 Python 字节，不会运行上游硬件脚本或示例。新的在线预测器使用本项目实现的数值核，并对这个固定外部参考求解器独立校验；上游实现没有作为源码打包进本仓库。

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

## 6. 新预测链和单独的仿真工况

`ts20_prediction.py` 传播关节 q/dq、连续积分参考 r、实际 MIT 包、固定总线时钟和测量年龄。MPC 输入仍是 `u=[vx,vy,vz,dq_cmd1,dq_cmd2]`。r 是适配器记忆状态，由实际消费的上层关节速度积分；不在每次求解时重置成测量 q。候选序列经过同样的积分、5 ms 采样、量化和一次合成延迟，再计算未来 q/dq。

QP 的局部映射为 `q_i = qbar_i + Q_i*z`、`dq_i = dqbar_i + D_i*z`，其中 z 为按各通道速度上限归一化的候选序列。名义状态由完整静摩擦/起动/换向事件模型传播。局部灵敏度与这个非光滑模型有差别，因此不能只依靠 QP 的可行状态；发布前必须用事件模型复核候选轨迹和数值子步内的极值，并保留局部信赖域、位置参考限位和原停止/在途保护。拒绝的候选指令不会写入实际命令历史。

当前关节信赖域上限冻结为归一化 0.02，基体三个输入仍按原速度/变化率约束优化。名义 0.58 s 时域中，`0.58 × 0.45 × 0.02 = 5.22 mrad`，用于把局部关节预测变化控制在原 6 mrad 后验误差预算以内；它是保守的开发数值域，不是最优控制性能结论。位置/角速度后验门槛仍为 6 mrad／0.10 rad/s，完整数值包络另加独立验收的 0.3 mrad／0.03 rad/s 裕量后检查实际限制。必要时最多重新线性化 3 次，整轮 MPC 总耗时超过 50 ms 即拒绝返回指令；外层还在第一条 joint/PVA 发布前检查适配器和返回日志在内的总耗时。停止任务后的恢复/降落使用独立物理反馈流程，避免实验输入失效阻断恢复。

机体水平二阶、垂向一阶响应继续作为旧工况的先验使用，明确标记 `base_response_is_prior=True`。它没有因为新执行器的软件测试通过而变成新整机辨识结果。位置/速度/加速度、姿态和末端几何关系仍通过同一上层接口计算；没有切换到轴端关节力矩或全身刚体动力学 NMPC。

新增 `ts20_runtime.py` 只读取当前控制时刻之前的完整记录：物理关节 q 与同时间戳的条件舵机端点角速度回报进入预测；Gazebo 的 4 ms 区间平均角速度另作审计。两种速度有不同时间定义，不能直接当作同一个量比较。这个模拟回报接口尚未验证真舵机的速度估计/滤波特性。内部自由模型 q 与实际物理 q 不一致时，新实验拒绝继续用自由运动假设。

末端反馈由 PX4 估计位置／姿态与上述物理关节 q 重建，`q_ref` 和积分参考 r 不替代反馈 q。历史旧运行的 `feedback_snapshot.control_state` 关节字段是在 TS20 替换前读取的 ROS JointState，不能当成最终 TS20 观测。新的上层因果 provider 路径直接用已选择的 TS20 物理 q／条件端点 dq 构造状态，因此该路径的 `feedback_snapshot.control_state` 与实际交给控制器的关节观测一致；原 ROS JointState 单独进入 `raw_ros_joint_diagnostic`，不供控制使用。复核具体运行仍应按其源码身份读取 `control_execution_trace.jsonl` 的 `state` 和 MPC 的 `prediction_measured_joint_*`，独立 native 末端真值继续用于效果验收。

新的因果入口只在上层 PX4 estimated feedback 活跃时启用：每轮先以固定的 estimator `/clock` 调用一次 `select_joint_observation`，只接受不晚于该时刻、源年龄不超过 25 ms 的完整实际记录。q、端点 dq、r、已读 MIT 包、消费指令历史和总线原点来自同一个私有快照；缓存中的 row／packet 复制后不可变包装，指令值与历史快照使用不可变 tuple，后续补写、裁剪或同时间戳事件替换不能改写本轮证据。公开观测保留不可变数值和 generation，绑定时还校验 runtime 持有的 opaque 对象身份；新选择开始即撤销旧 token，选择失败也不能沿用旧快照。对外 context 重新生成普通字典与独立数组，兼容 MPC 的冻结复制。

状态构造与 `after_state_read` 传递同一观测，不再二次读 CSV／选择，也不先把源时刻 q/dq 投影到现在后再由预测器重复传播。机体 p／R／v／角速度／加速度保持 PX4 来源，truth 仅作独立物理监督和验收。上层调度、任务计时使用 estimator `/clock`，物理监督时钟不再由未用于控制的原 ROS joint header 推进；非上层的悬停／关节 commissioning 保持原来的物理反馈入口和时钟。默认 legacy 分支的未来测量保护不变，真实 TS20 未来／过龄、序号、缺包与内部／物理 q 不一致仍拒绝；恢复先禁用上层实验并使 token 失效，不依赖失败的 TS20 日志读取。此修改在本地只读审查与 42 项合并回归之后，已完成第 11 节所述的新版服务器软件验收。新悬停与关节探针已通过本轮独立验收；新的完整保持因控制间隔超限仍未通过，快速轨迹未运行，软件证明不等于取得完整保持准入。

过去输入使用插件实际消费和实际发包的时间。对尚未发布的候选输入，当前预测仍假设它从当前控制快照时刻开始进入上层接口；计算和 ROS 传输耗时尚未独立补偿，明确标记 `upper_computation_and_ros_transport_delay_compensated=False`。3.346 ms 只在 MIT 发包之后应用一次，不能当作已覆盖整条上层管线的证据。新日志会记录控制快照到插件消费的间隔，后续按这一口径评估需要的补偿。

新机型为 `amsim_urdf_x650_2link_ts20_v1_flight`，单独使用 PX4 airframe 4997。原质量、惯性和碰撞几何保持不变；原两路理想 JointController 替换为 TS20 条件响应插件。物理施加采用每个物理步的预测位移除以步长，保留 Gazebo 连杆对机体的惯性反作用，但不提供可信的轴端力矩限制、带载响应或接触动力学。发生接触时必须检查实际 q，不能用内部自由运动 q 替代它。

`simulations/x650_ts20_hold_v1` 使用单独的协议、执行器身份与验收记录。旧工况的悬停/摆臂/轨迹成绩不会自动打开新工况的阶段门。每个新阶段仍需完整 native 物理记录、实际指令包、ULog、参数恢复和全时段时序检查；快速八字保持关闭。

## 7. 已完成的预测与地面接口检查

固定的 8 组独立数值试验覆盖静止、小阶跃、正负运动、双关节换向、正弦和异相切换。20、100、300、600 ms 及整段预测都只使用同一个首态；后续真实 q/dq 只作误差评价，不用于不断修正预测。冻结门槛为最大位置差 0.3 mrad、角速度差 0.03 rad/s。

初版仅 4/8 通过，失败证据保留。改用指数中点摩擦积分后，v2 的 8/8 通过；最坏位置差约 0.1032 mrad，最坏角速度差约 0.01569 rad/s。物理参数、增益、总线周期、延迟和验收门槛均未改变。C++ 数值加速器另须验证库字节身份、ABI、与 Python 参考核的一致性及同一套 8 组独立试验；不能仅用较快耗时代替数值验收。

独立 Gazebo 地面接口检查完成两关节实际正负运动：2750 个 4 ms 物理步、2200 个 5 ms MIT 包，内部/实际 q 最大差分别约 2.26e-9、2.80e-9 rad，总线网格最大误差约 9.95e-16 s。这是接口和方向检查，不是飞行验收。先前大摆幅触地的失败、短反向输入被静摩擦保持的未通过记录一并保留。

数值和地面接口检查不等于末端保持通过。新阶段最终结果由 `x650_ts20_conditional_stage_certificate_v1` 记录；不完整、求解失败、超时或参数未恢复的运行不能生成通过证书。

新摆臂记录的整段自由预测会输入该次真实 MIT 包序列，从同一个首态连续传播，仅在数值接口的分段上携带前一段预测状态；后续测量不作重置。这检验命令到关节响应的预测链，不等于在线 MPC 每一条未执行候选都已通过独立预测验收。末端保持还要另行检查真实滚动求解、实际下发、全时段末端误差与时序。

## 8. 新空中记录发现的粘滑与阻尼设计

4997 原增益工况的新悬停通过完整独立验收：20.000 s、5001 个 native 点，机体三维位置 RMS 1.725 mm、峰值 2.952 mm，最大发送间隔 24 ms；参数恢复和全窗 ULog、实际 MIT 包均通过。

接着的 30.004 s 空中摆臂完整执行、安全着陆并恢复参数，但两关节实际峰速 0.30723／0.30651 rad/s 超过事先冻结的小摆臂门槛 0.20 rad/s，**该次探针失败，没有发放通过证书，也没有准入末端保持**。关节参考跟踪 RMS 3.218／3.541 mrad、机体最大偏差 10.887 mm、最大倾角 0.1476 度等其他指标通过，不能代替速度门槛。原始记录和完整失败证据保留在服务器 `artifacts/ts20_chain_hold_20261022/joint_probe_failed_v1_diagnostic/`。

这条全新实际记录验证了关节预测链：以首态和实际 MIT 包自由延续整个测量窗，不用后续测量重置，q 峰值预测差 0.09964／0.09800 mrad，dq 峰值差 0.01525／0.01478 rad/s。预测器预报到了粘滑脉冲；新的控制响应仍需改善。落地接触以后的自由运动/物理位置分离不属于这段测量窗，不能混入其预测成绩。

对滑动分支 `F(v)=sign(v)*(Fc+delta*exp(-(abs(v)/vs)^2))`，速度非零处最强负斜率大小为：

```text
max(-F'(v)) = sqrt(2/e)*delta/vs = 0.17155278
d_incremental(v) = bbar + Kd + F'(v)
local characteristic: lambda^2 + d_incremental/Jbar*lambda + Kp/Jbar = 0
```

历史 Kd 对应 `bbar+Kd=0.10417049`，小于上述负斜率界，不能保证所有滑动速度附近的增量阻尼为正。此结论解释低速粘滑的可能机制，不等于证明整机必然失稳。新的**条件仿真**候选保留 Kp=6、全部物理参数及输入/验收门槛，只选有效 Kd=0.25，使 `bbar+Kd=0.25419491` 达到该斜率界的 1.48173 倍，满足预先选定的 1.4 倍裕量。未把它当成真舵机已读取或已批准的增益编码。

固定该次实际消费的上层速度和连续 r，仅替换 MIT 有效 Kd 的离线反事实计算中，速度峰降至 0.113014／0.112727 rad/s，起动次数从 49／56 降至 8／8，换向保留。r 与 q 的 RMS 差反而略增约 0.09／0.13 mrad，不能宣称定位精度已经改善。这只是同输入诊断，新的反馈关节指令、机体运动和末端结果都须重新闭环验证。

新增益使用独立的候选 profile、gain-specific oracle、模型 `amsim_urdf_x650_2link_ts20_damped_v1_flight` 和 airframe 4996。原来的 4997、失败记录及历史协议保留；原 0.20 rad/s 小摆臂门槛和 50 ms 时间门槛不放宽。只有同一新增益通过独立数值检查、新悬停及新摆臂之后，才能开展末端保持。

新增益已单独完成 8/8 独立数值检查，最大 q/dq 差分别为 3.078545 μrad／0.00079073 rad/s，仍使用原 0.3 mrad／0.03 rad/s 门槛。完整 15 节点 MPC 软件矩阵重新验证 30/30 通过，最大耗时 47.3256 ms；每条预测和 MIT 包均显式使用 Kp=6、Kd=0.25。原权重、速度/变化率约束、局部信赖域和后验误差门槛未变。软件包回归 106/106、独立协议测试 9/9 通过。这些软件成绩不代替新飞行的实时性和末端验收，精简证据见 [新增益软件验收记录](validation/ts20_prediction_chain_20261022/damped_software_acceptance_summary.json)。

新增益的新悬停和 30.008 s 摆臂已经完成独立审计；摆臂实际速度峰值为 0.105629／0.105753 rad/s，低于原 0.20 rad/s 门槛。整段首态自由预测的最大 q 差为 6.237e-7 rad、最大端点 dq 差为 4.975e-5 rad/s；机体最大偏移 10.0011 mm、最大倾角 0.144061 度。首次审计因旧配置名称硬编码而拒绝新模型；修复后先将整份物理 profile 与绑定的准备报告比较，再复用原几何/FK门槛，重新审计完整原记录，未改写原数据或源归档。

首次安静 MPC 保持在发布任何正式测量指令前被快照保护中止：观测选择 q/dq 后，CSV补写了一条仍早于控制时刻的状态；原 context 再读日志并选择新行，与已观测快照不一致。`ts20_runtime.py` 现为每次成功观测缓存精确 q/dq/r 和控制时钟，context 不再刷新 CSV；下一次观测才接纳新完整行及历史。q/dq/时间戳/控制时钟必须匹配，读取失败会使旧快照失效，原年龄、缺包和物理一致性保护保留。新增延迟补写与篡改/失败拒绝测试后，服务器完整套件运行 108 项，107 项通过、1 项可选原生核测试跳过；独立原生核验收记录仍另行保留。失败 run `x650_ts20_damped_hold_v1_20261022_222212_10301` 已安全落地、恢复参数，没有保持通过证书；修复后的保持必须重新取得完整记录。

修复快照后的 quiet v2 成功发布了 1 次 MPC 指令，但记录的整轮计算为 47.2862 ms，下次实际控制间隔为 56 ms，超过冻结的 50 ms，实验按保护规则停止并正常着陆、恢复参数。该次 QP 本身只用约 0.3288 ms；控制器调用约 45.4233 ms，故需继续定位预测/历史管理等开销。`x650_ts20_damped_hold_v1_20261022_223143_11458` 的完整失败记录保留，不属于末端保持通过结果。两次失败及已通过的悬停/摆臂摘要见 [原生验收记录](validation/ts20_prediction_chain_20261022/flight_acceptance_summary.json)。

精确重放该次约 38 s 的前序观测发现，原实现直到首次 context 才一次释放 9532 条状态、7626 个 MIT 包和 1320 个消费事件，首次 context 约需 8–11 ms。现将同样的“0.75 s 历史加一个前驱”维护提前到每次成功的因果观测；未来行仍保留，完整 CSV 与序列/时钟验证不变。真实逐观测重放中，缓存最大为 189／151／42 条，最终 context 完全相同；75 个预测控制量与 180 个预测状态对原记录逐值一致。交错旧/新/旧的独立首步回放显示新 context 约 0.125–0.304 ms，完整步约 18.67–19.43 ms。新增无 MPC 调用的长 prestream 测试后，109 项套件为 108 通过、1 项可选跳过。

这项缓存修复尚未使整套时序验收通过：随后 30 个原软件工况有 1 个 `+5 mm / age=0 / moving_warm` 达到 51.064 ms，被原总截止时间拒绝，结果为 29/30。其关节误差和约束检查通过，不能替代时间门槛；原失败记录保留，继续优化之前不会准入新保持。回放、源码身份和本次未通过的矩阵见 [增量缓存时序记录](validation/ts20_prediction_chain_20261022/incremental_runtime_timing_summary.json)，不作真机实时性声明。

## 9. TS20 专用约束视图与服务器软件验收

对上述 `+5 mm / age=0 / moving_warm` 工况的剖析表明，约束组装与 SLSQP 在该工况仍有较大开销。新增 `qp_constraints.py` 只精简传给求解器的等价约束视图：字节完全相同的行取区间交集；所有单变量原始行保留；利用这些行推导外向舍入的区间，并加保守浮点误差守卫，只有严格落在原界内的其他行才可省略。近边界、极小非零系数、无界或溢出的证明不会被近似处理。

这个候选仅由 `Ts20MpcController` 显式启用。默认 `MpcController` 返回原 C／lo／hi 数组对象和行序；原完整约束仍用于 primal／KKT 解质量检查，H／g、初值、求解器设置、非线性后验、权重和 50 ms 截止时间均不变。它不修改关节数值核、物理模型、增益或既有阶段的条件身份。

2026-10-22，本地既有 Python 3.12.14／NumPy 2.3.5 环境的 7 项测试通过，覆盖精确重复行、负系数、signed-zero、极小系数、近边界、不可行约束、溢出保留与随机可行集一致性。当时服务器离线，本地环境没有 SciPy，因此控制器选择测试及完整 MPC 回归尚未执行。该阶段的源码身份和边界保留在 [本地候选记录](validation/ts20_prediction_chain_20261022/qp_constraint_view_candidate_summary.json)，不改写为后来的服务器成绩。

2026-10-23，新容器 `autodl-container-f64b45a5b8-51661812` 恢复连接后，先按字节归档旧源码、原矩阵脚本及 29/30 失败记录，再只同步 TS20 文件。原预测核、native library、物理模型、增益与门槛不变。117 项服务器回归为 116 通过、1 项可选跳过；原冻结 30 工况脚本全部执行，30/30 通过，最大计算耗时 27.685 ms。该结果只打开全新安静末端保持的准入门，未构成完整保持或真机验收，快速八字仍未准入。

同一个原失败工况中，原完整约束视图和精简视图的 H／g／A／b／C／lo／hi 逐值相同；75 个归一化计划量、180 个预测状态、目标函数、完整非线性 q／dq 和编码后 MIT 包均为零差异。求解器约束由 305 行减为 130 行，合并 35 条精确重复行，省去 140 条被保留单变量约束严格蕴含的行。两侧原 full C 检查及非线性包络均通过，primal violation 为 2.776e-17，KKT stationarity 为 4.261e-17。

同容器、fresh controller 的交错计时中，原视图整步为 29.788／29.236／30.169 ms，精简视图为 26.435／35.830／26.217 ms；35.830 ms 的较慢记录完整保留。对应 QP 求解约由 9.25–9.89 ms 降为 6.65–6.74 ms。不能把旧容器 51.064 ms 与新容器成绩之差全部归因于算法：新容器配额为 18 核，环境和全部计时另行记录。证据、部署身份和原失败 SHA 见 [服务器软件验收摘要](validation/ts20_prediction_chain_20261022/qp_view_software_acceptance_20261023.json)。本地 JSON 是服务器摘要的格式化副本；服务器原文件 SHA 为 `da4a4d50748f10527ad8648c1f30cd448bc76005db73e59162ec287fa44c8197`，不声称字节相同。

软件通过后已执行全新的 quiet v3，但完整 30 s 验收仍未通过：第 589 次成功发布之后，在 t=19.808 s、dt=0.040 s 的第一子问题中，非线性一致性和实际约束通过，内部总耗时却达到 53.140568 ms，故原 50 ms 保护拒绝生成／发布新指令。先前成功发布周期的最大计算耗时 32.759283 ms，最大间隔 44 ms。已执行发布覆盖的 19.744 s、4937 个 native 点仅作 partial 诊断，末端三维 RMS 3.04747 mm、峰值 4.78237 mm、最大倾角 0.111107 度，不能替代完整窗口成绩。落地、解除锁定、typed 参数恢复与同步保存、42 项 PID、运行时关闭均独立核验；正式审计拒绝，未发放通过证书。详见 [quiet v3 失败记录](validation/ts20_prediction_chain_20261022/quiet_v3_failure_summary.json)。

`simulations/x650_ts20_hold_v1/launch_quiet_v3.py` 保留原 task／控制生命周期／审计，在本次独立平台来源旁证中封存实际 scripts/amsim、场景 YAML、launcher 和 src/amsim Python 文件及 git HEAD／diff。38 个平台源和 47 个控制源的开始／结束范围及字节一致，archive 与 run 绑定通过；3 项 Linux 来源变更／归档篡改守卫测试通过。该旁证只描述新运行，不给旧档案补写历史来源。失败重放包保存了前轮暖启动计划与状态，但当时 CSV cursor 和完整缓存没有序列化，待执行包是否已读有歧义，不能把事后上下文 reconstruction 称为精确重放，也不能仅因重放较快就断言历史超时来自调度。

原完整矩阵脚本在服务器 `artifacts/ts20_prediction_chain_20261022/benchmark_gain_kd025_matrix_v1.py`；先前仅更改输出位置的执行副本为 `artifacts/ts20_chain_hold_damped_v1_20261022/benchmark_incremental_runtime_matrix.py`。本轮 `scripts/validate_x650_ts20_qp_view.py` 先固定原脚本及历史失败 SHA，从原 AST 提取同一失败工况进行等价检查；再运行原脚本的全部语句，仅将唯一结果写入重定向到新目录，原结果保持原 SHA。本地单工况 profiler 不能代替这个完整矩阵。

结果工具 `scripts/plot_x650_ts20_commissioning_results.py` 要求完整通过的实际阶段证书，逐文件核对原始 CSV 与证书 SHA。保持图严格使用证书中的名义 30 s 评分区间，另外保留实际回调结束时刻，避免多纳入一个回调的尾部样本；关节图每次从绑定的实际首态和 MIT 包重新自由推演，不接收外部预测数组替代计算。图中分别标明物理区间平均 dq 与模型端点 dq，后者不当作独立实测速度。6 项合成回归通过；该工具尚未用本轮服务器数据生成正式图，合成测试不构成实验成绩。

`scripts/profile_x650_ts20_actual_step.py` 用于原 quiet v2 首个真实状态的离线回放；`scripts/profile_x650_ts20_matrix_fixture.py` 用于单个 `+5 mm / age=0 / moving_warm` 软件工况定位。它们不启动飞行，不代替完整矩阵或实时验收。当前工具将新输出与原失败记录分开，并记录实际加载代码及库的身份；历史报告中的 harness SHA 指向当时归档的执行版本，不能用本地后来修订的工具 SHA 覆写。服务器恢复后仍须验证当前工具的实际数据读取与结果，不把本地语法或合成检查当成重放通过。

两个诊断工具的语法检查和 `--help` 均通过，且 `--help` 无需 SciPy。7 项离线保护检查确认：已有空／非空输出目录均拒绝，原 marker 和 prefix 不被覆盖，增量读取遵循给定时钟并拒绝已消费行中的非有限数。矩阵单工况工具必须显式提供 `--native-sha256`；实际回放工具支持 `--historical-source-archive` 附加旧执行归档身份。新增等价性验收工具已在服务器完成上述软件检查；修订后实际 CSV 回放工具的主入口及结果绘图仍须另行用真实记录验证。

## 10. FFI 生命周期修正与新的实际阶段准入

对 quiet v3 的晚期状态作了两种待执行包分支的离线检查。包含该包的分支重现了原 q／dq 非线性后验误差，差异分别约 2.06e-16 rad、9.27e-15 rad/s；历史 CSV 读取游标与完整缓存当时没有序列化，不能称为整个运行时上下文的精确恢复。原 53.140568 ms 超时未复现，原因仍未确立，不能据此归因于 GC 或服务器调度。原失败记录及原执行工具字节均保留；当前 profiler 的来源标签已修正，当前工具 SHA 不替换历史报告中的执行 SHA。

干净的指针所有权探针确认，旧 NumPy `ctypes.data_as` 转换会留下 `c_void_p`／dict 小对象循环，等待 GC；它不保留 ndarray 缓冲区。新的同步借用方式从整数地址构造指针，并在整个 native 调用期间用显式 tuple 持有所有数组。只改变 Python／C 接口的内存管理，不改数值核、模型、增益、积分、QP 权重、时域、约束或 50 ms 门槛；也没有关闭或主动触发控制循环中的 GC。

新的 [软件证明](validation/ts20_prediction_chain_20261022/ffi_pointer_software_acceptance_20261023.json) 为服务器原始 JSON 字节，SHA256 为 `dd086c21d80a8907174cb8c78f81698daa33bb1d22ac8a0cb865e21c84379d0c`。它绑定 34 项源码／测试／工具与实际 native 库，并记录：

- 独立 oracle 8/8 通过；旧／新 FFI 的全部数组、MIT 编码包、事件、完整非线性包络与子步极值精确相同。
- 124 项整包回归中 123 通过，1 项未显式供库的可选 native 单测跳过，无失败或错误；另行显式供库的 oracle 已实际执行。
- 原冻结矩阵 30/30 通过，最大 wall time 26.452692 ms；原 29/30 失败结果不变。
- 同一单工况的完整约束与约束视图解在问题矩阵、75 维计划、180 个状态、目标函数、编码包与非线性 q／dq 上均为零差异；所有交错计时均保留，包括 full 侧的 37.025 ms 样本。

`Ts20MpcController` 另外记录 wall／当前线程 CPU 时间及 GC collection 计数，仅供观察；计数不能单独建立超时因果关系。最终拒绝之后才深复制冻结观测、提交前计划／历史和已返回的内层诊断，外层拒绝标志与状态回滚保持有效，被拒绝指令不能发布。

新的 `simulations/x650_ts20_hold_v1/launch_chain_v4.py` 需要显式传入经审查的软件证明路径与 SHA，并逐文件核对当前部署、native／CPP、新 oracle 及回归／矩阵证明；开始／结束都封存并检查实际平台、控制和软件源。7 项新守卫与 3 项既有平台守卫在服务器通过。新 oracle 生成新的实验条件 `1650aa6f30c1cfc13da31251d9418abb8699d6f060c4a05f88d97f5f8b3f67ee`，须重新取得 20 s 悬停、30 s 关节探针与 30 s 安静末端保持证书；软件通过不替代这些实际阶段。旧 quiet v3 启动器与档案不变，快速八字和外力入口仍关闭。

本轮 [fresh v4 实际结果](validation/ts20_prediction_chain_20261022/chain_v4_actual_20261023.json) 已取得同一新条件的悬停和关节证书：悬停完整 20 s、5001 个原生点，机体三维 RMS 1.78913 mm／峰值 3.02078 mm；关节完整 30.004 s、7501 个原生点，q 指令跟踪 RMS 分别 3.20516／3.52490 mrad、方向余弦 0.99627／0.99486，摆臂机体 RMS 4.53993 mm／峰值 10.31086 mm、最大倾角 0.153736°。整段首态自由预测 q 峰误差 6.09287e-7 rad；该数值描述条件模型一致性，不能替代上述指令跟踪误差或 24 V 真舵机精度。

v4 安静保持仍未取得完整通过证书：554 次 measure 发布之后，下一轮进入 MPC 之前的控制间隔达到 56 ms，原 50 ms guard 拒绝继续。最后一次 MPC outer wall／thread 为 17.40108／17.39479 ms，期间 GC collection delta 为零；这些量没有覆盖发布后的尾部日志与下一次反馈读取，不能据此确定历史间隔超限的原因。已执行的 18.244 s 仅作 partial 诊断，独立原生末端 RMS 3.23693 mm、峰值 4.89980 mm。降落／解除锁定、typed 参数恢复／同步保存、42 项 PID 及全部来源档案均核过。该 guard 位于 `ExecutionTrace.controller_step` 之前，本次失败轮的精确反馈快照尚未被记录；保留的 20 轮成功快照与邻近物理行不冒充缺失现场。后续需补齐循环层计时及拒绝快照后继续定位，不能把部分窗口当作完整验收。

已用该次通过的关节证书生成实际图表，并完成原始文件回传与可视检查，详见 [实际图表交付清单](validation/ts20_prediction_chain_20261022/joint_probe_plot_delivery_v4_20261023.json)。一次系统 Python CLI 在 2.446 s 内完成完整自由预测复算与 PNG，使用现成 NumPy 1.21.5／SciPy 1.8.0／Matplotlib 3.5.1，未安装依赖。PNG、NPZ 与两个 manifest 的本地／服务器原始字节 SHA 一致。7502 个 `ts20_state.csv` 物理 q／模型端点 dq 点与 7501 个独立 `native_trace.csv` FK 点分别保留来源，不按行号混同；保持失败没有生成 certified 图。绘图源码绑定实际导入预测器和 oracle，另显式标记基体响应先验身份是否记录；14 项合成守卫及实际数据复算／图像检查都已完成。

## 11. 循环层时序诊断

[离线分项结果](validation/ts20_prediction_chain_20261022/loop_serialization_analysis_20261023.json) 保留 v4 最后 20 个成功周期。19 个可计算的“前次 PVA 本地返回→下次 compute_before”wall gap 为 9.368–12.068 ms，期间仿真时钟与 wall 推进同量级；缺失的失败轮边界仍未知。真实历史 JSON 树及重新生成的 NumPy 诊断树分开测量，primitive 转换候选只节省约 0.35–1.1 ms，未部署该候选。直接改用 `JSONEncoder.default` 有 7 类语义反例，也没有替换生产 helper。这些离线测量不还原历史耗时，不足以宣称修复 56 ms。

该次循环诊断使用 TS20 专用 `loop_timing.py`，只由该场景 extension 显式安装；观测包装本身不改变原调度、发布操作或 50 ms guard。它委托原方法各一次，观察原 pump、callback drain、read、发布 trace 返回和原有文件 write／flush；普通路径不新增磁盘写入或 flush。最近 3 个 due/read 循环、64 个全部 pump 循环保留在有界标量环中，避免漏掉未到控制时刻的等待。真正本地 send 返回的 wall 时间引用原 ExecutionTrace；wrapper 的 CPU 时间明确属于包含 trace 写入的较晚边界，不冒充精确 send CPU 时刻。后续因果 provider 对共享状态入口有显式修改，不能把观测版当时的源码不变描述沿用到新版 provider。

自然 GC 观测只注册自己的唯一 callback，记录 start／stop 区间、线程和 generation／collected，保留最近 64 个完成事件和各代计数／最大值；没有主动 collect、disable 或阈值调整，也不扫描堆。区间包含间隔中的其他 callback，不等于 collector 独占 CPU 成本。收尾先移除自身 callback，并保留原有其他 callback 和 GC 配置。新版 observer 另为三代各保留 wall／当前线程 CPU 峰值的完整事件，共固定 6 槽，使最近事件环驱逐旧项后仍能核查峰值时段；11 项 observer 测试先在本地通过，随后纳入下面的服务器 fresh 回归。

观测版的 10 项 observer 测试与 4 项既有 extension 测试曾在服务器通过，原软件证明的 34 项源及前序证书按原校验通过；随后实际诊断保持失败，见 [完整精简记录](validation/ts20_prediction_chain_20261022/diagnostic_quiet_actual_20261023.json)。386 次 measure 发布之后，`EstimatedFeedback.read` 在 TS20 替换前读取了源时间晚于当前 estimator `/clock` 的 ROS JointState，触发禁止未来关节测量反向传播的保护；该失败轮未返回 read 状态、未调用 allocator。失败的具体 header／clock 数值对没有序列化，因此只报告保护条件，不重建缺失现场。独立 native partial 窗为任务 0.020–13.040 s，共 13.020 s、3256 点；末端 RMS 3.56655 mm／峰值 4.53748 mm 仅是这段部分记录，不能作完整 30 s 保持通过。着陆／解除锁定、typed 参数恢复、同步保存和 42 项 PID 已核过，原始记录及失败身份保留。

[GC 时段核对](validation/ts20_prediction_chain_20261022/gc_gen2_timeline_overlap_v1.json) 记录到一次 generation 2 callback 区间，wall 长 25.323905 ms，位于最后一次 measure PVA 本地返回之后、首次 `restore_folded` joint 发布之前；它与已记录的 controller compute／step 或本地发布操作区间没有重叠。失败 read 本身没有精确边界，不能把该 GC 区间定位为 read 内耗时，更不能据时间相邻断言它导致 ROS 消息先后交错；历史 56 ms 控制间隔失败也是另一次运行，不能借这条记录归因。

读内失败促成第 6 节的因果关节 provider：在解释 ROS JointState 之前选择真实可用的 TS20 源观测，而不是放宽未来／25 ms 年龄或原时序门槛。provider、默认／恢复路径及峰值保留 observer 的 42 项本地合并回归保留为开发证据；随后已取得[服务器 fresh 软件证明](validation/ts20_prediction_chain_20261022/causal_joint_software_acceptance_20261023.json)，该 JSON 保持服务器原始字节，SHA 为 `dfc55fea230a857829a5a109219db1c15e18f46f9f7f54b76d8feb4e95359201`，绑定 86 项实际源码。新执行的 288 项测试由 124 项 package、134 项 interface／audit、30 项 shared-loop 组成，全部通过、0 skip，显式提供 native 测试库。原 30 个矩阵夹具重新运行 30/30 通过，最大 26.684829965 ms；full/view 的矩阵、计划、状态、编码包及非线性输出比较一致，记录的差值均为零。独立 timing 的全部 full/view 记录一并保留，最大值分别 37.344370／26.682980 ms，不以较快记录替代完整结果。

首次 `software_v1` 报告为 248 项、0 failure／4 个同源 ImportError，原因是服务器未带齐已跟踪的 `test_x650_precision_stage_audit.py` 辅助文件；该次没有飞行，失败输出保留。补齐既有 helper 的精确字节（SHA `36c4d9f24185dff7a714ba7b48b282d909c3006cf02f9d014effe06dbea94699`）后，保持控制源码和 driver 不变，在全新 `software_v2` 目录重新执行上述回归与矩阵，没有继承旧 package／matrix 成绩。证明区分静态 before、实际导入后的来源观测和最终 after，并归档实际源；未改变的预测器／数值核、物理条件和 oracle 身份继续逐项核验。

软件证明绑定的是服务器实际执行字节。与本地 Git 的逐项核对发现，预测器测试、native CPP 和 oracle 验证工具三份服务器源各多一个末尾 CRLF，其余代码字节一致，Python AST 也相同；不是算法变化。对应原始源、实际 archive SHA 和差异报告保存于 [精确来源档案](validation/ts20_prediction_chain_20261022/server_executed_software_v2/source_diff_report.json)，Git 对该目录关闭文本换行转换，保留证明所引用的原始 SHA，不以本地文件 SHA 代替服务器执行身份。

本轮[实际阶段记录](validation/ts20_prediction_chain_20261022/causal_chain_actual_20261023.json)中，fresh folded hover 已完成独立验收：完整 20 s、5001 个原生点，机体三维 RMS 1.80599 mm／峰值 2.83115 mm；完整 ULog、参数恢复和实际平台／控制／软件源码归档核验通过。新的关节探针也已通过完整 30 s 验收：两关节 q 跟踪 RMS 3.20663／3.52423 mrad，摆臂机体峰值 10.80337 mm、倾角峰值 0.129915°，整段首态自由预测 q 峰差 5.72131e-7 rad。快速轨迹尚未运行，旧工况的 full-window 证书没有升级为新版证书。观察文件失败或观察计数不完整仍拒绝新观测阶段完整性，并继续封存执行器日志，不遮蔽原控制错误。

因果输入修正后的首次保持仍失败，详见[本轮精确失败与时序记录](validation/ts20_prediction_chain_20261022/causal_quiet_actual_20261023.json)。183 次 measure 发布后，下一轮在 `dt=64 ms > 50 ms` 的原保护处停止，未调用 allocator、未发布被拒指令。独立物理记录仅覆盖 6.412 s、1604 点，末端 RMS 3.66757 mm／峰值 5.22943 mm，不能当作完整 30 s 保持通过。部分窗口的因果反馈、参考状态、MIT 包、ULog、11 项 typed 恢复、同步保存、42 项 PID 与 49／38／86 项实际控制／平台／软件源档案均核过，已着陆解除锁定并关闭 runtime。

本轮有精确临界段：最后成功 PVA 返回后，`control_write_exit → diagnostic_write_begin` 耗时 35.102690 ms，其中完整自然 gen2 callback 区间为 31.200033 ms，位于诊断转换／编码的写入前间隙；实际诊断 write 为 0.070911 ms。随后 pump／callback drain 把处理时钟从 47.908 推到 47.972 s，下一轮拒绝 64 ms 间隔。没有采集 GC 触发栈，不能细分为 `json_value` 或 `json.dumps` 的哪条分配触发，也不把这次结论外推到历史 53／56 ms。随后实现诊断序列化避免整树复制、记录在线统计两项修正，GC 策略、模型、控制参数与门槛保持不变；其软件验收与新的实际阶段分开记录如下。

本轮记录路径修正中，TS20 显式 serializer 仅对字符串键、限定数值 dtype 和最多 32 层内建容器使用直接编码，避免先复制整棵 dict/list 树。其余输入完整回到原 `json_value + dumps`，保留键转换、object／longdouble、自定义类型、循环和非有限数的既有语义；没有启用 hook 的共享场景仍用原表达式。识别函数为模块级递归，不逐轮产生闭包自环。FlightRecorder 只保留采样数、phase 计数和间隔统计，完整 CSV 保持原字节；14 项 recorder 测试含冻结旧类的逐字段及 CSV／manifest 原字节对照，长期对象数量不随采样行数增加。

服务器真实末尾 20 条 decoded 诊断记录的 100 对 AB／BA 样本逐字节相同，旧／新编码中位数为 5.169196／3.617654 ms；另一个新生成 NumPy inner-allocation 样本的 5 对中位数为 2.292931／1.725961 ms。独立 tracemalloc 该 NumPy 样本峰值为 356115／296315 B，tracemalloc 不混作普通计时。全部 raw 计时、自然 GC、类型与分配结果见[完整离线观测](validation/ts20_prediction_chain_20261022/recording_serialization_observation_20261023.json)，由原报告 SHA `f41579662b059b549c8e9ad03182af51522122ad5780391055da1f09753efc01` 绑定。decoded JSON 不恢复历史 NumPy 类型／alias／heap；新 NumPy 样本不代表完整 adapter heap，离线节省不保证消除真实 gen2 停顿。

新[记录路径软件证明](validation/ts20_prediction_chain_20261022/recording_software_acceptance_20261023.json)原字节 SHA 为 `c71da7352984187a3c54be518787dee6669fbfe87fa055201d97f003056525b3`，绑定 89 项实际源；317 项测试（124 package、149 interface／audit、44 shared-loop）全部通过、0 skip，原 30 工况重新全部通过，最大 26.608396 ms，full/view 比较差值均为零。新的 folded hover 已通过完整 20 s 验收，三维 RMS 1.76454 mm／峰值 2.71331 mm；新的关节探针正在运行，完整末端保持仍须取得当前代码的实际证据，快速轨迹保持关闭。此前失败及证书保留各自原始身份。

## 12. 真机条件仍需补齐

仍需完成两类物理证据：

- 24 V 和最终连杆装配下的 MIT 命令→实际关节运动，覆盖正负、换向、工作角度、停止和温度；建立独立验证记录。
- 若要用真实力矩驱动 Gazebo 刚体，标定可信的力矩映射和装配参数；或者明确采用经加载辨识的闭环关节响应插件，并验证它对机体耦合的适用性。额定峰值和归一化惯量都不能代替该标定。

完成后，在新的工况身份下重新验证悬停摆臂与末端保持，再推进轨迹。真机前总体路线及同接口 NMPC 的比较设计见 [验证框架](X650_REAL_FLIGHT_READINESS_AND_NMPC.md)。
