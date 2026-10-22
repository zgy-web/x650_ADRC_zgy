# 2026-10-22：X650 八字实验阶段结果

**小幅和普通八字的 PID／MPC 共四组完整实验均已通过。普通八字 MPC 的末端三维 RMS 为 4.087 mm，相对同阶段 PID 的 15.965 mm 降低 74.402%；峰值由 51.437 mm 降至 8.595 mm。** 普通 MPC 首次 52 ms 时序中断记录完整保留；本次成绩来自等价 CPU 优化后的全新完整运行，不拼接失败记录。四组视频已渲染并通过完整解码验证，快速八字尚未开展。

本轮承接[预测链补齐与完整末端保持](X650_PREDICTION_CHAIN_RESULTS_20261022.md)，控制定义见[用于本轮八字的 MPC 公式与架构](X650_MPC_FORMULAS_AND_ARCHITECTURE.md)。本文数值来自已落地的 `accepted_results_summary_v3.json` revision 3；正式依据仍是各运行的完整 native／ULog 审计。倾角直接使用原审计的度数，没有再次作弧度到度的换算。

## 实验条件和计分方法

平台为服务器 PX4 SITL＋Gazebo，物理对象为导入的 X650 URDF＋原二连杆，总质量 3.495 kg。精密 EV 由 Gazebo 生成位置／速度观测辅助 PX4 EKF；控制器直接使用估计状态及关节反馈，独立原生物理记录用于验收。

本轮冻结 `mixed_response_prediction_chain_v2`，采用相同的物理模型、42 项 PX4 MC／MPC 参数、测量条件、基体响应模型、共同速度／变化率／关节限制。**四组实际加速度前馈均为零**；`pid_ff` 是历史枚举，本文称“PID 协同分配基线”。PX4 位置／速度接口中的速度前馈桥接仍使用，不能把这轮描述为“全部前馈关闭”。

| 项目 | 小幅八字 small | 普通八字 normal |
|---|---:|---:|
| 世界 X 幅值 | ±0.075 m | ±0.25 m |
| 世界 Z 幅值 | ±0.03 m | ±0.10 m |
| 匀速相位段标称周期 | 16 s／圈 | 12 s／圈 |
| 圈数 | 2 | 2 |
| 首尾相位加减速 | 各 2 s | 各 2 s |
| 轨迹运动窗口，任务时间 | 2–36 s | 2–28 s |
| 声明完整测量时长 | 40 s | 32 s |

正式末端误差为同一原生时刻的物理末端位置减去预先声明的解析目标：

$$
e(t)=p_{tip}^{native}(t)-p_{tip}^{ref}(t),\qquad
\mathrm{RMS}_{3D}=\sqrt{\frac{\int\|e(t)\|^2dt}{t_{last}-t_{first}}}.
$$

积分使用原始时间轴的梯形积分，峰值为原生样本误差范数最大值。计分覆盖完整测量窗口，包括开始保持、平滑起停、两圈运动和末尾保持；没有去均值、拟合平移或相位。它不是仅运动段 RMS，也不是最近路径距离。

## 已通过的四组结果

| 项目 | 小幅 PID | 小幅 MPC | 普通 PID | 普通 MPC |
|---|---:|---:|---:|---:|
| 完整测量时长 | 40.008 s | 40.008 s | 32.012 s | 32.012 s |
| 原生样本数 | 10,003 | 10,003 | 8,004 | 8,004 |
| 末端三维位置误差 RMS | **1.920318 mm** | **1.570659 mm** | **15.964581 mm** | **4.086544 mm** |
| 末端三维位置误差峰值 | 4.983532 mm | 3.983726 mm | 51.437011 mm | 8.594742 mm |
| X／Y／Z 误差 RMS | 1.386467／1.160278／0.647370 mm | 1.172564／0.843948／0.616291 mm | 14.943591／3.236058／4.591825 mm | 2.928146／1.576740／2.374805 mm |
| 误差均值 X／Y／Z | −0.457634／+0.165960／−0.297470 mm | −0.461220／−0.061483／−0.304252 mm | −0.192814／+0.367709／+1.177794 mm | −0.477551／+0.198576／−0.398953 mm |
| 最大物理倾角 | 0.208755° | 0.172001° | 1.237929° | 0.636279° |
| 最大实际控制间隔 | 36 ms | 48 ms | 40 ms | 40 ms |
| 首步实际控制间隔 | 28 ms | 28 ms | 24 ms | 28 ms |
| 控制记录最大 `compute_ms` | 12.617 ms | 31.452 ms | 14.241 ms | 25.293 ms |
| 完整阶段审计 | 通过 | 通过 | 通过 | 通过 |
| 着陆、解除武装、参数恢复 | 通过 | 通过 | 通过 | 通过 |

小幅成对实验中，MPC 的三维 RMS 相对 PID 降低 **18.208%**，峰值降低 **20.062%**；普通成对实验分别降低 **74.402%／83.291%**。两种尺度均为每个控制器各一次完整通过运行的比较，不代表统计显著性或所有工况下的优势；普通 MPC 的首次失败未纳入跟踪性能排名。

| MPC 优化验收 | 小幅 | 普通 |
|---|---:|---:|
| 被接受的优化次数 | 1,448／1,448 | 1,277／1,277 |
| 旧计划回退次数 | 0 | 0 |
| 最大求解耗时 `solve_ms` | 0.564113 ms | 0.694867 ms |
| 超过 20 ms 求解预算次数 | 0 | 0 |
| 最大原始约束违反量 | 0 | 0 |
| 最大 KKT 平稳性残差 | `1.6653e-16` | `7.2164e-16` |

`solve_ms` 只覆盖优化求解区间，`compute_ms` 还包括其他控制工作，两者均不能替代实际相邻指令间隔。四组共同满足原有末端 RMS≤5 cm、峰值≤10 cm、倾角≤8°以及原生关节软限位、角速度和发送指令变化率标准。此处“通过”表示达到既定实验门槛，不表示参数已经最优。

## 独立绑定的参考中心与航向

每次运行在任务前绑定参考姿态，末端中心由固定世界基体锚点和该姿态下的零关节构型计算。对照校验要求协议、PID 和模型一致，但不强制不同运行的绑定姿态相同。以下原始值全部保留：

| 运行 | 末端参考中心 ENU，m | 绑定估计航向 |
|---|---|---:|
| 小幅 PID | [−0.005594646, 0.040793652, 2.039704743] | −5.964664° |
| 小幅 MPC | [−0.005648411, 0.040732858, 2.039698150] | −5.879465° |
| 普通 PID | [−0.005612490, 0.040812029, 2.039707318] | −5.938403° |
| 普通 MPC | [−0.005768983, 0.040714279, 2.039697943] | −5.776180° |

小幅 MPC 减 PID 的中心差为 **[−0.053766, −0.060794, −0.006594] mm**，三维距离 **0.081426 mm**；绑定航向差 **+0.085198°**。普通对应中心差为 **[−0.156493, −0.097750, −0.009375] mm**，三维距离 **0.184751 mm**；绑定航向差 **+0.162223°**。这些是预先绑定的估计姿态差异，不是事后估算的轨迹配准量，也不是原生物理航向误差。指标始终对各自声明目标计算。

## 普通 PID 的限速线索

普通八字的幅值是小幅的 3.333 倍，标称周期更短，匀速相位段参考速度幅值提高 4.444 倍、加速度幅值提高 5.926 倍。普通 PID 的末端误差随运动需求增大；现有日志还显示 J1 在局部区间触及冻结速度上限。

以下只读辅助统计将“近 cap”定义为绝对速度≥冻结上限的 99%，仅用于描述占用，不改变验收规则：

| 统计 | 小幅 PID J1 | 普通 PID J1 |
|---|---:|---:|
| 发送速度峰值 | 0.0920122 rad/s | 0.4500000 rad/s |
| 实际原生速度峰值 | 0.0920122 rad/s | 0.450000000169 rad/s |
| 发送命令近 cap 样本 | 0／1,636 | 28／1,306，2.144% |
| 发送命令近 cap 时长 | 0 | 0.664 s，完整窗 2.074% |
| 原生实际近 cap 时长 | 0 | 0.668 s，完整窗 2.087% |
| 发送／实际最长连续段 | 0 | 0.288／0.284 s |

普通 PID 的 J1 命令与实际均有 6 段近 cap；J2 在两次运行中均无近 cap 段。普通 J2 速度峰值约 0.194250 rad/s，低于 0.35 rad/s cap。J1 实际峰值超过 0.45 的量仅 `1.69e-10 rad/s`，属于数值容差内触限。

占用时长按原始样本保持至下一样本计算，分母为完整声明窗口；命令测量段开头的 small 28 ms／normal 24 ms 未外推补齐。实际与命令峰值、占用时长接近，支持“局部限速”线索，但不足以把全部跟踪误差归因于关节速度限制，也没有证明执行器幅度跟不上。后续可结合指令变化率边界、停止包络和机体速度响应分析；不据此直接放宽 cap。

## 普通 MPC 首次中断记录

失败运行 **`x650_wholebody_mpc_v1_20261022_152539_18499`** 在任务时间约 **21.472 s 后**未能继续完成窗口，下一次实际控制间隔达到 **52 ms**，超过冻结的 50 ms 上限，触发恢复。该次已着陆、解除武装并恢复参数，失败记录保留于 `mpc_normal_job`。

中断前最后一条控制记录 `compute_ms` 约 **38.275 ms**，其中优化 `solve_ms` 约 **0.272 ms**。这说明要检查求解之外的控制计算、消息处理和调度跨度，不能把 52 ms 直接解释成 QP 求解耗时。离线 profiling 已定位到预测输出构建中的计算开销，但不能单凭离线结果把现场超时归因于某一个函数、消息处理或调度步骤。

该次没有完整普通八字证书，不报告其局部窗口跟踪性能排名，也不拼接补齐。原失败记录和 50 ms 门槛保留。后续全新运行 **`x650_wholebody_mpc_v1_20261022_154759_20284`** 使用下述等价实现，完成 32.012 s 整窗并通过审计，最大实际控制间隔为 **40 ms**，已经着陆、解除武装并恢复参数。快速八字尚未开展。

## 计算性能优化与离线等价复核

离线复核使用失败运行全部 **789 步**原始输入，包含激活时的实际指令历史、状态、参考轨迹和后续完整 warm-start 链。原版和优化版各执行两轮完整重放；输入 SHA 一致，唯一源码变化为 `prediction_chain.py` 中以专用 **N×3 叉乘**替换通用 `np.cross`。公式、有限差分 epsilon、响应模型、参数与控制门槛均未改变。

两版每一步的 **5 维指令、完整 15 节点预测计划和解析参考轨迹**与原飞行记录逐值比较，最大绝对误差均为 **0.0**。这验证了此批已记录输入上的数值等价性。去掉 profiler 干扰后的 adapter 耗时如下：

| 离线 adapter 耗时 | 原版 | 专用叉乘优化版 |
|---|---:|---:|
| 中位数 | 13.464 ms | 11.888 ms |
| P95 | 16.650 ms | 14.461 ms |
| 本轮最大值 | 21.017 ms | 18.348 ms |

中位数和 P95 分别下降约 **11.7%／13.1%**。这些是同一记录的离线 CPU 数据，adapter 计时范围也不等于现场含消息发布的 `compute_ms` 或实际控制间隔；本轮最大值不是硬实时上界。优化通过 **15 项测试**，源码提交 **`92ef899`** 已推送 Git。随后的全新普通 MPC 实验通过原有完整审计，构成本工况的实际验收；**不能由此断言所有工况下的时序超限已根治**。

可复核的本地证据位于 `E:/research_my_studio/flyarm/safe-aerial-manipulator-testbed/artifacts/x650_figure8_prediction_chain_20261022/offline_profile/`：原版 `result/profile_result.json`、优化版 `result_optimized/profile_result.json` 以及 `optimization_comparison.json`。

## 证据路径、图与视频状态

服务器项目根目录为 `/root/autodl-tmp/safe-aerial-manipulator-testbed`。以下路径均相对此根目录；原始数据、私有模型和媒体不随源码仓库发布：

| 状态 | 原始 run | 任务与审计目录 |
|---|---|---|
| 小幅 PID 通过 | `runs/x650_wholebody_mpc_v1_20261022_150814_15574` | `artifacts/x650_figure8_prediction_chain_20261022/pid_small_job` |
| 小幅 MPC 通过 | `runs/x650_wholebody_mpc_v1_20261022_151357_16552` | `artifacts/x650_figure8_prediction_chain_20261022/mpc_small_job` |
| 普通 PID 通过 | `runs/x650_wholebody_mpc_v1_20261022_151923_17527` | `artifacts/x650_figure8_prediction_chain_20261022/pid_normal_job` |
| 普通 MPC 通过 | `runs/x650_wholebody_mpc_v1_20261022_154759_20284` | `artifacts/x650_figure8_prediction_chain_20261022/mpc_normal_runtime1_job` |
| 普通 MPC 首次失败 | `runs/x650_wholebody_mpc_v1_20261022_152539_18499` | `artifacts/x650_figure8_prediction_chain_20261022/mpc_normal_job` |

每个成功任务目录中的 `audit/mixed_mpc_stage_audit.json` 是完整审计；`job_status.json` 指向实际 run、launch 和审计目录，launch 中的 `controller_sources.zip` 固化执行源码。失败任务保留状态、日志和恢复证据，不能当作成功审计使用。

| 身份 | SHA-256 |
|---|---|
| 物理模型 | `19e111daa9c915c3c402b2e7e936fcd83ca2b4821ec05308edde35cf2e14d784` |
| 源 URDF | `b33275b0ec9df6e2c495bac7956f702ad7d886c7d79230e711d0c5d70aa1e96d` |
| 42 项 PID 参数 | `f65f56cca1d5e50b6e2b09b57d9e9286298ad78d23cac701f98b0cae6fbd0b02` |
| 对照协议 | `943ee25af6ace870e8e47fe8eb561d953176833f8dda35e6ea95395d8625f6b6` |
| 运行条件 | `4a5aa0508e66d6ebf815ed2cbdc0a57956fa752d2bb6b440c88d0d36d5588f19` |

本地摘要为 `E:/research_my_studio/flyarm/artifacts/x650_figure8_prediction_chain_20261022/accepted_results_summary_v3.json`。它是展示摘要，不替代服务器原始证书。

**图已生成：** `artifacts/x650_figure8_prediction_chain_20261022/small_comparison/` 和 `normal_comparison/` 分别包含小幅与普通整窗误差、关节／计算时序、绝对世界 XZ 轨迹与 Y 偏离图及 manifest，两阶段 `verify_pair` 均通过。各运行保留自身参考中心，没有平移叠齐。

**四组视频已完成：** 小幅 PID、小幅 MPC、普通 PID、普通 MPC 均通过完整 FFmpeg 解码与源状态／FK 校验。回放保持既有 1440×900、24 fps、1 倍仿真时间、主视图＋机械臂近景＋误差曲线格式，并标明控制器／阶段；仅覆盖已审计测量窗口，不含起降，不虚构桨叶旋转。

四个 MP4 分别为成果根目录下 `pid_small_video/x650_arm.mp4`、`mpc_small_video/x650_arm.mp4`、`pid_normal_video/x650_arm.mp4`、`mpc_normal_video/x650_arm.mp4`。每个目录保留五个代表帧、FK 校验和带来源哈希的 `replay_manifest.json`。指标、图、视频与本报告打包到 `X650_figure8_PID_MPC_20261022.zip`，解压后可在浏览器打开 `OPEN_RESULTS.html` 离线对照播放；媒体文件及逐项 SHA-256 见 `delivery_manifest.json`。

当前 Remote Desktop Commander 文件工具支持图像展示，不支持 MP4 内嵌播放。完整结果包保存在服务器上述成果目录，可经现有 AutoDL 文件界面下载。

在服务器同一项目中，可用已有脚本重建小幅对照图；输出目录必须是新目录：

```bash
python3 scripts/plot_x650_mixed_stage_comparison.py \
  --pid-audit artifacts/x650_figure8_prediction_chain_20261022/pid_small_job/audit/mixed_mpc_stage_audit.json \
  --pid-run runs/x650_wholebody_mpc_v1_20261022_150814_15574 \
  --mpc-audit artifacts/x650_figure8_prediction_chain_20261022/mpc_small_job/audit/mixed_mpc_stage_audit.json \
  --mpc-run runs/x650_wholebody_mpc_v1_20261022_151357_16552 \
  --output artifacts/x650_figure8_prediction_chain_20261022/small_comparison_rebuilt
```

普通对照使用新的完整 MPC 运行：

```bash
python3 scripts/plot_x650_mixed_stage_comparison.py \
  --pid-audit artifacts/x650_figure8_prediction_chain_20261022/pid_normal_job/audit/mixed_mpc_stage_audit.json \
  --pid-run runs/x650_wholebody_mpc_v1_20261022_151923_17527 \
  --mpc-audit artifacts/x650_figure8_prediction_chain_20261022/mpc_normal_runtime1_job/audit/mixed_mpc_stage_audit.json \
  --mpc-run runs/x650_wholebody_mpc_v1_20261022_154759_20284 \
  --output artifacts/x650_figure8_prediction_chain_20261022/normal_comparison_rebuilt
```

视频重建使用 [native 派生回放说明](X650_MIXED_NATIVE_REPLAY.md)：先以通过的审计、原始 run 生成 `native_replay.csv`／manifest，再调用 `render_x650_arm_replay.py`。这是原始物理状态的离屏展示，不运行新动力学；正式指标继续沿用原生时间加权审计。
