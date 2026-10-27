# 原上层 MPC 基线审计与后续统一比较要求

日期：2026-10-27。范围：只读检查已保留的 C++ MPC、可调用封装与现有验收逻辑；未修改旧算法、旧参数、ROS 执行端或服务器，未进行新的物理实验。本审计不把源码层面的潜在遗漏写成已经发生的飞行故障。

## 结论

原控制器可以准确称为**采用在线局部线性预测与凸 QP 的上层协同 MPC**。它同时优化飞机三轴速度指令和两个关节速度指令，具有滚动时域、状态反馈、实际发送历史、执行约束和非线性候选复核。它不是仅给轨迹套了一个 MPC 名称，也不因采用线性化而“不规范”。其控制层级仍是 PX4 PID 之上的速度控制层，不是整机刚体与机械臂力矩动力学的全身 NMPC。

需要修正的具体问题是：**规划中的关节制动余量硬约束，没有在所有普通成功分支中用非线性候选重验；当前试验性 NMPC 则每次重验。**因此此前“同参数”成立，不代表两个后端的所有候选验收规则完全相同。还应明确区分局部 QP 的驻点检查与完整 NLP 的最优性证明。

没有在本轮只读检查中发现 QP 的 Hessian/线性项公式或末端几何链式求导的确定代数错误。没有据此否定既有完整末端保持、八字、下发消费和正常退出的原始记录。

## 原控制问题及代码证据

下表行号对应本文末尾绑定的本地源码版本。

| 内容 | 实现与判读 | 代码位置 |
|---|---|---|
| 决策变量 | 每个节点 `u=[vx,vy,vz,dq1,dq2]`；15 个节点，共 75 个归一化决策量 `z=u/velocity_limit` | `native/mpc_core.hpp:22–36`；`native/optimization.cpp:126–137` |
| 机体预测 | 水平速度指令到实际速度/加速度的二阶闭环响应，垂直一阶响应，ZOH 矩阵指数离散化；不是螺旋桨推力/机体力矩动力学 | `native/optimization.cpp:79–86` |
| 关节预测 | 实际报文历史、固定总线时刻、编码量化、辨识运动模型与事件传播；局部灵敏度用于构造仿射状态 | `native/mpc_core.cpp:76–153` |
| 末端输出 | 连杆三角几何、推力方向姿态与当前姿态残差衰减模型；在线计算位置/速度 Jacobian | `native/optimization.cpp:40–77,139–160` |
| 目标函数 | 末端位置、末端速度、机体名义位置、关节名义位置、归一化控制偏差、控制增量，共六项；按各预测步长加权 | `native/optimization.cpp:145,178–180` |
| QP | `min ||Az-b||²`，等价于 `0.5 zᵀHz+gᵀz+const`，`H=2AᵀA,g=-2Aᵀb`；约束 `lo≤Cz≤hi, |z|≤1` | `native/optimization.cpp:195–197,218–240` |
| QP 求解 | 检查正定性，Cholesky 白化，再用有界迭代的主动集求解器；保留原约束行复核 | `native/optimization.cpp:218–240` |
| 滚动更新 | 使用上一条已接受计划按当前时间移位；使用真实当前反馈和预览参考重新构造 QP；最多三个控制子问题 | `native/mpc_core.cpp:267–284` |
| 事务与发送 | `prepare` 在私有 Core 副本上求解；真实发送后才提交对应 Memory；未发出的候选可丢弃 | `control_packages/x650_mpc_controller_v1/src/controller.cpp:33–66` |

其中 `native/…` 均位于 `control_packages/x650_wholebody_v1/`。

当前配置的名义节点间隔为 0.04 s，第一段使用实际 `dt`；预测长度约 0.6 s。求解名义频率 25 Hz，下发 50 Hz。`arm_share=0.2` 用于构造名义机体/关节参考及正则项；末端跟踪的最终五通道指令由 QP 联合决定，不能把它解释为优化结果始终固定分担 20% 的运动。

## 允许的局部近似，不能混称为精确非线性导数

`mpc_core.cpp:91–97` 对未来报文做位置、速度量化，同时为其构造连续未量化指令的灵敏度。`mpc_core.cpp:128–149` 的常规传播使用冻结滑动分支的线性灵敏度；重试路径用小幅强迫项扰动重新传播。这些量不是“包含量化与静摩擦切换的完整混合模型的处处精确导数”。量化本身在台阶间的导数为零，跳变处也不可按普通光滑 NLP 处理。

对于局部线性 MPC，这是一种可以明确声明的预测近似。后端限制计划修改，并重新传播编码后的候选、检验连续关节包络及预测误差。该机制支持工程上使用局部 QP，不等于严格 SQP 已在同一个光滑非线性问题上满足一致梯度、Hessian 与全局化条件。

实际起始信赖域是归一化关节指令 `|z-z_center|≤0.02`，而不是 0.02 rad/s：两个关节对应约 0.009 / 0.007 rad/s 的单次局部修改。`selected_profile.hpp` 中配置半径 0.15 在 Core 组装时由 `min(trust_radius,0.02)` 限到 0.02。机体三个通道没有相同的 0.02 信赖域，只受自身速度/增量与目标模型约束。这是现有算法的设计范围，不是所有线性 MPC 必须如此。

候选最终检查的阈值为关节位置误差 0.006 rad、关节速度误差 0.10 rad/s，不能把“通过此检查”解读为局部预测等于真实预测。硬关节包络另行检查，状态位置裕量 0.0003 rad、速度裕量 0.03 rad/s，对应速度内界 0.42 / 0.32 rad/s。证据：`native/joint_safety.hpp:8–12`，`native/optimization.cpp:247–254`。

## 发现一：普通成功分支缺少实际候选制动余量复核

QP 在 `native/optimization.cpp:183–190` 对每个节点施加：

\[
q_{\min,j}\le \hat q_{i,j}+r_j\hat{\dot q}_{i,j}\le q_{\max,j},\quad
r_j=\frac{v_{\max,j}}{2a_{\max,j}}+dt_{\max}+T_{hold}+age.
\]

这里带帽状态来自 QP 的仿射预测。Core 求解后确实重新取得非线性候选的 `q,dq`，但 `native/mpc_core.cpp:402` 只有在 `recovery_triggered` 为真时才调用 `stopping_violation`。普通成功分支的 `x650_candidate` 只检查连续 q/dq 包络、积分参考边界及局部预测误差，不包含 `q+r*dq`。因此“仿射制动行合格”并不逻辑蕴含“非线性候选制动行合格”。

一个仅说明逻辑差异的节点数值例子，不是已观测的失败记录：关节 1、`age=0` 时 `r=0.635 s`。若仿射预测 `q=0.180 rad,dq=0.420 rad/s`，其制动量为 0.4467 rad，小于上界 0.45 rad；实际候选 `q=0.185,dq=0.420` 则为 0.4517 rad。位置预测差 0.005 rad 仍小于 0.006 rad 误差阈值，且 q/dq 各自没有超出状态内界，但制动量已不满足原要求。此例只说明三个检查不等价；下面的补充审计另外给出可绑定完整真实输入的数值证据。

当前试验性 NMPC 在每个候选中调用 `actual_stopping_violation`，在线发布前再次检查：`control_packages/x650_nmpc_controller_v1/src/offline_solver.cpp:18–31,65–73`，`src/online_solver.cpp:93–94`。现有独立审计同样只为 NMPC 检查该实际制动量（`scripts/audit_x650_upper_controller.py:150–160,204–205`）。这是待统一的验收口径差异。

已下载的原 MPC 保持、普通八字、快速八字报告均记录 `recovery_calls=0`，说明它们使用的正是普通分支。原报告没有给出所有普通分支的实际制动量最大值。本轮随后完成了原 `candidate_states/predicted_states` 与原始 CDR 状态年龄的逐节点补算：**今天这三组的全部 2060 个已接受候选都满足实际节点制动保护；另一个历史受扰工况的 569 个可恢复候选中，有 31 个不满足。**两套数据来源与范围不同，不能把历史受扰结果套用到今天的普通/快速八字。详细证据见下一节，原验收文件保持不变。

这一制动表达式本身是控制器使用的工程保护量。`a_max` 在现有 QP 中限制的是指令变化率；没有附带证明它等于所有负载、摩擦、延迟条件下的真实关节可保证制动力。因此统一检查能修复口径缺口，但不会自动给出全局递归可行性或真机安全证明。

## 2026-10-27 补充重算结果

两次补充均未运行优化器或新的 PX4/Gazebo 实验，也未改旧验收结论。保护式沿用旧代码原值与原 `feasibility_tolerance=2e-7`，没有调整阈值。

第一份报告 [legacy_candidate_audit_v1.json](validation/standard_nmpc_20261027/legacy_candidate_audit_v1.json) 使用 `artifacts/nmpc_interface_20261027/full_inputs` 的四份原始记录。原记录共 1281 次调用，按下一帧 Memory 中的计划恢复本次已提交候选；要求序号、`plan_t/last_t`、步计数、首指令、整个提交后 Memory 及历史修剪结果全部匹配。每组末帧没有下一帧，明确排除 2 条，未推测尾计划。新公共事件验证器检查其余 1279 条，包含原始已发送报文前缀、指令界、参考积分界、连续位置/速度包络、实际节点制动量、首指令 pending guard 与 160 ms 保持。

| 历史工况 | 可恢复/原调用数 | 全部公共检查通过 | 只因实际节点制动量失败 | 被检查任务时间 |
|---|---:|---:|---:|---|
| 保持 | 710 / 711 | 710 | 0 | 0.052–29.952 s |
| 原三段受扰 | 569 / 570 | 538 | 31 | 0.052–23.956 s |

历史受扰最不利候选为 `wrench_000363`，任务时间 15.320 s，第 1 个预测节点、关节 1：实际事件模型 `q=0.3854652595 rad`、`dq=0.1043932777 rad/s`，`r=0.639 s`，得到 `q+r·dq=0.4521725639 rad`，比 0.45 rad 保护上界多 **0.0021725639 rad**。这 31 条的连续 q/dq 包络、首指令保护与保持检查均通过，只有节点制动量不合格。它们是候选预测中的保护缺口，不能解释为观测到真实关节位置/速度越界，也没有证明这就是原受扰整段验收失败的全部原因。原受扰报告原本 `passed=false`，未被覆盖。

第二份报告 [todays_mpc_stopping_audit_v1.json](validation/standard_nmpc_20261027/todays_mpc_stopping_audit_v1.json) 专门检查今天配对实验中的 **MPC** 三组。直接读取原 Full 诊断及原 snapshot/proposal CDR，按原验收报告逐文件验证 SHA256，再交叉核对每个原预测时刻的 `candidate_states` 与 `predicted_states[:,6:10]`，用当帧原始 `control_time_s-joint_time_s` 计算保护量。此次仅重算制动量，不冒充重新执行了一遍全部公共事件验证。

| 今天的 MPC 工况 | 全部已接受候选 | 首指令实际发送候选 | 制动保护失败 | 最小保护裕量 | 原候选任务时间 |
|---|---:|---:|---:|---:|---|
| 完整末端保持 | 717 | 716 | 0 | 0.336789532 rad | 0.048–29.992 s |
| 普通八字 | 770 | 770 | 0 | 0.170232515 rad | 0.048–31.980 s |
| 快速八字 | 573 | 573 | 0 | 0.070501461 rad | 0.048–23.984 s |

保持末尾第 717 条是退出时明确丢弃的未发候选，实际发送前缀到 29.948 s；普通/快速均无未发尾候选。每条记录全部 15 个预测节点、两个关节都检查了，因此覆盖 61,800 个节点关节组合。最小裕量定义为 `min(q+r·dq−q_lower, q_upper−q−r·dq)`，是保护表达式剩余量，不能当成真实制动距离。候选的首指令发送不代表整条未来预测都已执行；下发、实际消费和整段物理误差仍以原审计为准。

三组原始报告分别绑定：

| 文件 | SHA256 |
|---|---|
| `artifacts/nmpc_live_20261027/mpc_hold_audit_v1/report.json` | `d206ad12dacb18525314c65295d3d0e1a757359a27b2dfc7b2e824dce0906e26` |
| `artifacts/nmpc_live_20261027/mpc_normal_audit_v1/report.json` | `f00929e4ac2dbc0b352f2d9a63ab7a74ca1a460d7cec9bbf0a9be250d504bef3` |
| `artifacts/nmpc_live_20261027/mpc_fast_audit_v1/report.json` | `265dbb247e3c8b65890b1516fdda5c5f228c5b92d8d874e6ee1c30e0bde3f353` |

新增补充证据的 SHA256：

| 文件 | SHA256 |
|---|---|
| `docs/validation/standard_nmpc_20261027/legacy_candidate_audit_v1.json` | `ca3e5ac14e5143eccb96441689ef86082c68f8a4ea0b2c39e1d0b62aa850c752` |
| `docs/validation/standard_nmpc_20261027/todays_mpc_stopping_audit_v1.json` | `5d0bdb3f5d7ba1c47928c2707e42a239e3fb77579669c9517ac9c11e872d622c` |
| `scripts/audit_x650_legacy_common_gate.py` | `b72667c1aba62ce896dcc3b92796d0a0091ad007db4dac59298a936301d33038` |
| `scripts/audit_x650_recorded_stopping.py` | `2d570accf088f4f21ac9b44a26e2b44c30c3bd562ae0c441f586370acb3bd1da` |

历史候选补充的完整输入及运行库身份另存于第一份报告：源 manifest SHA256 `028dc7cc49492f8c630394a65213c85ca6402ef886754cc9e15ee857db5325e3`；隔离 `x650_standard_event_replay` SHA256 `cdc6808f3f1bfe56116d2d98068c017edba56ead18a755d8990899edd5c1463b`。报告列出每个运行库、全部 4 份源记录和每批重放输入/输出的原路径、字节数及哈希。服务器的新补充目录为 `artifacts/standard_nmpc_20261027/legacy_candidate_audit_v1` 与 `todays_mpc_stopping_audit_v1`。

## 发现二：现有 KKT 门控的声明需要精确

`native/optimization.cpp:209–216` 从近活动约束重构非负乘子，计算原始可行性、驻点残差和互补残差。`native/mpc_core.cpp:389–390` 实际门控的是前两项；互补残差被保存，但没有门控。现有独立审计 `scripts/audit_x650_upper_controller.py:81–92` 也检查原始可行性与非负乘子驻点残差，没有独立互补残差门槛。

这不是证据表明主动集求解器求错了，也不否定已有数值一致性记录。更精确的描述是“凸 QP 解经过原约束可行性和驻点残差复核”，而不是“完整 NLP KKT 条件已经证明”。新的统一数值审计应报告 primal、dual、stationarity、complementarity 各项及其定义/量纲；最终比较不能把旧 QP 的 KKT 残差直接当成新 NLP 的 KKT 残差。

## 建议的最小公共候选验证器

保持冻结旧库与历史证据不变，新增只读、无 ROS I/O 的公共验证库。输入是固定配置、原始测量及时间戳、原始已发送历史、候选物理计划与待发送编码。输出结构化报告，不修改 Memory，不选择轨迹，不发指令。

公共检查至少包括：

1. 输入/历史的有限性、单调时间、来源年龄、原始 issue 截止时间、不可改写的已发送报文前缀。
2. 同一速度与指令增量界、同一积分参考位置界、同一编码与总线时刻；所有量纲公开。
3. 同一个**实际编码候选验证模型**的重新传播、节点之间 q/dq 硬包络及 0.16 s 首指令保持验证。
4. 每个候选、每个节点的实际 `q+r*dq` 保护量，双方相同容差；明确这是节点保护，不附会为尚未证明的连续时间制动不变量。
5. 首指令、候选计划与 proposed Memory 的一致性；只有真实 writer 通知对应命令已发出才提交，消费确认维持异步。

可从旧 Core 的 `packets/reference/rollout/first_command_hold` 与新 NMPC 的 `actual_stopping_violation` 提取语义；因它们当前大多为私有函数，第一步可在新验证库中实现独立只读版本，并用原记录逐字段对照原报文、节点、包络和保持结果，避免为共享代码而直接改变旧二进制。`evaluate_first_command_hold` 已是公开纯查询入口（`native/mpc_core.cpp:234–240`）。

**求解器专属检查与公共物理检查应分开。**旧 MPC 的“局部线性化误差小于阈值”、局部 QP KKT，与新 NMPC 的光滑模型导数验证、NLP/SQP 残差属于算法内部资格；共同安全检查不要求 NMPC 伪造一个旧 QP 证书。新旧算法都必须通过同一真实编码候选保护。

加上统一保护后若旧 MPC 的某些候选被新规则拒绝，应给新运行明确命名，例如 `legacy_mpc_common_gate_v1`；旧数值核仍冻结，但准入规则已经扩展，不能把它与历史运行称为完全相同的软件验收政策。

推荐的最小上线修复位置是新建 `legacy_mpc_common_gate_v1` 适配层：先由保留的 MPC 可调用接口 `prepare` 得到候选，再将原始 Memory、输入和候选物理计划交给公共验证器；全部通过且原 issue 时间预算仍满足才发布，实际 writer 通知对应指令发送后才提交 Memory。验证失败时丢弃当前 proposal，不因内部求解成功就提交历史。这样可与新 NMPC 共享相同候选保护，同时保留旧库和原验收证据。若只修原 Core 普通分支，技术上可把已有 `stopping_violation` 检查改为无条件执行，但必须另命名、另编译并重新资格验证；本轮**没有改动旧 `Core::step` 的接受策略**。两种方案都不能只把拒绝删掉，或把“不满足保护”当作优化器数值错误；是否恢复重规划应由后续明确的算法分支处理。

## 与规范光滑 NMPC/SQP 的公平比较框架

需将两种问题分开报告：

- **工程系统比较**：冻结旧 MPC + 公共保护，与新光滑 NMPC + 相同公共保护，使用相同真实 Gazebo/TS20/PX4 被控对象。控制器内部预测模型如果改变，明确这是“预测模型与求解方法一起变化”，不能把全部差异归因于非线性求解器。
- **求解方法比较**：对同一个新光滑预测模型、同一代价和物理约束，增加一个明确的单次局部线性/QP基线，与真正带一致导数及全局化的 SQP/NLP 求解比较；该基线不是旧 MPC 的改名。

两种比较都需预先固定预测时域/离散步长、六项代价及单位、终端项（如新增需双方一致或单独消融）、约束与裕量、参考轨迹、实际初始条件处理、CPU/线程亲和、Full 记录开销、25/50 Hz 调度及原 issue 到发布的 50 ms 总预算。只改变迭代次数时必须明确计算预算是否相同。

资格顺序：导数与离散模型单测 → 原始固定记录的完整实际候选复核 → 私有 DDS 软件拒绝/异步提交测试 → 新完整末端保持 → 普通八字 → 快速八字。记录各次失败与未发尾候选，使用原时间戳及原 LiveStart 中心计算误差；不做后验平移、相位对齐或只选成功片段。若只跑每格一次，仍只能报告单次观测；后续用预声明重复次数/扰动种子比较分布。

## 本轮证据绑定与未完成事项

本轮先完成只读源码审计，随后按新增任务执行了以上两套原记录补充重算，并增量实现公共验证 API。没有执行新的物理实验，没有覆盖原报告。下列 SHA256 为**最初只读审计时**的文件字节；源码后续变化应单独绑定验证版本，而非悄改旧结论。

| 文件 | SHA256 |
|---|---|
| `control_packages/x650_wholebody_v1/native/mpc_core.cpp` | `7826911a28df82152f5a0b6dab078b2415747f45bdbfc23170edda6a0a7799d0` |
| `control_packages/x650_wholebody_v1/native/optimization.cpp` | `b78bdb1612ebe211eb6a063aacf3db56cd4ca7a1b07115efc7f7f86575e41fc6` |
| `control_packages/x650_wholebody_v1/native/joint_safety.hpp` | `248f4bfd2d217858acf62ea0ddfac30ca187db40867c80f928acccf9192d6dde` |
| `control_packages/x650_mpc_controller_v1/include/x650_mpc/selected_profile.hpp` | `e5f13843861576bddda14afa807182b4e43b28db95e06a791b14b66b84580547` |
| `scripts/audit_x650_upper_controller.py` | `ca73bc755f7213f67bbacc6b2bd93c50369f17fa6e61464f8256db942af91574` |

审计后的实现跟进：按新的开发任务，在 `native/mpc_core.hpp/.cpp` 增量加入了公共 `PlanValidation/evaluate_physical_plan` API，并在 `control_packages/x650_nmpc_standard_v1/tests/test_event_validator.cpp` 加入回归用例。它直接复用原报文编码、事件传播和首指令保持逻辑，无 QP 调用，不改变 `Core::step` 的原接受策略。旧 ABI-4 数据布局保持不变；增加新符号需要另行隔离构建，不替换已归档的旧库。上表与前文行号仍绑定本次**审计开始时的旧源码**，不是新增 API 后的文件哈希。构建和回归结果由新标准后端的验证记录单独给出。
