# 新容器上的冻结 MPC 数值回归与地面初始化修复

更新日期：2026-10-24。本目录承接[历史实际接入记录](../isolated_live_20261023/README.md)，保存新 QP 库的完整软件资格证据、新容器资源与计时结果，以及后续实际仿真验收状态。

**当前执行版本为 revision16 / v9。历史时间比较修复通过5组C++与实际ROS拒绝输入测试，hold23不再触发历史时间误拒绝，但26次求解后因候选错过发送窗口、旧指令达到62.67毫秒而失败，已落地并恢复参数。尚未进入普通八字。后续先优化被实测确认的候选准备开销，并通过实际候选输入捕获做数值一致性验证；通过新的完整保持后再进入普通八字，快速八字仍后置。**

历史 hold_v15 的完整保持属于第 7 版和原数值库；它不作为本次新库、新协议的阶段准入。hold_v16 的 QP 失败、normal_v1 的 GC/时序失败、hold_v17 的交接初始化失败、hold_v18 的地面初始化失败以及 hold_v19/hold_v20 的活动指令过期失败均保留，不覆盖为通过结果。

## 1. 历史运行绑定与当前证据身份

| 项目 | 绑定 |
|---|---|
| 数值库 | `libx650_optimization.so`，ABI 1 |
| 数值库 SHA-256 | `b9cbdf78c76e92f973800e07c6219611759fcb4471a1a39192c5cbf30f5e2e48` |
| hold_v19 保持任务 | [`software/quiet_task_cpp_v6.json`](software/quiet_task_cpp_v6.json) |
| hold_v19 任务 SHA-256 | `3759184e2e9086e032cecc680dab7da7391d0b56309fb82fc88bf2644c1e36e1` |
| hold_v19 执行协议 | [`startup/execution_protocol_rev11.json`](startup/execution_protocol_rev11.json)，revision 11；后续准备版本另行绑定 |
| 已归档协议快照 | [`startup/execution_protocol_rev11.json`](startup/execution_protocol_rev11.json) 为本次 revision 11；[`software/execution_protocol.json`](software/execution_protocol.json) 保留历史 revision 9 原始字节 |
| 软件证据清单 | [`software/manifest.json`](software/manifest.json)，逐项保存服务器原路径和 SHA-256 |
| 初始化/传输证据清单 | [`startup/manifest.json`](startup/manifest.json)，7 份文件逐项保存服务器原路径、字节数与 SHA-256 |
| 后续追加失败证据 | [`startup/hold19_failure.json`](startup/hold19_failure.json)，独立追加，不在最初 7 文件清单内；身份见第 5 节 |
| 性能准备与复制证据 | [`preparation/manifest.json`](preparation/manifest.json)，4 份原样小报告，共 21921 字节；离线、合成和实际运行分析分别标记 |
| 最终版本证据清单 | [`final_preparation/manifest.json`](final_preparation/manifest.json)，13 份服务器原样报告，逐项核验字节数与 SHA-256 |
| hold_v20 保持任务 | [`final_preparation/hold_task.json`](final_preparation/hold_task.json)，v8；SHA-256 `9ef8c12c6a9591bdc80660710b0de25f8589defdf12b0362f9109444ff64cc4c` |
| hold_v20 执行协议 | [`final_preparation/protocol.json`](final_preparation/protocol.json)，revision 13 |
| 最终 native 数值证明 SHA-256 | `f07527f471e3a0c272ac5b916c73f52e9ac7be55732f11cb9731a61dbf3a26fb`，见 [`native_acceptance.json`](final_preparation/native_acceptance.json) |
| 独立参考预测器源码 SHA-256 | `ts20_prediction.py` 恢复为 `f25e45d755c236cd46c0e4fd2e11fb23fe926bc1b7e286dd87c9ac03414b7b66`，见 [`deployment_v25.json`](final_preparation/deployment_v25.json) |
| hold_v20 失败核验与证据 | [`hold20/failure_report.json`](hold20/failure_report.json)、[`hold20/manifest.json`](hold20/manifest.json)；完整原始计数、故障与恢复独立核对，部分原样文件压缩归档 |
| revision 14 执行准备证据 | [`execution_v14/manifest.json`](execution_v14/manifest.json)，6 份服务器原样小文件，共 26126 字节；无新物理保持结论 |

原样证据与派生分析在各清单中分别标记。报告中的绝对路径指向原服务器证据，不能把本地复制后的路径当作新的运行来源。第 10/11 版改变地面准备、就绪与活动阶段的交接；表中 v6/revision 11 身份保留给历史 hold_v19，不表示已经准入后续版本。v7/revision 12 因中间共享预测器源码不匹配在预检被拒绝；最终恢复独立参考源码，并将优化限制在 native 实现，重新取得完整后端资格后绑定 v8/revision 13。hold_v20 已明确失败；运行专属源码清单、执行协议和二进制哈希继续按该次身份保留。

## 2. 优化前绑定版本已完成的软件资格

| 检查 | 已核实结果 | 原样报告 |
|---|---|---|
| 完整测试组 | 339 项，0 失败、0 错误、0 跳过 | [`package_report.json`](software/package_report.json)、[`native_tests.json`](software/native_tests.json) |
| 原冻结矩阵工况 | 30/30，通过且来源未变 | [`matrix_original_host.json`](software/matrix_original_host.json) |
| 原始记录逐条重放 | 388/388，通过；历史失败记录保留 | [`native_replay.json`](software/native_replay.json) |
| 数值/软件准入 | `pass=true`、`no_flight=true`、`hardware_qualified=false` | [`native_acceptance.json`](software/native_acceptance.json)、[`software_acceptance.json`](software/software_acceptance.json) |
| 新容器同输入复测 | 30/30，来源与新库绑定不变 | [`matrix_new_host.json`](software/matrix_new_host.json) |

以上覆盖矩阵组装、QP 数值结果、候选分类、实际 MIT 包一致性及原始可行性/KKT 门限。这些报告属于优化前各自绑定的源码。最终仅优化 native 实现的版本已另外取得 344/30/388 资格，详见第 5 节；独立参考预测器保留原字节，未通过修改旧证书哈希绕过来源检查。数值证明不包含新的动力学飞行，不能据此公布新的末端误差或认为普通八字已经通过。

QP 修复源自 hold_v16 的第 1416 次求解：原始约束违反量 `3.270557920181938e-7` 被白化/归一化后，低于旧聚合停止阈值，导致内部提前返回；外层原 `2e-7` 门限正确拒绝了该候选。修复在内部增加每个原始不等式 `1e-10` 的要求，归一化后使用 `1e-10 / row_norm`，同时保留原聚合终止条件、迭代上限及外层所有检查。同一输入修复后违反量为 0，独立 KKT 残差约 `2.78e-17`。没有改动 MPC 模型、目标函数、权重、预测域、限制、TS20 参数或 PX4 PID。

## 3. 新容器资源与计时

主机记录见 [`software/host.json`](software/host.json)：

- 容器：`autodl-container-edeb42a3f7-c2d98427`。
- 可用逻辑 CPU：0–127，共 128 个；这不代表分配了 128 个核心的计算配额。
- cgroup `cpu.max`：`1200000 100000`，即 12 核时间配额。
- 角色 CPU：source 17；worker 87/89/91/97；dispatcher 93；ingress 94；consumption 95。
- 角色编号均有效且未互占同一物理核心；背景仿真/记录进程的集合按当前 SMT 拓扑重新生成，排除关键角色及其同胞，不照搬旧容器的数组。

新容器 30 个同输入工况中，`native_uninstrumented_ms` 最大 **23.3781635761261 ms**，`native_outer_ms` 最大 **23.224614560604095 ms**。逐字段矩阵检查带有额外观察开销，其 `native_instrumented_ms` 最大 **32.80509263277054 ms**，不与无插桩执行混用。上述都是有限工况的软件计时，不能证明并行 Gazebo/ROS 闭环中每周期必然达标，也不是硬实时最坏上界。

## 4. hold_v17 的失败与 revision 10

第 9 版 hold_v17 在进入实际接管 `activate` 的第一次 pump 触发约 56 ms 的控制间隔保护，随后安全恢复。它没有取得完整 30 秒保持资格。原始来源与失败边界见 [`hold17_failure.json`](startup/hold17_failure.json)。

本次定位到的停顿发生在上一 pump 返回与下一 pump 进入之间：墙钟约 **34.052 ms**，同线程 CPU 约 **30.685 ms**。其中共享生命周期在空中重新构造 allocator 时重复执行数值证明哈希核验；离线重现该核验约为 **24–26 ms**，随后还要同步启动 ingress、dispatcher、consumption 三个进程。问题属于交接边界上的同步初始化，没有证据把它归因于新 QP 求解变慢。

revision 10 将这些工作前移到原参数事务已建立、飞机仍 landed/disarmed、`loop.strict=false` 的地面阶段：

1. 构造并缓存生命周期 allocator，核对其求解步数为 0；不做求解或合成状态 reset。
2. 启动三个传输进程，确认实际 DDS/桥接端点匹配；保持 dispatcher 为 WAITING，正常输出计数为 0。
3. 空中只验证缓存的配置、模型、PID、循环对象和步数身份，附上实际任务轨迹；实际机体锚点仍由共享流程 reset。
4. 真实参考、命令历史与时间原点继续由起飞和预流后的同一次 `LiveStart` 注入。`LiveStart` 前 ingress 不准入状态，worker 不求解，执行器不输出。
5. 保存 `preflight_transport.json` 并与控制元数据互核；审计要求准备结束早于实际起飞和 `LiveStart`，且零步数、零正常输出等条件成立。

本地新增的 7 项地面准备测试和 4 项审计测试已通过。准备失败仍通过原参数恢复和自有进程清理路径退出。C++ 数值库及参数不变，20 ms 周期、50 ms 指令寿命/完整计算与模型 dt 门限、25 ms 来源年龄保持不变。第 10 版仍在 WAITING/READY 阶段启动活动时间保护，后续实际尝试暴露了以下问题。

### hold_v18：未起飞的初始化失败

本次运行未完成地面准备，尚未写出 `preflight_transport.json`；失败报告及原始证据哈希见 [`hold18_failure.json`](startup/hold18_failure.json)：

| 证据 | 实际记录 |
|---|---|
| 运行结果 | `controller_metadata.complete=false` |
| 主错误 | `Isolated consumption exited 1` |
| 消费进程日志 | `Consumption reader polling gap exceeds 25 ms` |
| 活动阶段 | `task_start` 与起飞状态转换均不存在，`lifecycle.started=false`，无 `LiveStart` |
| 执行观察 | 377 个 WAITING tick，0 次 actuator send；没有 MPC 活动控制 |
| 清理 | `diagnostic_parameters_restored=true`，`launch.runtime_closed=true` |
| 降落字段 | `landed_and_disarmed=false`；因为未起飞直接退出，不能写成完成了降落验收 |

消费读取器在 READY 后约 12 ms 报轮询间隔错误；原检查从前一次 poll 的入口计时，包含该轮的历史 CSV 解析、DDS 图查询和 ready 文件写入。没有分别剖析这些操作的耗时，因此不能认定某一项是主导原因。可以确定的是：此时没有活动控制指令，将构造/就绪阶段耗时计作活动消费监督超时，会在交接之前终止本次运行。

独立复查发现 dispatcher 的同类边界：WAITING 时没有已接受指令，但仍会因空闲 tick 迟到超过 20 ms 锁存漏期；首候选后的相位重锚不能撤销已经锁存的故障。该问题需要与消费读取器的生命周期同时修正，不能通过重复启动碰运气。

### revision 11：就绪与活动监督分开

当前修订保留第 10 版的地面 allocator 缓存与实际 `LiveStart`，增加：

- 消费读取器在 STARTING/READY 阶段读取历史数据，但不发布可作活动消费证据的状态。实际 `LiveStart` 校验 session、epoch、配置和原始发出时间后，才启动原 25 ms 监督。
- `LiveStart` 后先读至 EOF，丢弃此批所有积压记录；只允许其后新完整行进入活动消费状态，并要求其仿真时间晚于实际任务原点。观察时间使用本次读取开始时刻，不以解析结束时刻重新标新。
- dispatcher 无已接受指令时保持 WAITING、零输出，空闲心跳迟到不算漏执行周期；首条候选接受后原 20 ms 漏期检查立即生效，之后不可重置。来源年龄 25 ms、指令寿命 50 ms、显式故障锁存和 STOP 优先保持不变。

dispatcher 的新生命周期及活动门限边界用例已在本机实际编译运行通过；随后服务器完成对应构建与测试。消费状态机的边界、陈旧、重复、空闲与屏障测试以及真实 ROS 进程探针也已通过。下面的测试证明接口和生命周期行为，不构成新的物理保持通过证据。

### 已完成的服务器构建与合同测试

[`startup/manifest.json`](startup/manifest.json) 中的 7 份原样文件已逐项核对字节数与 SHA-256。部署清单见 [`deployment_v23.json`](startup/deployment_v23.json)，本次协议快照见 [`execution_protocol_rev11.json`](startup/execution_protocol_rev11.json)。

| 项目 | 已取得结果 | 证据 |
|---|---|---|
| C++ ROS 包构建 | `flyarm_control_gateway` 成功，包构建耗时 41.3 s，总汇总 41.7 s；有编译警告 | [`build_v23.log`](startup/build_v23.log) |
| C++ 合同测试 | CTest 5/5，包括 periodic gate、消费 CSV、消费生命周期及准入节拍 | 同一构建日志 |
| Python 准备与审计 | 地面准备 7/7 在服务器复测通过；此前审计负向测试 4/4 通过 | 构建日志记录本次 7 项地面准备测试；4 项审计负向结果属于此前本地检查 |
| 新执行二进制传输合同 | `transport_v10` 通过，7 次发送、7 次回执、停止零速确认；活动输出间隔 19.991179–20.078059 ms | [`transport_v10.json`](startup/transport_v10.json) |
| 消费读取器 ROS 生命周期 | 2/2 场景通过，原始二进制哈希未变；软件夹具，无 Gazebo/PX4 或执行器 I/O | [`consumption_startup_v1.json`](startup/consumption_startup_v1.json) |

必须区分构建成功和组合脚本全部成功：`build_v23` 包装脚本最终退出码为 1，其末尾运行服务器上遗留的旧 `transport_v9` 探针，因缺少要求的 DDS 端点而报 `dispatcher not ready`。此前的 C++ 包构建、CTest 5/5 和 Python 7/7 均已成功。同步 Git 中已有修正的正式探针后，新的 `transport_v10` 完整通过；没有为得到通过而修改控制规则或跳过就绪要求。这次旧探针失败不表述为物理控制失败，也不隐去组合脚本的非零退出。

消费探针的 `ready_live_active` 场景实际暂停 READY 进程 **81.054305 ms**，此阶段无输出且恢复后可正常接收真实 `LiveStart`；EOF 屏障之后的新完整记录才能发布。随后活动阶段暂停 **80.204781 ms**，进程按原 25 ms 间隔保护退出，退出码 1。另一场景发送错误配置哈希，进程同样退出 1 且零输出。错误 session/epoch、READY 不发布与历史积压不冒充新状态也包含在报告判定中。这里的“通过”指预期拒绝与生命周期合同符合，并非这些暂停工况仍能持续控制飞行。

## 5. 实际仿真失败结果与下一次验收

| 运行 | 版本 | 当前状态 | 是否准入下一阶段 |
|---|---|---|---|
| hold_v17 | revision 9，新 QP 库 | 初始化间隔保护退出，已恢复；失败记录保留 | 否 |
| hold_v18 | revision 10，同一新 QP 库 | 地面消费读取器超时退出，未起飞/无 LiveStart，参数已恢复、运行时已关闭 | 否 |
| hold_v19 | revision 11，同一新 QP 库 | 地面准备和真实接管成功，活动指令寿命超期，已完成停止与恢复；`complete=false` | 否 |
| hold_v20 | revision 13、v8 任务，native 准备及复制优化 | 212 次计算数值成功，活动指令过期失败；224 组发送/消费及停止恢复已核实 | 否 |
| hold21 | revision 14，同一 v8 任务与数值后端 | 约1.1秒后旧指令年龄63.318ms触发保护；全部56组发送/消费及恢复已核实 | 否 |
| 普通八字 | 必须继承通过的新 hold 同版本 | 待新的完整保持证书 | 否 |
| 快速八字 | 后续独立阶段 | 未启动 | 否 |

### hold_v19：已接管后的活动指令过期

启动标识为 `20261024_111153_4885`，物理运行目录为 `x650_isolated_live_v1_20261024_111240_4886`，由 `progress_v3` 流程推进。真实 `LiveStart` 原点为 **41.164 s**。与 hold_v18 不同，本次已经完成地面准备、起飞与接管，随后因 `C++ output latched fault reason 1`（`STALE_COMMAND`）退出，`complete=false`。

后续原样取回的 [`hold19_failure.json`](startup/hold19_failure.json) 为 **4103 字节**，SHA-256 为 `a9016d2e79cbec7631661f2a6db6799d8dcf7c03be20f5c4602584332d290db4`。文件内保存原运行路径和 8 份来源记录的哈希；它不属于最初的 7 文件 `startup/manifest.json`，原清单保持不变。

| 活动 tick | 已执行/待执行指令 | 原始快照年龄 | 结果 |
|---|---|---|---|
| 3220 | 候选 1 | 26.208560 ms | 新指令正常发送 |
| 3221 | 候选 1 | 46.288549 ms | 重复发送；原始时间不刷新 |
| 3222 | 候选 2 | 44.223783 ms | 新指令正常发送 |
| 3223 | 候选 2 | 64.220177 ms | 超过原 50 ms 寿命，锁存故障，不再发送该活动指令 |

实际记录含 **3 次正常发送**（其中 1 次重复）、**4 次总发送**（含停止零速）和 **4 次消费回执**，并非 3 个不同候选均已下发。候选门记录 2 次接受；worker 的 3 次计算均返回成功状态，完整计算耗时分别为 **22.874、22.468、21.412 ms**。同次 worker 记录没有 gen1/gen2 GC，gen0 仅有约 57/68 µs 的短回收；source pump 最大为 **4.855864 ms**，故障时来源仍新鲜。这次不能归因为 QP 可行性失败、长时间 GC 或再次执行地面初始化。正式的[worker 原始记录分析](preparation/worker_analysis.json)保存三次计算的精确值、输入哈希和诊断边界；其中“数值候选接受”指 worker 返回成功，不代表第三条已经通过候选门或实际执行。

调度链仍要求候选对应的实际发送通知到达后才准入下一次求解。当单次计算和消息传输超过 20 ms 时，结果可能错过下一个输出槽而等待第二个槽；下一轮又从实际发送后开始，导致上一指令虽然计算时未超 50 ms，重复发送时却已过期。上表记录了这一实际相位链。当前不能删除实际发送栅栏来提速，因为数值控制器已更新上一指令和计划状态，未实际发送的候选不得成为已执行历史。

恢复结果已经取得：`release.zero_consumed=true`、`release.writer_exited=true`、`landed_and_disarmed=true`、`diagnostic_parameters_restored=true`、`runtime_closed=true`。它们确认本次短运行退出与恢复完成，不构成完整 30 秒保持证书，也不覆盖所有 pending-zero 故障边界。

### 数值准备与诊断复制：已取得的有限范围证据

`NativeJointMatrices` 的未来 MIT 包批量构造及相关预测准备减少 Python 逐项构造和重复转换，目标是保留同一实际历史、包网格、数值预测与候选检查。以下[离线准备优化报告](preparation/preparation_optimization_v2.json)属于尚包含共享预测器改动的中间版本，使用 hold_v19 的 3 条原始 CDR 快照，按原顺序和原实际历史重放；每组重新构造并 reset 引擎，只有离线执行用的 monotonic 寿命基准重新赋值，不改状态、仿真时间或实际 dt。它保留其原始源码身份，不作为最终版本的计时证明。

该中间报告的完整调用计时未插桩，沿用 worker CPU 87/89/91/97 和 20 ms 绝对计划时刻，迟到不重锚。报告同时含新解释器构造后空闲 0.5 s 的序列，以及同进程上一组回放后重新构造引擎的序列；前者不等于清空 CPU 缓存，后者也没有沿用上一组算法历史。

| 证据范围 | 已取得结果 | 限制 |
|---|---|---|
| 原准备实现，18 次完整 worker 离线调用 | 13.140603–25.405952 ms | 同一小组原始输入反复执行，不是新飞行 |
| 优化后准备，18 次完整 worker 离线调用 | **12.254170–19.107057 ms** | **仍使用旧双深复制 adapter，未包含单次诊断复制优化**；小样本最大值不构成实时上界 |
| 同输入数值比较 | 报告核对的矩阵最大绝对差 0，命令最大绝对差 0；针对性 native 测试标记通过 | 不能替代组合版本的全量软件、388 条回放和物理验收 |
| 单次诊断复制，本地合成树微基准 | 默认两次复制中位 1145.2534 µs，owned 单次复制中位 **564.5442 µs**，减少约 50.7% | 80 条 envelope 与 15×10 预测矩阵的复制边界计时；不含完整 MPC、ROS 或飞行 |
| 复制语义回归 | 本地新增 11 项、ROS Python 共 39/39、原 adapter 9/9 通过；随后服务器 ROS Python 39/39 通过 | 不能代替组合版本完整数值和运行时资格 |

单次复制只由 live 构造器在第一次求解前启用，保持与普通八字两条入口均覆盖；公共 adapter 默认仍为原两次独立深复制。私有 live 路径先拥有整张 allocation 快照，再建立独立的外层 diagnostics 字典，单次结果内部可共享只读使用的子树，绝不与 allocator 或下一步共享；不建立自引用环。worker 仅读取，随后在下一次回调前由原诊断发送器同步序列化成不可变字节。原输入拒绝、命令字段、诊断字段和值均保持，未删除实际发送栅栏或放宽时间门限。

最终复制微基准见 [`ownership_copy_final.json`](preparation/ownership_copy_final.json)；早一版报告 [`ownership_copy_initial.json`](preparation/ownership_copy_initial.json) 保留其原源码身份，不作为最终实现的计时绑定。4 份报告连同来源路径、字节数及 SHA-256 收入[独立清单](preparation/manifest.json)，没有复制原始 CDR 或大矩阵数组，也没有修改原始报告。离线准备计时与本地复制微基准不能简单相减来宣称组合版本已达标。

上述两项修改已作为 **deployment_v24** 合并并部署，源码在本轮检查期间冻结。服务器 ROS Python 39/39 已通过；首轮 `preparation_replay_v1` 回放仅为 **386/388**，记录 index 10 的 native 带插桩调用为 **51.281 ms**，index 58 的 reference 带插桩调用为 **53.334 ms**，均超过原 50 ms 门限，故本轮不能判为通过。这两项报告的是时间拒绝而非数值差异，失败证据仍保留，不能因其带有插桩就忽略。

`preparation_replay_v2` 唯一试验条件调整为将回放绑定至实际 worker CPU **87/89/91/97**，源码与门限不变。第二轮完成 **344 项回归、30/30 矩阵工况、388/388 原记录通过**；此前服务器 ROS Python **39/39** 也通过。这属于中间共享预测器版本独立取得的后端资格；第一轮失败仍保留，最终收窄源码范围后又执行了下述 v3 完整回归，没有把 v2 证书转授给最终版本。

### v7 实际预检与最终优化范围

后端资格通过后已准备 v7 任务与 revision 12，但 `merged_engine_v1` 的实际预检发现：共享 `ts20_prediction.py` 的源码字节已改变，原 8 工况预测器验收所绑定的 `candidate_sha` 不再匹配。预检按原要求拒绝准入，**没有起飞，也不是新的飞行控制失败**。后端 344/30/388 通过不能替代这份共享预测器的独立来源绑定。

随后 `preparation_oracle_v1` 对当时的新共享源码实际重跑原 8 个工况，8/8 通过。该结果属于中间源码，并非最终版本资格。共享源码变化还会改变原 hover/probe 证据身份，因此最终选择更小的改动范围：将相同的批量 `_rollout_native` 实现移入 `NativeJointMatrices` 的专用 override，恢复独立参考 `ts20_prediction.py` 原始字节 `f25e45d755c236cd46c0e4fd2e11fb23fe926bc1b7e286dd87c9ac03414b7b66`。原传播模型、native 二进制库与控制参数未变，也没有修改旧证书或放宽门限。v7/`merged_engine_v1` 预检拒绝及中间 8 工况结果均按历史身份保留。

### 最终 native 实现资格、合并计时与 hold_v20

最终源码提交为 `c22078a`，仅优化 native 预测准备并采用前述单次诊断复制。服务器以最终源码重新执行 `preparation_replay_v3` 与 `preparation_software_v3`；[`final_preparation/manifest.json`](final_preparation/manifest.json) 中 13 份原样报告已逐项核对字节数与 SHA-256。

| 最终版本检查 | 已取得结果 | 原样证据 |
|---|---|---|
| ROS Python 与复制语义 | 39/39 通过 | [`qualification.log`](final_preparation/qualification.log) |
| 软件回归 | 344 项，0 失败、0 错误、0 跳过 | [`native_tests.json`](final_preparation/native_tests.json)、[`package_report.json`](final_preparation/package_report.json) |
| 冻结矩阵工况 | 30/30 通过 | [`matrix.json`](final_preparation/matrix.json) |
| 原始记录重放 | 388/388 通过 | [`recorded.json`](final_preparation/recorded.json) |
| 数值及软件身份绑定 | 最终 native 证明 SHA-256 为 `f07527f471e3a0c272ac5b916c73f52e9ac7be55732f11cb9731a61dbf3a26fb` | [`native_acceptance.json`](final_preparation/native_acceptance.json)、[`software_acceptance.json`](final_preparation/software_acceptance.json) |
| 合并引擎离线回放 | 6/6 成功；完整计算 **12.739778–18.463390 ms**；命令最大绝对差 0 | [`merged_engine.json`](final_preparation/merged_engine.json) |
| 实际保持任务与协议 | v8 / revision 13；启动标识 `20261024_113454_7214`，活动指令过期失败；详见下节 | [`hold_task.json`](final_preparation/hold_task.json)、[`protocol.json`](final_preparation/protocol.json) |

合并计时已包括最终 native 实现和 `owned_allocation_once_v1`，使用 worker CPU 87/89/91/97。两组各 3 次调用分别来自空闲 0.5 s 后的新引擎及第二个新引擎，只重新绑定配置身份与离线 monotonic 寿命基准；报告明确 `no_flight=true`。这 6 次调用不能代替 Gazebo/ROS 活动负载下完整 30 秒的时序、物理末端误差和退出审计，也不构成实时最坏上界。随后 hold_v20 的实际失败再次表明离线调用成功不能单独授予物理闭环资格。

### hold_v20：计算数值成功，活动指令仍过期

本次 launch 为 `x650_isolated_launch_20261024_113454_7214`，物理运行目录为 `x650_isolated_live_v1_20261024_113542_7216`，真实 `LiveStart` 为 **41.632 s**。v8 任务、revision 13、数值证明与源码前后身份已核对。元数据及 launch 均 `complete=false`，主错误为 `C++ output latched fault reason 1`；最后一条 worker 输入任务时间为 **4.444 s**，不能当作完整 30 秒保持。

[独立失败核验报告](hold20/failure_report.json)来自只读取回、逐项校验哈希的原始运行文件；[证据清单](hold20/manifest.json)区分原样源文件、已有派生时序分析与本次独立核验。原样 JSON 和飞行采样 CSV 收入 [`original_sources.zip`](hold20/original_sources.zip)；较大的 TS20 CSV 与原发布轨迹保留服务器原路径、字节数、SHA-256，已在本地逐行核验，没有用摘要计数代替原消费记录。

| 检查范围 | 独立核实结果 |
|---|---|
| worker 记录 | 212 条连续记录，212 次成功，无 QP/非线性候选拒绝；诊断发送与持久化计数一致、零丢弃 |
| 完整计算耗时 | p50 **13.672671 ms**，p95 **17.823232 ms**，p99 **21.127434 ms**，最大 **22.521692 ms** |
| QP 耗时 | 中位 **0.374779 ms**，最大 **3.431521 ms**；QP 不是此次拒绝来源 |
| 候选门 | 211 条连续接受记录；第 212 次虽然数值成功，但没有进入候选门或实际发送 |
| 关节实际消费 | 224 组发送与 224 组回执身份集合完全相同；223 组正常发送（含 12 次重复）及 1 组停止零速，共 448 条单关节命令 |
| 原始计数核对 | 两关节各从 1230 到 1453 连续消费，命令值和消费仿真时间逐项匹配原 TS20 状态；下一普通输出均在前一消费确认之后 |
| 输出所有者 | 检查 3398 条原发布记录，接管至释放期间没有旧所有者输出重叠 |
| 退出恢复 | 最终零速已消费、写者已退出；末次采样 `armed=0, landed=1`；56 项参数值/类型/原始字节恢复一致并同步保存；6 个自有进程退出码均 0，运行时关闭 |

关键尾段见 [`failure_tail.json`](hold20/failure_tail.json)，完整 worker 时序分析见 [`worker_timing.json`](hold20/worker_timing.json)。第 211 次完整计算仅 **14.368327 ms**，但计算结束至候选门收到结果额外耗时 **7.221433 ms**，到达时已经比 tick 3396 晚 **2.371235 ms**；该槽因此重复候选 210。tick 3397 才发送候选 211，此时原始快照年龄 **44.298550 ms**；tick 3398 旧指令年龄达到 **64.300282 ms**，原 50 ms 保护锁存故障。第 212 次使用实际 **40 ms** 的输入 dt，计算耗时 **21.276403 ms**，完成时已经比故障晚 **2.460933 ms**，因此没有下发。

这个“计算结束至候选门”间隔包含 worker 发布边界、消息交付和调度等待，现有记录尚不能把 7.221 ms 全部归因于其中某一项。故障 tick 的来源状态年龄仍在原 25 ms 限制内；活动段只有 8 次 gen0 GC，最大 **75.447 µs**，没有 gen1/gen2 长回收。后续 revision 14 针对活动计算准入节拍修订，并记录发布边界各阶段时间，保留真实发送提交语义、原 20 ms 输出周期及 50 ms 寿命；新执行版本必须重新验收，不能把 hold_v20 片段转作完整保持资格。

本次只核实失败链、全部实际关节消费和退出恢复，不声称完成 PX4 全窗 ULog 或完整物理末端精度验收；零速健康消费也不代表所有 pending-zero 故障场景已具备真机资格。

### revision 14：实际发送驱动下一次准入，补齐阶段时间证据

数值算法仍为 `c22078a` 对应的 native 实现，数值库、MPC 参数、v8 任务 SHA `9ef8c12c…` 及数值证明 SHA `f07527f4…` 保持不变；本次没有重新宣称执行 344/30/388，也没有改写其原始报告。变动范围是执行准入与可观测性，部署前后源码身份见 [`deployment_v26.json`](execution_v14/deployment_v26.json)。

首轮仍等待实际仿真源时间前进至少 20 ms。后续每一轮必须取得匹配上一候选的**实际执行端发送确认**，该确认只释放一次下一轮求解；重复或不匹配确认不能释放新的求解。在此基础上选择已收到且严格更新的真实状态，保留原始发出时间，模型 `dt` 使用实际状态时间差，不强写为 20 ms。取消的是后续叠加的独立“物理时间至少再过 20 ms”条件，避免实际发送节拍之外再等一个传感器帧；没有移除实际发送屏障，也没有提前使用未执行候选的历史。仍只允许一个在途候选，下一次输出前仍须确认双关节实际消费，原状态年龄、完整计算/模型 dt 上限、50 ms 指令寿命及故障锁存不变。

C++ ingress 使用有界预留内存记录来源、准入、候选门和实际发送释放事件；worker 使用预分配标量缓冲记录回调、引擎返回、publish 起止、CDR、pickle/socket 和回调退出时间。两个缓冲都在退出后写盘，活动回调不为这份时间记录写 JSON；溢出显式报告，不覆盖旧记录伪装完整。候选先发布、后诊断复制的原顺序保留。观测的目的，是将 hold_v20 未能分离的“计算结束至门到达”细分，尚不能声称已经消除了全部发布停顿。

新增审计将这些阶段记录与原始 CDR 的序号、session/epoch、原始 issue/compute 时间，以及实际发送释放的对应序号和真实 `dt` 相互核验；重复、缺失、溢出、时间倒序或身份不符拒绝验收。允许正常结束时最多一个已计算但尚未到门的尾候选，不允许中间缺口借此通过。异步订阅可能在 publish 返回前收到消息，审计不强加错误的跨进程返回顺序，也不修正原时间。

| 本次执行层检查 | 已取得结果 | 原样证据 |
|---|---|---|
| 服务器构建 | `build_v26.exit=0`；gateway 38.6 s、reference 1.47 s，2 包汇总 40.5 s | [`build_v26.log`](execution_v14/build_v26.log) |
| C++ 合同测试 | CTest 5/5，包含实际状态准入节拍 | 同上 |
| ROS Python 测试 | 47/47，覆盖真实 dt 传递、阶段时间记录及原诊断语义 | 同上 |
| 阶段时间审计 | 6/6，包含原时间/序号篡改、溢出/缺失、非法阶段顺序与单个尾候选边界 | [`execution_audit_v26_test.log`](execution_v14/execution_audit_v26_test.log) |
| 安装来源核对 | 9 个已安装模块与部署源码字节一致；记录协议及执行二进制 SHA | [`installation_v26.json`](execution_v14/installation_v26.json) |
| 软件传输合同 | `transport_v11` 通过，7 次发送、7 次回执、停止零速已消费；活动间隔 19.999414–20.057059 ms | [`transport_v11/report.json`](execution_v14/transport_v11/report.json) |
| 真实 ROS 进程准入探针 | `admission_probe_v1` 通过、子进程退出码 0；首轮 16 ms 不准入，后续 16/20/24 ms 来源时间差原样传递；错误/重复确认及未发送候选不推进 | [`admission_probe_v1/report.json`](execution_v14/admission_probe_v1/report.json) |

本次 `state_ingress` 二进制 SHA-256 为 `b4f22c6a4fe1078d83680b0b5606b45194e0cb316a57f95d5acac1c40761d8fe`，revision 14 协议 SHA-256 为 `c669828ce717945abda9578b964af2023c8c1ce6fb0025f028e171e951b4a3a6`，均见安装报告。6 份原样文件的来源路径、字节数及哈希见[清单](execution_v14/manifest.json)。准入探针使用真正运行的 ROS 进程和同一二进制，但输入为合成状态/ACK；报告明确 `physical_validation=false`、`eligible_as_stage_gate=false`，不算物理反馈验收。探针入口为 [`probe_x650_ingress_cadence.py`](../../../scripts/probe_x650_ingress_cadence.py)。**hold21已因指令过期失败并恢复，revision14没有完整保持通过结论。**

只有在修复后的版本重新完成所需软件/数值与传输检查，并在新的运行中取得完整 30 秒、全部关节实际消费、PX4 全窗 ULog 编码、单一输出所有者、停止零速确认、降落上锁与参数恢复、独立物理末端和全量数值重放通过后，才能补入新的保持通过报告。hold_v18、hold_v19 和 hold_v20 均维持失败状态；正常恢复、软件回归通过或输出进程一直存活都不能替代完整保持结论。

通过后用[一次普通八字 driver](../../../scripts/run_x650_isolated_normal_sequence.py)绑定明确的新 hold 报告与 SHA，而非选择最新目录；执行完整 32 秒、12 秒/圈两圈、X±0.25 m/Z±0.10 m 的固定参考。失败不自动重跑，不自动进入快速八字。保持至八字之间冻结协议、源码和执行二进制，避免证据在中途改变。

## 6. 真机迁移边界

本阶段验证 X650 URDF、原二连杆、条件 TS20 plant 与实际 PX4/Gazebo 接口；没有取得 TS20-50 在 24 V 实际负载下、Orin 或动捕接口的硬件资格。迁移应复用完整接口契约、冻结工况、原始日志与独立判定方法，在目标硬件重新构建、绑定与测量。

**20 ms 是输出周期，50 ms 是从原始快照起算的最长指令寿命。** 重复旧指令不续期；不足 50 ms 的单次计算不代表能够赶上有效输出槽。

待确认普通关节命令与停止零速重叠的 pending-zero 边界仍未完整认证：最新消费计数可能越过待确认记录，导致停止零速回执不能建立。当前缺少回执会拒绝该次验收，停止新写入者后由原恢复流程接管；这不等于已经取得真机故障停止资格。该边界及通信断流必须另做实际故障注入验证。

工程架构与测试层次见[执行隔离及测试方法](../../X650_EXECUTION_ISOLATION_AND_TEST_METHOD.md)。


### hold21 与 revision 15：明确区分计算超期和故障观察延迟

hold21（launch `20261024_114909_9088`）在真实接管后约 1.1 秒触发原指令年龄保护，未完成保持，普通八字未启动。[原件清单](hold21/manifest.json)及[独立分析](hold21/ANALYSIS.md)包含55次计算、全部关节消费和恢复过程；完整原始CDR、ULog及启动源码归档在该目录。第54/55次engine墙钟耗时为26.923411/23.136564 ms，发布至ingress没有hold20的7ms长尾；第54条首次下发时年龄43.316650 ms，下一槽达到63.318013 ms。56组发送/消费匹配（含停止零速），最终落地解除武装，56项参数恢复，运行时关闭。失败片段不授予末端保持证书。

另一个已复现的问题是Python故障观察被饿死：主执行器交替调用`spin_once(.02)`和`spin_once(0)`，导致Humble就绪回调迭代器反复重建。ACTIVE段546条连续反馈之间没有四类ROS观察回调，故障直到恢复后才进入记录端。1158.116454 ms是故障产生至记录进程接收的延迟，不是DDS到达延迟。修复保留原VehicleAPI默认等待，仅active路径统一为0；独立Node/Executor、depth1的故障监督在主pump前处理并锁存首次同会话故障，完整证据的depth4000和原始CDR继续保留。

[真实ROS回调探针](execution_v15/ros_callback_fairness_v5/report.json)直接使用待部署VehicleAPI和FaultSupervisor：混合等待时8个订阅的服务次数为`[160,160,0,0,0,0,0,0]`，统一0后为`[43,43,43,43,43,42,42,42]`；独立监督在主执行器仍饥饿时以2.678924 ms锁存故障，后续STOPPED未覆盖它。统一等待本身仍有159.374813 ms深队列延迟，不能替代独立监督。软件探针使用私有DDS域、空MAVLink假链接，无仿真或执行端。早期v4错误地假设统一等待能保证50ms内发现深队列故障，该失败报告保留在服务器；v5没有改控制门限。

私有live诊断复制改为按数值图类型复制，保留memo、别名、数组独立所有权及未知类型deepcopy协议。公共默认双副本行为不变。[真实诊断复制微基准](execution_v15/hold_v21_worker_profile_v1/copy_report.json)中位减少约0.8ms；这不是完整控制周期的改进保证。新增6项复制语义测试，ROS Python **53/53**；监督、生命周期与地面准备 **3+5+7**项通过。

[CPU/频率对照](execution_v15/hold_v21_affinity_probe_v1/summary.json)保留三组各55条原记录的一次回放：生产mask、单核87、单核89的中位/最大engine分别为14.481/16.966、11.191/13.277、14.542/17.039 ms；更快的单核87组频率也更高，因此不能归因于单核绑定。回放仅一个线程，实际worker有11线程及Gazebo背景负载；资源协议未改变，未挑选最佳组作为实时上界。

revision15 / v9任务已以当前源码重新取得 **344项回归、30/30矩阵、388/388历史记录**，零跳过，见[资格原日志](execution_v15/preparation_qualify_v4.log)及[15份原样报告清单](execution_v15/manifest.json)。独立参考预测器、两个native库、MPC参数、20ms输出、50ms指令寿命、25ms来源年龄及实际发送提交屏障均未改变。任务SHA为`7da0db083acca05a8be7b3872db6e096fc9435653eef698a15700a6d8e99a286`，协议SHA为`3097556ae513a72a0ebe6f1578d5fc1b2110cc94dfe28775f04c4cd983fad3e4`。hold22已启动，当前尚无完整通过结论；通过独立全窗验收后才准入普通八字，快速八字继续后置。


## 14. revision16：历史时间一致性与拒绝输入证据

[hold22复核](hold22/README.md)记录18次计算数值通过、第19次在输入准入被拒绝。使用原记录重建的同一总线时刻存在7.105427357601002e-15秒浮点差；原C++严格未来比较与Python既有1e-10秒容差不一致。重建通过accepted18全部context字段与原始CDR相等交叉验证，但被拒绝的snapshot19原始CDR未留存，详细限制见其manifest。

本次统一两个历史未来比较，并新增失败时固定大小输入拷贝，关闭时才序列化CDR及源序号、接收与检查时钟。原wall-clock保护、实际dt、模型/参数及task v9不变。5/5 C++测试组、3/3拒绝证据校验器测试及实际私有DDS节点探针通过。探针逐字段核对拒绝输入，并证实错误session不生成证据；原始报告见[execution_v16清单](execution_v16/manifest.json)。该软件证明不等于完整物理保持通过。hold23已完成但未通过，26次求解数值正确仍未满足执行时序，后续依然按完整保持→普通八字→快速八字推进。
