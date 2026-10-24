# hold25 原输入的 A/B/A2 背景负载回放

这次按同一组真实输入进行软件回放，**没有在未解锁的 Gazebo/PX4 背景下复现 hold25 飞行中的计算长尾**。三个条件的计算中位数和高分位接近，均完整处理 1056 条输入，返回命令与原记录的最大绝对差均为 0。

这项结果只适用于本次条件。B 条件的仿真实时因子（RTF）仅约 **0.25–0.28**，hold25 飞行时约为 1；回放本身也没有执行实时 ROS 状态接收、下发、消费确认和记录链。因此不能据此认定真实飞行时序已经修复，也不能排除完整运行负载、核间迁移或频率变化的影响。

## 方法与范围

三轮均使用原生产库 `x650_native_optimization_build_v3/libx650_optimization.so`，SHA256 为 `b9cbdf78c76e92f973800e07c6219611759fcb4471a1a39192c5cbf30f5e2e48`。没有在三轮之间更换单位 Hessian 初始化优化库。回放进程固定在 CPU 87，三个 BLAS 线程数均为 1；这也与 hold25 原工作进程允许使用多个 CPU 的条件不同。

输入来自 hold25 的全部 1056 对 snapshot/proposal CDR，按原顺序初始化及重放。实际状态、任务时间、物理步长、已执行历史和模型参数保留；仅将输入的 monotonic 签发时间重设为离线调用的开始时间，以满足原有 50 ms 门限。该时间重设不能用来验收原实时链路的源数据年龄。

计时前完成解码和固定大小数组分配，构造后先空闲 0.5 s，再按 20 ms **绝对节拍**运行；迟到时不重置下一次截止时间。未插入阶段包装、QP 矩阵复制或 cProfile。每步保留完整的延后诊断计算，控制命令比较放在计时区间外，自然 GC 的开关与阈值没有改变。回放不创建执行器发布端；后台只启动仿真、读取遥测并验证未解锁状态。

| 条件 | 实际安排 |
|---|---|
| A | 无仿真后台，同一旧库重放一次 |
| B | v5 后台通过新鲜遥测及在地面、未解锁检查后，重放一次 |
| A2 | B 结束、后台正常退出且确认所属进程全部清理后，再重放一次 |

三个正式条件各执行一次，没有从多个成功回放中挑选最快成绩。后台前四次启动/诊断失败没有产生 B 样本，原因列于下文。

## 结果

单位为 ms。`engine` 从进入 `engine.process` 到其返回，包含解码和命令准备；`engine + diagnostic` 继续计入完整延后诊断。两者均不是实际 ROS 发布、执行器消费及下一次反馈的完整闭环时延。

| 指标 | A 无后台 | B 未解锁后台 | A2 清理后台后 |
|---|---:|---:|---:|
| engine 中位数 | 9.532 | 9.485 | 9.517 |
| engine P99 | 12.990 | 12.405 | 12.473 |
| engine 最大值 | 15.101 | 14.235 | 13.650 |
| engine + diagnostic 中位数 | 11.231 | 11.205 | 11.205 |
| engine + diagnostic P99 | 14.818 | 14.185 | 14.381 |
| engine + diagnostic 最大值 | 16.293 | 16.602 | 15.353 |
| 开始时刻最大迟到 | 4.307 | 1.065 | 0.592 |
| 通过 / 总调用 | 1056 / 1056 | 1056 / 1056 | 1056 / 1056 |
| 命令最大绝对误差 | 0 | 0 | 0 |

三轮都没有单次 engine 或 engine + diagnostic 超过 20 ms。详细墙钟、线程 CPU、实际 CPU 编号、原始与重设签发时间，以及每条命令误差均保留在压缩包内的完整原始 rows 中。统计和逐文件哈希见 [`comparison.json`](comparison.json)。

## 后台状态、交叉任务与清理证据

v5 的 [`ground_before.json`](background/ground_before.json) 和 [`ground_window.json`](background/ground_window.json) 均满足 `passed=true`、`ground_disarmed_verified=true`，没有发布控制命令或写参数。两段观测分别得到 RTF 0.249525 和 0.275464。

[`status.json`](background/status.json) 记录 `completed_background_observation`、`released=true`、`runtime_file_absent=true`、`remaining_owned_processes=[]`；退出码为 0。协调器在核实以上条件之后才启动 A2，后台结束与 A2 首个计时调用分别为 monotonic `59607106843552915` 和 `59607112841505775` ns。

同期的 worker placement 软件检查没有与 B 重叠。[其原始回调记录](placement_probe/child_final.json) 中最后一个回调结束于 `59607005885682080` ns，B 首个计时调用开始于 `59607068326688290` ns，相差 **62.44100621 s**。[原检查报告](placement_probe/report.json) 及其 SIGINT 时刻一并保留。该核验排除了这一已知检查的重叠，不代表证明服务器上完全没有其他任务。

## 未形成 B 样本的前四次准备失败

原始日志保留在服务器 `/root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/isolated_live_20261024/unarmed_load_v1` 至 `unarmed_load_v4` 及同级 runner/log 文件，本归档不复制整份启动日志。

| 尝试 | 原因与处理 |
|---|---|
| v1 | 诊断启动脚本遗漏创建 `plant_logs`，仿真 topic 发现失败；补齐目录。 |
| v2 | 固定 5 s 观测尚不足以等 DDS 状态 topic 创建；未通过地面判据，清理退出。 |
| v3 | 已有完整状态流且未解锁，但固定观测结束时 land 消息过旧；保留失败，改为等新鲜、完整的地面状态再结束检查。 |
| v4 | 新诊断 wrapper 遗漏原 `PYTHONPATH`，无法导入 `pymavlink`；这是诊断脚本配置错误。清理后先单独验证导入，再复用原 `hold_v25.sh` 完整环境头启动 v5。 |

以上失败不被计入 A/B/A2 性能样本，也不被作为控制算法失败或飞行验收。v5 的独立导入检查原件见 [`unarmed_load_v5_import_check.log`](background/unarmed_load_v5_import_check.log)。

## 原件与复核

[`original_reports_and_scripts.zip`](original_reports_and_scripts.zip) 保留三个完整逐轮报告、原回放脚本、v5 地面/清理证据以及 placement 检查的 **17 份原始文件字节**。ZIP 为 264,492 字节，SHA256 为 `f2c5ec0a946ddcdff8de4e387bc937bfb4280912495873488c482db0956ba67f`。[`raw_archive_manifest.json`](raw_archive_manifest.json) 逐项列出压缩包条目的字节数和 SHA；大 rows 只压缩归档，没有裁剪或重写。

为了便于直接查看，小报告和原脚本同时在目录内展开。回放脚本为 [`replay.py`](replay.py)，后台驱动与启动环境在 [`background/`](background/)，追加取回的实际协调脚本为 [`environment_coordinator_v5.py`](environment_coordinator_v5.py)。这些脚本保留服务器上的原路径，作为本次复现入口和执行记录，不是可直接迁移到真机的启动命令。

[`retrieval_receipt.json`](retrieval_receipt.json) 绑定展开文件及原压缩包；[`verify_archive.py`](verify_archive.py) 只读取本目录，重验原件哈希、1056 条计数、命令一致性、库/输入/CPU条件、后台清理顺序和交叉检查时间。运行它不会启动仿真或回放计算。
