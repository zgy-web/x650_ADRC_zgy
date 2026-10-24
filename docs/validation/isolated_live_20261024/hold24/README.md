# Hold24：反馈新鲜度保护触发，故障模式交接与恢复完成

本目录复核实际 PX4/Gazebo 记录 `x650_isolated_live_v1_20261024_124523_18157`，对应 launch `x650_isolated_launch_20261024_124434_18155`、execution revision 17、任务 SHA-256 `c04a3bbf484b4770d3c0425ec4b2613dc12008761f02e429187979c07534a181`。**本次未完成 30 s 末端保持，不产生完整保持准入证书**。`hold_v24.exit=1`，launch `complete=false / exit_code=2`；普通、快速八字均未由 `progress_v8` 启动。

可复算的独立摘要见 [review_summary.json](review_summary.json)。LiveStart 的物理时间原点为 44.38 s，最后反馈的物理时间为 49.3 s，对应任务进度 4.92 s；首个 writer fault 在 LiveStart 后 4.959495189 s 的 host monotonic 时间发生。这两种时间保持各自语义，没有拟合或互换。

首个故障是 tick 3333 的 `reason=2 / STALE_SOURCE`。原始 source、ingress、worker 和 dispatch 记录以 source sequence 965 关联；C++ 的反馈新鲜度以 **source issued 时间**为起点：

| 事件 | 原始 host monotonic 纳秒或直接差值 |
| --- | --- |
| source 965 issued | 59602891521603876 |
| source publish 返回 | 59602891523943739 |
| ingress 接收 source 965 | 59602891523958312 |
| tick 3333 首 fault | 59602891552399967 |
| issued → ingress 接收 | 2.354436 ms |
| issued → fault | **30.796091 ms**，超过 25 ms 反馈新鲜度上限 |
| ingress 接收 → fault | 28.441655 ms |
| supervisor producer → callback observation | 17.011097 ms |

该时刻候选 247 的指令年龄同为 30.796091 ms，仍低于 50 ms 指令寿命上限。这里不能把 `reason=2` 写成求解超时或指令 50 ms 过期。supervisor 的时间是独立回调观察时间，不是网络到达时间或诊断文件写盘时间；本审查没有仅凭该延迟推断线程、GIL 或网络根因。

247 次 worker 请求、247 次数值结果和 247 次候选门检查全部通过，无候选拒绝。engine wall time 中位数 **9.747985 ms**、最大 **13.832118 ms**；完整 callback 中位数 **12.035491 ms**、最大 **16.173027 ms**。末轮 247 的 engine 为 9.051705 ms。新增加的后置诊断阶段已逐行补核：所有 247 行均满足非零的 `publish_end ≤ diagnostic_complete_begin ≤ diagnostic_complete_end ≤ snapshot_cdr_begin`，且下一 callback 不早于上一 callback 退出；`worker_final.diagnostic_completion_error` 为空。后置阶段仍占用 callback，单靠发布完成不能推断下一轮已可执行；本轮末尾阶段和首 fault 的关联可由原时序重算。

实际发送为 **246 条普通命令 + 1 条停止归零**，共 247 条消费回执。候选 247 虽已通过检查，未作为普通命令执行。两关节共 494 次消费，逐关节 counter 连续覆盖 1194–1440；原始插件 CSV 中的 counter、命令值和消费时间均与发送及回执匹配。首 fault 后无普通输出，归零在 fault 后 0.133855 ms 发出、6.799674 ms 实际消费。3397 条原控制发布 trace 未发现 LiveStart 至 writer 释放期间的输出权重叠；原始 ROS CDR 与材料化记录的全部字段一致（dispatch 3335、send 247、event 247、receipt 247）。这些检查由 [review_remote.py](review_remote.py) 对完整服务器原件执行，结果和大文件 hash 保留在 [remote_writer_recovery_review.json](remote_writer_recovery_review.json)。

新故障恢复分支在本次真实失败中完成了验证：先确认停止归零被消费、writer 已退出，再发送 `DO_SET_MODE=176`、参数 `[1,4,3]` 请求自主悬停。记录包含本次的新 ACK、target 1/1、新状态 callback 序号及递增的 PX4 时间戳；约 **52.177 ms** 内确认 `nav_state=4` 后才折臂，随后发送 `NAV_LAND`。没有重放过期 MPC 候选，也没有第二个并行 setpoint writer。

完整 ULog 的 120 条 vehicle status 记录未观察到 `failsafe=1`。原始 ULog 时间中，49.552 s 发出自主悬停命令，49.568 s 进入 AUTO_LOITER；50.432 s 的 offboard signal lost 标志出现在已切自主模式之后，不构成此次恢复的 PX4 failsafe。50.716 s 发出 LAND，50.732 s 进入 AUTO_LAND，55.044 s 报告 landed，57.056 s 报告未解锁。最终飞行记录同样为 `armed=0 / landed=1`。六个隔离子进程均返回 0，launch `runtime_closed / sources_unchanged / runtime_binaries_unchanged` 均为 true；56 项 PX4 参数的 value、type、raw hex 和来源身份精确恢复，接收时间戳自然不同。该结果证明此记录中的故障退出流程，不扩展为真机安全或完整保持合格结论。

资源片段以最邻近外侧样本包围 LiveStart 至 fault，共 99 个样本；`nr_throttled` 与 `throttled_usec` 增量均为 0。这排除了该括窗内新增容器配额节流的证据。它不证明单次操作的实际 CPU 频率，也不确定 source 主循环停止发布的原因。

`manifest.json` 区分服务器原始字节、派生分析和复算脚本。20 份小型原件与 `retrieval_manifest.json` 的 bytes/SHA-256 全部核对；`resources.json` 和 `flight_tail.json` 是明确标注源 hash 与选择范围的派生片段。完整 dispatch、CDR、插件 CSV、控制发布 trace 和 ULog 留在报告列明的服务器路径，不能把源文件清单误认为所有原件均已复制。本目录独立审计清单不覆盖后续可能添加的 `followup/`；该目录另有自己的来源与检索清单。

在本目录离线复算：

```bash
python summarize_review.py --verify review_summary.json
```

`review_remote.py` 可在原服务器的 ROS Humble、`ros2_px4_ws` 和 **`flyarm_ros2_live_v1_20261023`** 环境下只读复核完整证据；不使用旧 `structure` install，不启动仿真、不解锁、不发执行命令。原件和摘要没有通过改写时间戳、放宽阈值、截取有效前段来获得完整保持通过结论。
