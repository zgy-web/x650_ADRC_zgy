# Hold25：计算错过发送时隙，未完成完整保持

本目录归档实际 PX4/Gazebo 记录 `x650_isolated_live_v1_20261024_131702_20177`，对应 launch `x650_isolated_launch_20261024_131614_20176`、execution revision 18。任务 SHA-256 仍为 `c04a3bbf484b4770d3c0425ec4b2613dc12008761f02e429187979c07534a181`，C++ 数值库仍为 `b9cbdf78c76e92f973800e07c6219611759fcb4471a1a39192c5cbf30f5e2e48`。MPC 参数、20 ms 发送周期、25 ms 状态新鲜度及 50 ms 指令寿命上限均未因本次追踪而改变。

**本次未完成 30 s 末端保持。**最后实际反馈的任务时间为 21.168 s，`hold_v25.exit=1`，launch `complete=false / exit_code=2`。这里归档的是完整保留的失败记录、实际执行检查与成功恢复证据，不是保持通过证书，也不能作为普通或快速八字准入。

独立复算摘要见 [review_summary.json](review_summary.json)。Hold24 的首故障为 `STALE_SOURCE`，当时 source issued 年龄超过 25 ms；**Hold25 没有复现该 source 暂停**。本轮首故障是 tick 4070 的 `STALE_COMMAND`：最新 source 4098 的 issued 年龄仅 2.828795 ms、接收年龄 0.254776 ms，但上一条命令 1055 的年龄已经达到 63.807141 ms。

| 执行事件 | 由原始时钟和相同序号复算的结果 |
| --- | --- |
| 1055 engine 耗时 | 18.840497 ms |
| 1055 engine 返回相对 tick 4068 scheduled | 晚 2.289303 ms |
| 1055 到达候选门相对 tick 4068 actual | 晚 2.258476 ms |
| tick 4068 | 重复执行 1054，当时指令年龄 45.084826 ms |
| tick 4069 | 1055 首次发送，实际发送年龄 43.966654 ms |
| 1056 engine 耗时 | 20.977180 ms |
| tick 4070 | 1055 达到 63.807141 ms，锁存故障并禁止普通输出 |
| 1056 engine 返回相对首 fault | 晚 1.704397 ms，未进入候选门、未发送 |

1056 次数值计算均通过，候选门实际接受 1055 次、拒绝 0 次；“数值通过”没有被算成“已经发送”。1055 和 1056 的完整 callback 分别耗时 22.306755、24.740122 ms，其中 thread CPU 为 22.295037、24.728932 ms，主要是实际 CPU 执行时间。相邻两轮错过固定发送时隙，与“必须等上一条真实发送后才推进下一求解”的顺序一起使旧指令到达寿命上限。这里不把 CPU 时间增长的具体原因直接归结为硬件频率、调度或某个函数；那些判断需要更窄的性能证据。

新增 source_activity 固定缓冲记录完整保留了失败位置：

- 7136 条 pump 全部完成记录，phase 时间有序，无 overflow、重复标记、重叠或未完成行；最大 pump 为 8.088555 ms，最大泵外间隔为 0.064768 ms。
- 4098 条 source 均逐条绑定到对应 pump 的实际 read→publish 区间。最后一个失败 pump 7136 停在 `supervision`，异常、freeze 和 finally 结束时间都保留，恢复阶段没有覆盖它。
- 38 个进程级 GC 原始事件完整组成 19 对，仅 generation 0/1；最长 0.434389 ms。故障前后 100 ms 没有 GC 事件，GC 开关和阈值前后一致。
- 1056 条 worker 后置诊断记录也均满足非零的 `publish_end ≤ diagnostic_complete_begin ≤ diagnostic_complete_end ≤ snapshot_cdr_begin`；下一 callback 不早于上一 callback 退出，completion error 为空。

实际执行证据为 **1057 条普通发送（含 2 次重复）+ 1 条停止归零，共 1058 条消费回执**。两个关节共 2116 次实际消费，counter 连续覆盖 1205–2262。原始插件 CSV 的 counter、命令值、消费时间与发送和回执逐条相符；全部材料化 ROS 字段与原始 CDR 一致（dispatch 4071、send 1058、event 1055、receipt 1058）。首 fault 后无普通输出、无旧控制器并行发布，归零在其后 0.132929 ms 发出、4.395740 ms 被实际消费。独立 supervisor 在 0.387149 ms 后观察到首 fault；这是回调观察时间，不是网络到达或文件写盘时间。

新的恢复分支再次在真实失败中完成：确认归零消费与 writer 退出后，请求 AUTO_LOITER，并用本次新 ACK、新 callback 序号、递增 PX4 时间戳和 `nav_state=4` 确认自主悬停，耗时约 26.146 ms；之后折臂、LAND。完整 ULog 的 152 条 vehicle status 记录未观察到 failsafe。原 ULog 时钟在 65.528 s 进入 AUTO_LOITER、66.668 s 进入 AUTO_LAND、70.980 s 报告 landed、72.992 s 报告未解锁。最终飞行 CSV 同样为 `armed=0 / landed=1`，六个隔离子进程均返回 0；runtime 已关闭，源码和二进制绑定未变化。56 项参数的 value、type、raw hex 和来源身份逐项精确恢复；接收时间戳不用于参数值相等判断。

## CPU 资源事实及其解释边界

本轮原始 host 快照与运行后只读硬件检查的 hostname 均为 `autodl-container-edeb42a3f7-c2d98427`。运行后读取到的 CPU 型号为 **Intel Xeon Platinum 8352V @ 2.10 GHz**。可见拓扑为 2 个 socket、64 个物理核心、128 个逻辑 CPU；容器的 `cpu.max=1200000 100000`，等价于 **12 个 CPU 的时间配额**。这不是 128 个独占 CPU，也不表示角色 affinity 内的 CPU 都被当前容器独占。

| 运行角色 | 本次 launch 绑定的允许逻辑 CPU |
| --- | --- |
| source | 17 |
| worker | 87、89、91、97 |
| dispatcher | 93 |
| ingress | 94 |
| consumption | 95 |

本项目的后台进程避开上述 CPU 及其 SMT 兄弟，但这只是对本项目拥有的进程实施的亲和性分配；没有证明宿主机上的其他租户被隔离，也没有声称硬实时。worker 的四 CPU mask 是允许范围，不等于一次求解实际并行用了四核。

资源记录以最邻近外侧样本完整包围 LiveStart 到首 fault，共 413 个样本、21.255874294 s。该窗口累计消耗 77.839106 CPU-s，平均约 **3.662 个 CPU**；`nr_throttled` 与 `throttled_usec` 的增量都为 **0**。因此本轮没有新增容器配额节流的证据。仅凭平均占用低于 12，也不能保证每个 20 ms 时隙都满足端到端时限；这里保留的 worker-mask 频率是粗粒度采样，不能指定某轮实际跑在哪个 CPU、用了什么频率。**本次可以证明特定尾部调用未及时赶上时隙，不能证明这类 CPU 硬件必然做不到，也不能外推 Orin 真机已经满足要求。**

## 原件与复算

`originals.zip` 保留 19 份服务器原始文件的完整字节，其中包括新增 source_activity、完整 worker/source/ingress 时序、发送/回执、恢复状态、参数前后值和原始 host 快照。压缩前约 10.26 MB、ZIP 约 1.80 MB。每个 ZIP member 的原路径、字节数和 SHA-256 见 [originals_manifest.json](originals_manifest.json)，本地已逐个解压到内存重新核验；ZIP 压缩没有改写 JSON 字节。

完整 CDR、插件 CSV、控制发布 trace 和 ULog 保留在 [remote_writer_recovery_review.json](remote_writer_recovery_review.json) 指定的服务器路径及 hash；本次没有复制这些大文件，也没有删除服务器数据。该报告由 [review_remote.py](review_remote.py) 在完整原件上只读核验，并包括 ULog 原始时间的恢复提取。`resources.json`、`flight_tail.json` 和 `host_postrun.json` 都明确标为派生记录；它们不是原文件逐字节副本。不同时间域没有通过拟合互换。

本地可复算，无须 ROS、服务器或解压到文件系统：

```bash
python summarize_review.py --verify review_summary.json
```

服务器完整原件检查脚本需要已有 ROS Humble、`ros2_px4_ws`、`flyarm_ros2_live_v1_20261023` 和 pyulog 环境；它不会启动仿真、解锁或下发执行器命令。顶层 `manifest.json` 绑定本目录各文件，区别原始 ZIP 容器、ZIP 成员原件、派生结果和审查脚本。
