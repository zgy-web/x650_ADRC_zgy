# Hold23：指令寿命超限，退出与恢复完成

本目录记录 2026-10-24 的实际 PX4/Gazebo 尝试 `x650_isolated_live_v1_20261024_121409_13665`，对应 launch `x650_isolated_launch_20261024_121323_13664`。**本次没有完成 30 s 末端保持**：`hold_v23.exit=1`，launch 的 `complete=false / exit_code=2`。`progress_v7.json` 在保持验收前停止，未进入普通或快速八字。此资料不作为完整保持或真机准入证书。

独立复核见 [review_summary.json](review_summary.json)。原始 LiveStart 的物理时间原点为 41.512 s；其后约 0.632 s（同主机 monotonic 时钟），C++ writer 在 tick 3225 首次锁存 `reason=1 / STALE_COMMAND`。当时最新指令 25 的寿命为 **62.673571 ms**，超过冻结的 50 ms 上限；最新反馈的接收年龄只有 2.708026 ms。26 次 worker 数值计算均 accepted，候选门实际接纳 25 次、拒绝 0 次；第 26 次计算结束晚于首个 writer fault，不能算作已执行命令。

| 关键事件 | 原始时钟计算的结果 |
| --- | --- |
| 候选 25 的 engine wall time | 20.278171 ms |
| 候选 25 的 engine_return 相对 tick 3223 scheduled | 晚 0.853766 ms |
| 候选 25 的 publish_end 相对 tick 3223 scheduled | 晚 1.138701 ms |
| tick 3223 | 重复候选 24，年龄 44.062831 ms |
| tick 3224 | 候选 25：判定时年龄 42.669545 ms，实际发送时年龄 42.850754 ms |
| tick 3225 | 候选 25 达到 62.673571 ms，禁止普通输出并锁存故障 |
| 候选 26 的 engine_return 相对首 fault | 晚 0.171874 ms |
| 独立 supervisor 接收首 fault | producer 后 0.898562 ms |

上述时间区分计划 tick、实际判定、publish 返回、候选门接收和实际 send。publish 返回并不证明 DDS 已交付；本档案通过相同序号连接 worker、gate 和 writer 记录。supervisor 的 0.898562 ms 来自独立回调时间，**不是诊断文件写入时间**。本轮没有再次出现 hold22 的上下文浮点边界拒绝。

实际执行和恢复方面，30 条发送包含 29 条普通发送（其中 4 次重复）和 1 条停止归零，30 条消费回执全部配对。两个关节共 60 次命令消费，逐关节 counter 连续覆盖 1180–1209；独立检查了原始插件 CSV 中的值、counter 和时间。首 fault 后没有普通输出，归零发送在其后 0.156385 ms，归零消费回执在其后 6.285428 ms。3343 条原控制发布记录未发现 LiveStart 至 writer 释放期间的输出权重叠。原始 ROS CDR 与材料化记录的全部字段一致：dispatch 3226、send 30、event 25、receipt 30。

六个隔离子进程均返回 0，launch `runtime_closed=true`，源码和二进制绑定均未变化；56 项 PX4 参数的 value、type 和 raw hex 全部恢复一致。最终飞行记录为 `armed=0 / landed=1`，ULog 也独立确认着陆和退出解锁状态。**恢复过程中仍出现短暂 PX4 failsafe**：原始 ULog 在 43.216 s 记录 offboard 信号丢失，43.228 s 进入 `failsafe=1 / nav_state=5`，43.900 s 进入 AUTO_LAND 并清除 failsafe；47.764 s 报告 landed，49.780 s 报告未解锁。这里不声称“无 failsafe 退出”，也不将这些 ULog 时间拟合到 host monotonic。

资源监测完整覆盖 LiveStart 至 fault。其最外侧括窗包含 14 个样本、0.678221 s，cgroup 的 `nr_throttled` 和 `throttled_usec` 增量均为 0，容器平均约使用 3.55 个 CPU（配额为 12 个）。这排除了该括窗内新增配额节流的证据；粗粒度频率采样不能证明某次计算的 CPU 频率，也不能单凭这些数值归因调度延迟。

本目录保留小型原件及可复算材料：

- `manifest.json` 区分服务器原始字节、派生分析和审查脚本；原件 SHA-256 与读取时的 `retrieval_manifest.json` 对照。该检索清单也包含本目录刻意未复制的 CSV/完整资源日志，不能把清单条目误认为所有原件都已归档。
- `isolated_live/worker_timing.json`、`ingress_timing.json`、`source_timing.json` 和 send/receipt/event 等为服务器原件；`remote_writer_recovery_review.json` 包含末尾 writer 原字段、原始大文件的 SHA-256、完整 CDR/消费复核和 ULog 恢复提取结果。
- `resource_window.json`、`flight_tail.json` 是标明来源 SHA-256 和选择范围的派生片段；JSON 重编码后的片段不是源日志原始字节。
- 完整 dispatch、插件 CSV、控制发布 trace、CDR 和 ULog 保留在报告列明的服务器路径。本次不重复复制大型文件；离线摘要与原始文件级复核是不同证据层次。

本地复算：

```bash
python summarize_review.py --verify review_summary.json
```

`review_remote.py` 为只读的原始证据复核脚本，可在已有 ROS 2 消息和 pyulog 环境的原服务器上重新执行并核对报告中的源文件 hash；不会启动仿真、解锁或发执行命令。它用于复核这一失败记录，不产生新的实验或验收证书。
