# X650 位置外环对比 · 阶段 A 和 B 执行记录

2026-11-18，阶段 A 已完成。阶段 B 经过 37 次尝试取得 36 条有效飞行轨迹，每个算法与工况组合均有 3 次有效重复。37 次启动前和退出后进程审计均通过。原矩阵第 10 次发生起飞前平台故障，独立补测成功。模型、控制器、参数和协议哈希已核对。平台已停止。

[新旧批次完整对照](../../runs/x650_stage_b_reconciled/COMPARISON.md) · [逐次尝试及补测映射 JSON](../../runs/x650_stage_b_reconciled/comparison.json) · [原调度清单](../../runs/x650_comparison_clean_baseline_v2/manifest.json)

## 复测结果

下表为三维位置 RMSE，单位 cm，每格为 3 次有效飞行的均值 ± 样本标准差。

| 工况 | PID | LADRC | MPC |
|---|---:|---:|---:|
| 悬停 | 5.44 ± 0.64 | 4.90 ± 1.35 | 5.12 ± 0.68 |
| 三轴阶跃 | 9.45 ± 0.36 | 11.96 ± 0.81 | 9.21 ± 0.69 |
| 机械臂运动 | 5.06 ± 0.22 | 5.59 ± 1.63 | 4.76 ± 1.09 |
| 外力扰动 | 7.15 ± 0.55 | 5.65 ± 0.74 | 5.11 ± 0.45 |

**外力工况复现了 LADRC 相对 PID 的优势。** 全程平均 RMSE 低 20.95%，旧批次低 22.01%。10–12 s 施力期间的平均误差峰值分别为 PID 16.31 cm、LADRC 12.17 cm、MPC 7.15 cm。撤力后达到 5 cm 误差带并保持 2 s 的次数分别为 2/3、3/3、3/3。LADRC、MPC 的进入时间均值本轮为 5.67 s、7.25 s，旧批次为 4.14 s、0.85 s。两轮恢复时间的排序发生变化。

**阶跃和机械臂运动给出了不同的开发方向。** LADRC 的阶跃 RMSE 两轮均高于 PID，本轮高 26.55%。机械臂动作工况中，LADRC 从旧批次均值最低变为本轮最高，均值比 PID 高 10.32%。其运动期间峰值也高于 PID（7.89 对 7.08 cm）。目前有重复支持的研究方向是飞机外力抗扰，以及阶跃跟踪和抗扰之间的取舍。机械臂扰动应继续作为独立工况检验。

悬停的最低均值从旧批次的 MPC 变为本轮的 LADRC，二者在本轮仅差 0.22 cm。两轮各组样本量均为 3，噪声序列各自生成，运行仪表和主机负载也有差异。现有数据无法把排名变化归因于桥进程清理。

**时序和物理核验。** 有效测量段共 56,251 次控制调用，超过 20 ms 的次数为 0。PID、LADRC、MPC 计算耗时 P95 分别为 0.138、0.202、1.249 ms。仿真控制步长最大为 21.00 ms，实际指令发送墙钟间隔最大为 40.38 ms。36 次有效飞行均确认降落，已核对各自的 101 项 PX4 参数、rosbag 元数据与 ULog 哈希。9 次机械臂动作均通过实际运动核验，9 次外力请求的施加至清除间隔均为 2.00 仿真秒。

阶段 B 主批次单独统计为 35/36 完成飞行，整批证据标志为 `false`。补测批次为 1/1 通过。汇总按原始调度位置关联补测，同时保留所有 37 次的分母及唯一失败的诊断。阶段 A 的功能飞行另行归档。完成 A、B 后，阶段 C 可沿用本次基线开展预定的四组 LADRC 带宽筛选。

## 阶段 A 验证

| 检查 | 实际结果 |
|---|---|
| 比较实验自动测试 | 45 项通过，涵盖时钟接收停顿、遗留桥进程、异分区隔离、拒绝启动下一试次、审计证据和分段积分。 |
| 旧轨迹兼容性 | 对旧批次 36 条轨迹离线复算，逐次全程三维位置 RMSE 与原归档相同。 |
| LADRC 外力仿真 | 完成起飞、外力施加与清除、降落。测量段 1,500 条记录包含所需时序字段。 |
| 进程审计 | 启动前和退出后相关进程数均为 0。仿真进行中的独立快照检出 10 个相关进程。 |

[功能试验收据](../../runs/x650_stage_a_verification/receipt.json) · [启动与退出快照](../../runs/x650_stage_a_verification/receipt.audit.json) · [运行中快照](../../runs/x650_stage_a_verification/active_snapshot.json)

**运行时证据。** 运行器在 Linux 上记录同一用户、当前 Gazebo 分区中带试次标识或具有仿真进程特征的进程。快照保留 PID、启动时间、进程组、角色及扫描错误。批次读取独立的审计文件，核对文件哈希、试次身份和进程状态。有残留或证据缺失时停止后续调度。

**时序。** 每个周期分别记录仿真控制步长、墙钟发送间隔、反馈接收年龄和接收端观察到的仿真时钟停顿。系统一分钟平均负载表示主机可运行或不可中断任务的平均数量。每次试验的分位数、最大值和样本数保存在 `summary.json` 的 `metrics.timing`。计算耗时分布还计入能取得计时的失败调用。

**分段抗扰。** `arm` 和 `force` 各自报告动作前、动作期间和动作结束后的真值误差，`force` 另有 10–20 s 固定响应窗。`metrics.windows` 给出时间加权 RMSE、峰值、误差幅值积分 IAE（m·s）、采样覆盖。恢复报告保留实际观察截止时间、首次观测是否已在 5 cm 带内，以及是否连续保持满 2 s。

## 阶段 B 实验条件

沿用旧批次的 PID、LADRC、MPC 参数和辨识响应模型。每种算法依次参与悬停、三轴阶跃、机械臂动作及世界系 +X 的 1 N 外力实验，每组重复三次。批次复用旧调度顺序、50 Hz 名义外环、PX4 内环和公共限幅。每次单独启动并退出 PX4 SITL 和 Gazebo。

[旧批次基线](../../runs/x650_comparison_formal_v1/summary.json) · [新批次冻结配置](../../runs/x650_comparison_clean_baseline_v2/controller_config.json) · [新批次响应模型](../../runs/x650_comparison_clean_baseline_v2/response_model.json)

第 10 次调度（首轮 PID 外力）在就绪检查时未收到 MAVLink 心跳。监督器记录 PX4 退出码为 `-11`，Linux 内核记录同一时段发生在 `libzmq.so.5.2.4` 的保护故障。故障发生在起飞和外力指令之前，两端进程审计均通过。其余试次继续使用已有冻结版本，缺少的飞行在独立批次补测，原始失败计入总尝试次数。[第 10 次日志](../../runs/x650_comparison_clean_baseline_v2/trial_10.log) · [监督器日志](../../runs/x650_comparison_clean_baseline_v2/trials/_control/start_20261118_143309.log) · [内核记录](../../runs/x650_comparison_clean_baseline_v2/trial_10_kernel.log)


## 数据与复算

[主批次报告](../../runs/x650_comparison_clean_baseline_v2/README_CN.md) · [独立补测报告](../../runs/x650_stage_b_retry_10/README_CN.md) · [本次复算程序](../../runs/x650_stage_b_reconciled/reconcile.py)

从仓库根目录运行下列命令可重新核对两个阶段 B 批次与旧基线，并在新目录生成同口径结果。

```bash
.amsim/comparison-venv/bin/python3 runs/x650_stage_b_reconciled/reconcile.py \
  --old runs/x650_comparison_formal_v1 \
  --main runs/x650_comparison_clean_baseline_v2 \
  --replacement 10=runs/x650_stage_b_retry_10 \
  --output runs/x650_stage_b_reconciled_copy
```
