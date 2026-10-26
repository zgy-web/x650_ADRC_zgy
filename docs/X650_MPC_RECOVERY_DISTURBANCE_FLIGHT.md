# X650 可信中心恢复后端：整段受扰闭环验证

本实验将 ABI 2 C++ MPC 接到实际 PX4 SITL、Gazebo 和两关节 TS20 条件执行器。**本轮真实施扰运行在 8.604 s 候选速度余量检查处拒绝，完整 24 s 验收未通过。** 它承接 [可信中心恢复的软件验收](X650_MPC_TRUSTED_CENTRE_RECOVERY.md)，不以固定输入回放代替闭环物理实验。结果和原件索引见 [本轮交付记录](validation/cpp_recovery_flight_20261026/README.md)。

## 原工况与版本绑定

仍使用 `x650_cpp_robust_wrench_v1` 场景，24 秒快速 XZ 八字，X/Z 幅值分别为 0.25/0.10 m，8 秒一圈、两圈，末尾有保持段。任务文件与原受扰实验逐字节相同，SHA-256 为 `7b168599e2a182f1a3d3dd860b88d2f87cfb0c23ca258964c0126f4c499af388`。

| 任务时间 | 世界坐标系外扰 |
|---|---|
| 7.0–7.6 s | X 向 +1.5 N |
| 13.0–13.6 s | X 向 −1.5 N |
| 20.5–20.9 s | Y 轴 +0.04 N·m 自由力偶 |

矩形脉冲为左闭右开区间，使用原生物理步时钟和不可变 LiveStart 起点。施加点为 `base_link` 自身质心；记录同时检查世界坐标力和自由力偶，不能把总力矩中的力臂项误作额外扰动力偶。模型总质量仍为 3.495 kg，精密 EV、PID、QP、TS20 模型、MPC 参数与限制保持。没有注入延时。

新容器只重绑 CPU 角色、worker 执行线程和辅助线程的亲和性。原 20 ms 绝对下发周期、40 ms 最大实际发送间隔、25 ms 状态新鲜度、50 ms 候选与控制时限，以及异步消费进展预算均保留。关节信赖域半径仍为 0.02。

`simulations.x650_cpp_recovery_v1.launch` 使用新的软件准入文件绑定新 Core、匹配 ABI 2 worker、关键执行源码、380 次原始回归及 379 条实际 ROS worker 回放。原有快速八字证书用于检查已准备的物理条件；新准入明确 `physical_acceptance=False`，不能沿用旧二进制的闭环认证。完整新物理验收必须另做。

## 施扰启动竞态修复

首次运行控制轨迹完成，但施扰器状态为 `FAILED: stod`，`anchored=False`、`task_rows=0`，因此这次运行不属于有效受扰实验。原始目录、拒绝审计和失败解释均保留。

原因是原启动代码先独占创建最终 `disturbance_start.txt`，再写入并关闭文件。独立 Gazebo 物理线程可能在创建与写完之间读到空字符串，导致 `std::stod` 抛错。此故障不来自 MPC 求解。

修复只改变标记文件的发布：在同目录以独占方式写 staging，完整关闭后使用 `os.link(staging, final)` 一次发布。最终名字已存在时明确失败，不覆盖原件；异常只清理本次成功创建的 staging。文件系统不支持硬链接时也明确失败。原 `t0_sim_s` 内容、激活前后 50 ms 仿真时间检查、插件二进制与任务波形不变，没有增加每周期往返等待。

反例覆盖物理读者在空文件、部分写入、完整但尚未关闭时看不到最终名，以及已存在最终文件、写入失败和临时名称冲突。后验审计继续要求完整逐物理步的波形与每个脉冲冲量；启动成功不能替代这项检查。

## 验收口径

`scripts/audit_x650_recovery_robustness.py` 在新目录产生独立报告，不修改原件。它核对：

1. 当前运行的任务、源码、库、工作进程、CPU 角色、原始 CDR 和不可变起点。
2. 完整物理时间窗内的末端同刻误差、关节预测与实际状态、姿态及原安全包络。
3. 每个绝对下发槽、真实发送时刻、候选年龄、状态年龄和源快照身份。
4. 每关节逐物理步的命令采用账本、明确证明的 supersession，以及双关节最终零命令采用后独占 writer 释放。
5. PX4 不同 PVA 指令值在 ULog 中的编码、退出接管、落地、解除锁定及参数还原。
6. 原生施扰记录的起点、完整时间窗、三段逐步幅值、力偶和各自冲量。

`latest_value_v1` 允许较新指令替代尚未采用的旧值，必须有完整账本说明；不能把“消费记账通过”表述为每条发送都被电机执行。这里检查的是 Gazebo 关节插件的实际采用，也不是硬件 CAN ACK。PX4 的重复同值 setpoint 也不能靠 ULog 值相同逐条确认。

性能规则仍是：每段撤扰后，末端对**原世界坐标系、同一时刻参考**的三维绝对误差进入 10 mm 以内，并持续至少 1 秒，在下一段外扰或 24 秒任务结束前完成。不得减去脉冲前偏置、拟合平移、对齐相位，或使用接管和落地样本补齐恢复段。整体 RMS 使用时间加权。

报告分开列出 `integrity_and_execution_passed` 和 `recovery_performance_passed`，二者均为真才有 `overall_disturbance_test_passed=True`。不能只凭正常退出、完整控制窗或宽安全包络通过就宣称恢复精度通过。

Compact 实时记录保留实际候选、恢复中心和最终 native 节点；独立审计核对中心顺序、参考积分、变化率、连续包络和节点制动余量。完整 QP 矩阵与局部模型误差由绑定的 Full 软件回放证据覆盖，不能声称本轮 Compact 已记录并重新审过全部矩阵。若新运行没有触发恢复分支，也不能将闭环结果归因为恢复分支的因果改善。

## 复现入口

在已有 ROS 2、PX4 消息与匹配 ABI 2 overlay 的服务器环境中，使用本轮清单中的真实绝对路径和 SHA：

```bash
python3 -m simulations.x650_cpp_recovery_v1.launch \
  --task /absolute/inputs/task.json \
  --startup-manifest /absolute/inputs/startup_manifest.json \
  --startup-sha256 VERIFIED_STARTUP_SHA256 \
  --qualification-manifest /absolute/inputs/qualification_manifest.json \
  --protocol /absolute/inputs/protocol.json --dry-run
```

通过预检后，用同一命令去掉 `--dry-run`。每次创建新的 launch/run 目录，禁止覆盖失败原件。结束后再执行：

```bash
python3 scripts/audit_x650_recovery_robustness.py \
  --launch-dir /absolute/new_launch --run-dir /absolute/new_run \
  --output /absolute/new_audit

python3 scripts/plot_x650_recovery_disturbance.py \
  --launch /absolute/new_launch --audit /absolute/new_audit/report.json \
  --output /absolute/new_plots

python3 -m pytest -q \
  tests/unit/test_x650_cpp_robustness_disturbance.py \
  tests/unit/test_x650_cpp_recovery_qualification.py \
  tests/unit/test_x650_recovery_robustness_audit.py \
  tests/unit/test_x650_recovery_disturbance_plot.py
```

该实验属于精密真值辅助 EV 的单一仿真工况。它不证明任意外扰下的递归可行性、Orin 最坏执行时间、动捕实际误差或 TS20-50 在 24 V 带载时的硬件模型精度。真机接口和计算时限仍须按 [迁移测试方法](X650_EXECUTION_ISOLATION_AND_TEST_METHOD.md) 独立验证。
