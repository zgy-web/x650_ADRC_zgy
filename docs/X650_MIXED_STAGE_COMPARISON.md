# 已通过审计的 PID / 上层 MPC 同阶段结果图

`scripts/plot_x650_mixed_stage_comparison.py` 是结果展示入口，不启动仿真、不拟合模型，也不生成新的实验验收证书。输入必须是同阶段的 PID 分配基线与上层 MPC 各一份已通过的 `mixed_mpc_stage_audit.json`，以及原始 run 目录。当前协议的两组加速度前馈都为零；文件名里的 `pid_ff` 是既有控制器标识，不表示本实验启用了非零加速度前馈。

在保留原始路径的服务器项目目录运行：

```bash
python3 scripts/plot_x650_mixed_stage_comparison.py \
  --pid-audit /absolute/pid_audit/mixed_mpc_stage_audit.json \
  --pid-run /absolute/pid_run \
  --mpc-audit /absolute/mpc_audit/mixed_mpc_stage_audit.json \
  --mpc-run /absolute/mpc_run \
  --output /absolute/new_comparison_directory
```

输出目录必须尚不存在，避免覆盖原始证据或旧图。脚本拒绝未通过审计、物理模型/PID 参数/协议/阶段不同的组合，也拒绝展示输入文件哈希与原审计不一致的记录。脚本检查展示所需的原始输入，不重复执行完整 ULog 验收。

输出如下：

- `mixed_stage_tracking.png` / `.pdf`：末端 XYZ **位置**误差与三维位置误差范数（mm），机体世界 ENU 位置（m），原始机体姿态（deg）。位置误差来自物理记录器的末端坐标减去声明轨迹，不使用速度误差替代。
- `mixed_stage_joints_solver.png` / `.pdf`：两个关节实际角度，以及实际角速度与发送的角速度命令（deg/s）；另画优化器 `solve_ms` 和原控制循环记录的 `compute_ms`。当前接口发送关节速度，图中不存在虚构的角度指令。
- `mixed_stage_paths.png` / `.pdf`：仅在非悬停的八字阶段额外生成。PID 与 MPC 分别使用独立面板，在**世界 ENU 的绝对 XZ 坐标**上叠加原生物理末端与该次运行的解析参考；各面板 X/Z 等比例，标题显式标注该次预先绑定的 ENU 参考中心。下排显示同一次运行的实际 Y 减去声明参考 Y，便于识别离开 XZ 轨迹平面的误差。使用完整测量窗，包含初始保持、平滑启停、两圈轨迹和最终保持，不平移对齐轨迹、不拟合相位，也不将两次运行强行共享一个中心。
- `comparison_manifest.json`：原审计、metadata、task、native、控制命令和诊断文件 SHA256，使用的绘图/计算源码 SHA256，输出图片/PDF SHA256，以及复现原审计的末端 RMS/峰值。它明确标记 `eligible_as_stage_gate: false`。

横轴分别使用每次实验预先声明的测量起点，不寻找最佳相位。各次实验的参考中心由各自起飞后绑定的机体姿态和固定基准位置决定；微小中心差异被保留，不能强制平移两次原始物理轨迹来消除差异。误差计算不减去样本均值，绘图不重新拟合参数，RMS/峰值使用完整原始测量窗并与已通过审计的数值核对。

优化器 20 ms 预算只约束记录的 `solve_ms`。它不包含部分问题构建和 KKT 检查，不能称作全部控制耗时；`compute_ms` 是日志中单独测量的控制代码跨度，也不是整台系统的端到端延迟。两次单独实验的图只能反映这两次已验收仿真，不自动构成统计显著性或硬件性能结论。
