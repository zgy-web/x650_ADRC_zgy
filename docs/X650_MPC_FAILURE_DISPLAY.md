# MPC 未完成实验的原始时序诊断图

```bash
python3 scripts/plot_x650_mpc_failure.py --run /absolute/failed_run --output /absolute/new_output
```

仅接受 `controller_metadata.json` 明确为 `complete: false`、`phase: failed` 的 MPC 运行。测量窗从实际记录的 `task_start_sim_s` 开始，到最后一条成功发送的 `measure` 控制记录结束，不依赖缺失的正式 `task_end_sim_s`，不混入失败后折臂、降落或恢复数据。要求原始 native 物理样本覆盖这一段；不填补缺失数据。

输出 `mpc_failure_partial.png`、同名 PDF 和 `failure_display_manifest.json`。六幅子图分别展示末端 XYZ 位置误差、三维位置误差范数、机体 Y 相对固定基准及末端 Y 相对声明目标的误差、q2 与软下限、q2 实际角速度与已发速度命令、优化器和控制代码记录的计算耗时。原始关节命令时间是控制记录时刻，并不冒充已独立测得的 Gazebo 接收时刻。

参考由本次固定基体位置、绑定姿态和机械臂几何计算；只使用原始模拟时间，不减均值、不寻找最佳相位、不拟合模型。manifest 记录输入与输出 SHA256，并明确标记 `diagnostic_only: true`、实验未完成、不可作为阶段验收。它显示的部分时窗 RMS/峰值不能与完整 30 秒 PID 结果做正式优劣排名，图中最后一条命令也不代表失败求解曾经发出。

`solve_ms` 是优化器耗时，`compute_ms` 是单独记录的控制代码跨度；二者均不等于关节命令端到端传输延迟。SDK 的 `Float64` 速度消息没有发送时间字段，实际执行延迟仍需结合原始 rosbag/native 接收或作用记录判断。
