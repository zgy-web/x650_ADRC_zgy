# 普通八字：实际反馈与独立执行架构

本入口只接受普通八字。冻结已验收的 C++ 数值后端、模型、PID/MPC 参数、关节限制、运行时期限及进程资源分配，只替换任务参考轨迹。快速八字没有本入口。

轨迹沿世界 X/Z 平面，幅值分别为 ±0.25 m、±0.10 m，12 秒/圈、两圈；采用原 C2 相位过渡，2 秒初始保持、26 秒轨迹段、4 秒回零保持，共 32 秒。中心由本次实际起飞、预流后的 LiveStart 确定。控制器前瞻和状态消息目标使用同一个解析轨迹；不对结果拟合平移或相位。

启动前需要新的 `x650_isolated_live_stage_certificate_v1` 完整末端保持报告。入口核对调用者提供的报告 SHA256、全部原始证据、30 秒窗口、相同数值后端、执行协议、关键源码与执行二进制。旧的单进程保持报告和部分成功片段不能替代此门槛。新参考入口及两处显式分派钩子由本次源码归档记录；算法和执行通道保持原版本。

在服务器已加载 ROS 2 工作区环境后，先生成任务（`HOLD_REPORT` 和 `HOLD_SHA` 必须来自独立审计结果）：

下面的 `NEW_BACKEND_HOLD_TASK` 指**当前执行协议绑定的新后端保持任务绝对路径**；不能继续沿用旧后端任务的文件名或哈希。先取得该新任务的完整保持通过证书，再构造普通任务。

```python
import json
from pathlib import Path
from simulations.x650_isolated_figure8_v1 import task
baseline = Path(NEW_BACKEND_HOLD_TASK).resolve(strict=True)
cfg = task.build_task_config(
    {'path': str(baseline), 'sha256': task.baseline_sha256()},
    {'path': HOLD_REPORT, 'sha256': HOLD_SHA})
Path(NORMAL_TASK).write_text(json.dumps(cfg, indent=2) + '\n')
```

```bash
python3 simulations/x650_isolated_figure8_v1/launch.py --task "$NORMAL_TASK" --transport-proof "$TRANSPORT_PROOF" --dry-run
python3 simulations/x650_isolated_figure8_v1/launch.py --task "$NORMAL_TASK" --transport-proof "$TRANSPORT_PROOF"
python3 scripts/audit_x650_isolated_figure8.py --launch-dir "$LAUNCH_DIR" --run-dir "$RUN_DIR" --output "$AUDIT_DIR"
```

也可以使用可复用顺序驱动。它要求显式固定任务、保持证书、运输证明和执行协议的 SHA256；支持协议修订号 ≥9，后续修订仍必须通过相同的证据绑定及阶段门槛。默认只构造普通任务并执行 launch `--dry-run`，不启动飞行：

```bash
python3 scripts/run_x650_isolated_normal_sequence.py \
  --repo "$REPO" \
  --hold-task "$NEW_BACKEND_HOLD_TASK" --hold-task-sha256 "$HOLD_TASK_SHA" \
  --hold-report "$NEW_HOLD_REPORT" --hold-report-sha256 "$HOLD_REPORT_SHA" \
  --transport-proof "$TRANSPORT_PROOF" --transport-proof-sha256 "$TRANSPORT_SHA" \
  --protocol-sha256 "$EXECUTION_PROTOCOL_SHA" \
  --output "$REPO/artifacts/NEW_NORMAL_SEQUENCE_DIRECTORY"
```

明确追加 `--execute` 后，驱动仅执行一次普通八字、独立验收和验收通过后的图表生成。输出目录必须是仓库 `artifacts` 下的新目录；准备检查与正式执行分别使用新目录。驱动只使用本次日志的唯一 launch 标记和 launch 文件绑定的唯一 run，保留各步 PID、命令、日志和哈希；失败不自动重试、不启动快速。中断不会主动杀掉仿真恢复流程，后续启动前应检查记录的进程和恢复状态。

当前新增顺序驱动只完成本地程序与模拟执行测试，**尚未部署或启动新后端的服务器普通八字**。本说明不把已写好的驱动、新任务文件或通过的 dry-run 计作物理实验通过；最终状态以本次服务器完整审计为准。

实际运行沿用独立状态接收、C++ 定周期下发、实际关节消费确认和独立诊断记录。每次下发之前必须确认上一条两个关节都已消费；退出时零速消费确认和唯一写者释放完成后才返回原恢复流程。

验收覆盖完整 32 秒，单独补充 2–28 秒运动段指标，同时核对全部原始指令消费、PX4 ULog 编码、发送与接管时序、记录完整性、原生物理末端误差和每次输入的独立数值复算。一次未完成的运行保留原始证据，不能生成通过证书，也不能因此推进快速八字。
