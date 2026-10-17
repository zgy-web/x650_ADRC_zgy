# PID + 前馈 + 末端协同闭环：接入与验证

本次新增 `flyarm-pid-ff 1.0.0` 控制包与 `pid_ff_wholebody_v1` 场景。运行方法、接口和坐标系见[控制包 README](../control_packages/pid_ff_wholebody_v1/README.md)。

## 控制关系

```text
末端目标位置 / 速度 / 加速度
                  │
实际机体与关节状态 ─┴─→ 末端位置误差 + 带约束的协同分配
                              ├─→ 飞机位置 / 速度参考
                              │      + 轨迹加速度与臂运动惯性前馈
                              │         → PX4 固定参数 PID → 飞机
                              └─→ 两关节速度指令 → Gazebo 关节执行器
```

`ON` 为末端协同反馈开启，`OFF` 保留同样的 PID 和前馈、关闭该协同反馈。默认名义轨迹贡献为飞机 80% / 机械臂 20%；纠偏通过实际状态、代价权重和约束在线求解，不固定按这个比例分配。

适用模型固定为 `x500_underfold` 两自由度机械臂。当前实现控制末端位置，使用仿真真值辅助机体反馈；两关节输出速度，反作用力矩只用于诊断。它不等同于六维末端位姿控制、关节力矩前馈或全动力学 MPC。

## 2026-10-17 已完成的验证

| 验证项目 | 结果 | 原记录 |
|---|---|---|
| API 与仿真适配测试 | 15/15 通过，无跳过 | [本地测试日志](validation/pid_ff_wholebody_v1_20261017/local_tests.log) |
| 合成输入与原实现比较 | 两种模式共 640 步，容差 5e-12 | 同上 |
| 历史真实控制日志重放 | 8 次实验、13,443 步；15 类输出最大绝对差为 0 | [重放报告](validation/pid_ff_wholebody_v1_20261017/historical_replay.json) |
| 新场景实际悬停 | PX4 SITL / Gazebo 中成功完成 | [运行摘要](validation/pid_ff_wholebody_v1_20261017/integration_summary.json)、[飞行日志](validation/pid_ff_wholebody_v1_20261017/flight.log) |
| PID 与源码核验 | 26 项 PID 读回确认，运行记录含模型/源码哈希 | [控制器元数据](validation/pid_ff_wholebody_v1_20261017/controller_metadata.json)、[部署清单](validation/pid_ff_wholebody_v1_20261017/deployment.json) |
| 关闭与数据归档 | 已降落上锁、关闭仿真，PX4 ULog 已归档 | [运行结果](validation/pid_ff_wholebody_v1_20261017/result.json) |

历史重放选取平移受扰、姿态受扰、普通八字、快速八字的第 1 次重复，各包含 OFF/ON。重放从原预发送阶段开始保留滤波历史，比较的是控制命令，不是重新执行八次物理仿真。

新场景本次实际运行的是 **8 秒无外加扰动悬停**：共有 751 行记录，其中任务段 414 行，控制间隔 20–24 ms，末尾包含约 0.3 s 收尾。它确认封装后调用链与起降流程可运行，不替代受扰恢复或快速八字的配对性能实验。

## Git 版本的可复现性

- 仿真控制代码和固定参数与上述服务器验证版本一致。README、测试参考加载方式及仓库目录配置在整理提交时补齐。
- `tests/fixtures/legacy_v1` 保存旧数值实现及 PID 的冻结副本，按原 SHA-256 校验；新克隆也能运行完整 15 项测试，不依赖本地未提交的研究目录。
- `benchmark/whole_body_figure8_v1/.gitkeep` 保留既有共享飞行锁目录，避免新克隆启动时缺少目录；运行产生的锁文件已忽略。
- `.gitattributes` 保证被冻结且进行哈希校验的源文件与参数在 Windows/Linux 上保持相同字节。
- 提交前的仓库测试日志见 [Git 工作区验证](validation/pid_ff_wholebody_v1_20261017/git_checkout_tests.log)。这项测试无需 ROS 或飞行仿真。

大型原始 ROS bag、ULog、控制轨迹、视频及构建产物不随源码提交。原运行路径、输入 SHA-256 和运行状态保留在上述记录内。历史 `deployment.json` 描述当时的服务器部署，不代表文档整理后的整个 Git 树。

## 历史日志重放

需 NumPy 和 PyYAML；每个原始运行目录必须有 `metadata.yaml`、完整 `control_trace.csv`（含预发送行）和 `model_snapshot.sdf`。从仓库根目录执行：

```bash
python control_packages/pid_ff_wholebody_v1/tests/replay_control_trace.py \
  --case /absolute/path/to/original/run \
  --output /absolute/path/to/new/replay_report.json
```

可重复传入 `--case` 比较多次实验。不要把裁剪后的展示数据或视频轨迹代替完整控制日志。
