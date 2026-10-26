# X650 MPC：可信中心重建与重新线性化

本次更新针对已定位的局部冲突：旧计划附近的关节信赖域，与预测位置／速度／制动余量要求没有交集。保留归一化关节指令半径 **0.02**，在已识别的 QP 不可行错误发生后，重建中心，再建立局部预测与 QP。目标函数、执行器模型、权重、物理边界、原指令变化率和发布时限保持原值。

当前实现是上层协同速度 MPC。飞机三个速度输入和两关节速度输入共同优化，PX4 保留底层飞行控制。恢复分支没有把它变成力矩级全身 NMPC，也没有重新标定 TS20-50 的 24 V 带载模型。

## 控制流程

1. 从实际反馈、实际积分参考、已发送命令和 MIT 包历史构造本次输入。将上次已接受计划向前平移，按原逻辑求局部 QP。
2. 原 QP 正常可解时，沿用正常路径及原非线性候选检查，不触发恢复。
3. 仅对 `QP infeasible` 和 `constant QP infeasible` 尝试中心恢复。无效配置、非有限值、已越界状态、其他数值错误及超时继续拒绝。
4. 优先生成限变化率的关节制动计划。飞机部分从平移计划出发，仍受原速度和变化率限制；两关节从上一条实际命令出发，逐节点向零速度靠近。
5. 对这个内部中心生成真实参考和量化 MIT 包，使用同一 native TS20 模型传播。检查全程非线性包络、参考积分、节点制动余量，以及围绕新中心重新组装后的全部原约束。
6. 第一个中心未通过时，最多增加一次辅助 QP，向制动计划投影。辅助问题只移除旧中心的信赖域行，保留原物理、变化率、参考、制动和待执行指令约束。所得计划仍须重新传播、重新组装并通过第 5 步，才能成为可信中心。
7. 围绕通过检查的中心重新建立预测与原控制 QP；正式控制问题的关节信赖域仍是 0.02。最终解还须通过原非线性检查，并额外复核 native 节点制动余量。只有全部检查与时限通过后才返回指令并提交一次内部状态。

中心是求解器内部的计划，不会作为临时控制命令下发。若找不到可信中心，返回拒绝，保留原 `Memory`；不会复用旧计划来伪装本次优化成功。

## 0.02 的含义

设第 (i) 个节点的归一化输入为 (z_i)，关节速度上限为 (ar v_j)，中心为 (c_i)。正式 QP 的关节行始终是

\[
|z_{i,3+j}-c_{i,3+j}|\leq0.02,\qquad
u_{i,3+j}=\bar v_j z_{i,3+j}.
\]

当前两关节上限分别为 0.45、0.35 rad/s，因此相对本次中心的局部速度变化仍为 ±0.009、±0.007 rad/s。0.02 不直接表示关节角变化、飞机位移或误差阈值。它也不限制新中心只能在旧中心的 0.02 内；新中心的可信性由重新传播和原约束检查建立。

令 (h_i) 为实际首步／后续节点时长，制动关节中心按原变化率上限生成：

\[
c_{i,j}=\operatorname{clip}\left(0,
\max(-\bar v_j,c_{i-1,j}-\bar a_jh_i),
\min(\bar v_j,c_{i-1,j}+\bar a_jh_i)\right),
\]

其中首节点的前值是上一条实际采用的控制命令。没有用测量角速度替换上一条命令，也没有把积分参考重置为测量关节角。

恢复中心及恢复后的最终候选，对 native 预测节点另核查原制动余量形式：

\[
q_j^{\min}\leq q_{i,j}+T_j\dot q_{i,j}\leq q_j^{\max},\quad
T_j=\frac{\bar v_j}{2\bar a_j}+\Delta t_{\max}+0.05+\text{joint age}.
\]

这沿用当前模型中的制动余量要求，不是对任意外扰的可达性证明。连续位置／速度包络由原 native 传播检查；上式在预测节点核查。

## 计算预算与接口

- 每次 `Core::step` 最多三个正式控制 QP，包含首次无解的尝试。
- 最多检查两个重建中心；辅助投影 QP 最多一次，并与正式求解共同消耗原 **50 ms** 总时限。
- 昂贵重试前检查剩余时间，最终发布前再次检查时限。该检查不能抢占正在执行的单个预测或 QP，不构成硬实时最坏时间保证。
- 失败不提交计划、命令历史、时间或步数；成功只提交一次。ROS wire 消息仍是原接口，没有增加每周期执行器往返等待。
- `Diagnostics` 增加了恢复证据，原生 Core ABI 更新为 **2**。Core 和 `isolated_worker` 必须使用匹配头文件重新构建；不要把 ABI 2 库替换到 ABI 1 工作进程下面。原冻结库保留用于数值对照。
- Full 模式记录原 QP、替换前的 `linearized_states`、native 状态和真实包；Compact 模式保留实际候选及恢复中心证据，完整 JSON 写入仍在诊断线程完成，不为记录再跑一次预测。

## 本轮服务器验证

全部结果来自新容器 `autodl-container-3187449845-10f5a45f`。原始失败运行和原二进制未覆盖，原 QP 库、TS20 预测库的 SHA 保持不变。

| 项目 | 结果 |
|---|---|
| 原核从原 LiveStart 重建 1～379 次调用 | 前 378 次完整输出匹配；379 再现 `QP infeasible`，内存不变 |
| 旧／新核逐条固定输入对照 | 378 次正常输出差为 0，提交后完整内存相同 |
| 原 7.544 s 失败快照 | 新核通过，两个正式 QP，一个可信制动中心，无辅助投影 |
| 原约束、局部半径、非线性检查 | 独立审计通过，原物理边界与 0.02 未放宽 |
| 合成边界反例 | 仍拒绝，完整内存回滚；该反例在 QP 前被拒绝，不冒充恢复搜索失败的覆盖 |
| 实际 ROS 2 ABI 2／Compact 工作进程 | 私有 DDS 域回放全部 379 条原输入通过；最后一次 Core 21.408 ms，完整回调 21.505 ms |
| Python 反例／C++ ROS／native 基准 CTest | 24／6／4 项通过 |

这批输入固定在原飞行已经记录的状态上，只刷新 ROS 回放所需的单调时钟签发时间。后续状态不会随新输出重新演化，因此 **这是软件回放验收，不是新的受扰闭环飞行验收**。原快照约 19～20 ms 的 Core 时间、ROS 恢复回调时间都是本机观测值，不代表 Orin 上的时限保证。

数值、原始回归、构建命令、所有失败尝试和库身份见 [交付索引](validation/cpp_recovery_20261026/README.md)。原有保持／普通／快速八字证书仍针对原版本，不能自动转移给新后端。下一阶段需要绑定 ABI 2 软件证据，重新运行实际仿真反馈和执行链上的完整受扰工况，检查全程消费、退出、恢复误差及参数还原；不注入额外延时。

## 复现入口

先在已有 ROS 2／PX4 消息环境运行原始 CDR 导出。输出必须是新目录：

```bash
python3 scripts/export_x650_recovery_recording.py \
  --run /absolute/original_run \
  --output /absolute/new_original_export
```

`scripts/x650_cpp_recovery_sequence.cpp` 用匹配的旧 ABI 1 头文件和冻结库编译，复用生产 `Contract` 和轨迹代码重建全部内存。CLI 为：

```text
old_sequence predictor.so predictor_sha256 recording.json recording_sha256 NEW_OUTPUT_DIR
```

`scripts/x650_cpp_recovery_replay.cpp` 分别针对旧／新头文件和库编译。CLI 为：

```text
old_or_new_replay predictor.so predictor_sha256 input.json input_sha256 NEW_OUTPUT_JSON
```

每个库的实际加载路径与 SHA 写入 raw 输出，独立审计拒绝串库。两种回放器的具体服务器编译命令在归档 `compile_v1.py`、`compile_and_replay_v3.py` 中；新 Core 链接原 QP 库，不覆盖它。将下列路径和哈希替换为新输出的真实身份后执行批量对照：

```bash
python3 scripts/run_x650_recovery_regression.py \
  --old-executable /absolute/old_replay \
  --new-executable /absolute/new_replay \
  --predictor /absolute/libts20_prediction_v3.so \
  --original-report /absolute/original_reconstruction/report.json \
  --original-report-sha256 VERIFIED_REPORT_SHA256 \
  --new-core-sha256 VERIFIED_NEW_CORE_SHA256 \
  --output /absolute/NEW_REGRESSION_DIRECTORY

python3 -m unittest discover -s tests/unit -p 'test_x650_recovery_*.py' -v
```

原输入、过程退出码、两份 raw、独立 audit 和最终汇总全部新建保存。任何缺件、超时、来源改变、正常输出变化或约束检查失败都会使汇总失败。`probe_x650_recovery_worker.py` 进一步复用这些带哈希的输入与新启动清单测试实际工作进程；该入口不会创建 ingress、dispatcher、Gazebo 或执行器发布者。
