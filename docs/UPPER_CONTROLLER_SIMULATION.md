# X650 上层 MPC／NMPC 同条件仿真入口

这组入口将保留的可调用 MPC 和新上层 NMPC 接到同一套实际 PX4／Gazebo 反馈与执行链。两者输出均为机体三轴速度和两个关节速度；PX4 内环、TS20 执行模型和真实消费记录保持共用。它不是推力／力矩接口的全身 NMPC，也不构成真机验收。

## 算法与执行分工

```text
PX4 估计状态 + Gazebo 实际关节状态／已发包历史
                         │
                  C++ 状态接收与准入
                         │ 25 Hz 最小准入间隔
             ┌───────────┴───────────┐
             │ callable MPC / NMPC   │
             │ 私有候选 + 私有 Memory │
             └───────────┬───────────┘
                         │ 候选和原始 issue 时间
                新鲜度／数值／约束检查
                         │
                  C++ 50 Hz 下发器
                    ┌────┴────┐
                 PX4 PVA    关节速度执行端
                    └────┬────┘
               ActuatorSend 实际发送通知
                         │
                   提交一次 Memory
```

关节是否消费由独立异步进度账本监督。上层不等待每条命令的完整消费往返。候选发布到 ROS 话题仍不等于执行端已经发送：新 worker 保留候选状态，只有对应 `ActuatorSend` 的会话、编号、速度、PVA 编码和时间均匹配时才提交。下一快照与发送通知回调乱序时最多缓冲一条快照。正常退出时未发送的候选被丢弃。

Python 负责实验启动、接管、监督、退出和离线分析；状态接收、在线求解与检查、周期下发均在 C++。完整数值诊断由有界队列交给辅助线程写盘。

## 本轮 NMPC 的准确范围

NMPC 使用同一预测模型，对非线性关节响应、姿态影响和末端误差进行完整前推与六项实际代价评估，在原始请求上最多提出四个重新线性化的候选。只有通过硬约束、节点间包络、首令保持、实际停止余量检查且代价下降的候选才替换已选结果。

后续提案失败可以保留此前的安全候选，但仍须满足从**原始请求 issue** 起算的 50 ms 期限。不能更新起算时间来掩盖排队或计算时间。40 ms 是开始新提案的软预算；C++ 数值调用不可抢占，超过硬期限返回的结果会被拒绝。

该实现是有界、顺序重新线性化的上层非线性优化原型。接受一个安全候选不代表已证明一般 NLP 的局部／全局收敛。线消息保留的 `accepted_optimum` 字段只作为既有执行契约的候选通过标志；真实非线性代价、选中迭代及拒绝原因单独保存。

## 相同条件与阶段顺序

| 项目 | 两后端共用设置 |
|---|---|
| 平台 | 导入 X650 机架＋原二连杆机械臂 |
| 执行接口 | 飞机三轴速度＋两个关节速度 |
| 上层准入／下发 | 25 Hz／50 Hz |
| 最大实际控制步长／候选保持 | 100 ms／160 ms |
| 预测节点 | 15 个，后续节点间隔 40 ms |
| 模型、PID、代价权重、硬界 | 从现有 multirate 静止保持任务逐项派生与校验 |
| 记录模式 | 两者均 Full，辅助线程持久化 |
| 完整保持 | 30 s |
| 普通八字 | X ±0.25 m，Z ±0.10 m，12 s／圈，2 圈，总窗 32 s |
| 快速八字 | 同幅值，8 s／圈，2 圈，总窗 24 s |
| 参考对齐 | 原始时刻、同一世界坐标系；不拟合平移或相位 |

每个后端分别完成新的保持→普通→快速验收链。历史八字采用不同的时序配置，只能作为背景记录。新对照必须读取本组任务与配置身份，不能直接搬用历史分数。

软件验收包含原 C++ 数值测试、新在线准入测试、实际 ROS worker 的健康与拒绝测试。私有 DDS 域的软件测试会显式注入合成发送通知，验证提交事务；该通知不是实际执行或实际消费证据。正式物理实验只接受实际下发器的通知。

## 可复现命令

服务器使用既有依赖；先编译 `x650_nmpc_controller_v1`，再在独立 overlay 编译 `flyarm_nmpc_cpp`。当前实验环境入口为：

```bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
source artifacts/nmpc_live_20261027/environment.sh
```

准备新保持输入并执行软件测试：

```bash
python3 scripts/prepare_x650_upper_experiment.py \
  --stage artifacts/nmpc_live_20261027 \
  --backend nmpc --case hold --input-name inputs_v1
```

MPC 对照选择 `--backend mpc_callable`。普通和快速任务另需 `--prior` 指向同后端前一阶段的新物理审计报告；`--worker-probe` 可绑定同后端已完成的保持软件测试报告。所有输入目录必须是新的，避免覆盖失败证据。

从准备目录启动时，显式传入 `task.json`、`startup_manifest.json` 及其 SHA256、`protocol.json`、`options.json` 和 `qualification_manifest.json`：

```bash
python3 simulations/x650_upper_nmpc_v1/launch.py \
  --task "$INPUT/task.json" \
  --startup-manifest "$INPUT/startup_manifest.json" \
  --startup-sha256 "$(sha256sum "$INPUT/startup_manifest.json" | cut -d' ' -f1)" \
  --protocol "$INPUT/protocol.json" \
  --options "$INPUT/options.json" \
  --qualification-manifest "$INPUT/qualification_manifest.json"
```

追加 `--dry-run` 只校验准入。正式启动使用平台原有飞行互斥锁，保留完整接管、恢复和参数恢复事务。

## 结果证据

每次运行独立保存源码／二进制身份、原始 CDR、数值候选与迭代、原始 issue 时效、实际 PVA／关节发送、关节消费、PX4 ULog、物理轨迹和退出记录。

独立审计将执行完整性与末端性能分开报告。只有完整窗口、连续下发、实际消费、正确接管退出、参数恢复与原性能界限同时通过，才生成可供下一阶段使用的报告。软件测试通过、任务进程返回零、单个候选通过，都不能替代完整物理验收。

2026-10-27 已完成两种后端各自的新保持、普通八字、快速八字，共六组通过的实际仿真。结果、同条件校验、时间统计、可重画的末端数据与范围说明见[六组正式验收](validation/nmpc_live_20261027/README.md)。这组单次配对未显示 NMPC 一致的跟踪精度优势，MPC 接口继续保留。
