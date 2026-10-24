# hold21：未完成的末端保持与故障恢复证据

本次是 revision 14、冻结 task v8 和同一 C++ 数值后端的**失败尝试**。运行目录为 `x650_isolated_live_v1_20261024_114955_9089`，启动目录为 `x650_isolated_launch_20261024_114909_9088`。任务要求保持 30 s，最后一轮候选的任务时间仅为 1.116 s，因此不授予完整末端保持通过结论，也不作为普通／快速八字的准入依据。场景返回码为 1，启动包装器返回码为 2。

## 两条独立问题链

1. **有效指令未在寿命内更新。** C++ 在 DispatchTick3031、单调时钟 `59599541425042337 ns` 检测到指令54的年龄为 63.318013 ms，超过原有50 ms寿命，锁存 `STALE_COMMAND`。55次候选计算的数值检查全部通过，但只有前54个候选得到 gate 接受并实际发送；候选55计算完成时已经晚于首fault 3.851055 ms，ingress没有转发它。数值计算通过与执行及时性通过是两种不同条件。

2. **Python 监督回调未及时处理已有fault。** 首fault的完整原始CDR直到 `59599542583158791 ns` 才进入记录进程，即 fault 后 **1158.116454 ms**。记录中的 `received_ns` 在记录进程 `recvmsg` 和反序列化之后采样，**不是DDS网络到达时间，也不是Python监督回调入口时间**。本轮没有记录精确回调入口，不能把该差值解释为DDS网络延迟。

实际运行的阶段数据如下；原件见 ZIP 中 `isolated_live/worker_timing.json`、`ingress_timing.json` 和 `recording/diagnostics.jsonl`。

| 测量区间 | 候选54 | 候选55 |
|---|---:|---:|
| snapshot发出→worker回调入口 | 4.106 ms | 3.846 ms |
| worker回调入口→engine返回，墙钟 | 26.923 ms | 23.137 ms |
| 整个worker回调，墙钟 | 27.810 ms | 24.026 ms |
| 整个worker回调，线程CPU | 27.801 ms | 24.017 ms |
| 内层allocator测量墙钟 | 20.000 ms | 16.140 ms |
| QP测量墙钟 | 3.751 ms | 0.589 ms |
| proposal发布返回→ingress回调入口 | 0.030 ms | 0.027 ms |

26.923／23.137 ms是包含接口封装的engine边界墙钟，不能直接称作同一边界的CPU时间。整个回调的CPU与墙钟接近，支持本次主要消耗在实际计算／处理工作上；本表不进一步归因于某个未单独测量的函数。ACTIVE期间只有4次gen0 GC，最大95.608 µs，没有gen1／gen2 GC。离线重放或带插桩性能结果不能替代本次实际运行时间。

## 观察回调饥饿的证据

完整source记录共3800条，发送与接收计数一致、无丢包、无序号缺口。evidence2974～3519是**连续546条feedback，四个ROS观察主题零回调记录**；最后一条feedback为sim43.100。恢复时才开始补收旧DispatchTick2974及对应send／receipt／event。669次ACTIVE pump的`extra_callbacks`均为1。

启动源码ZIP逐项与原source manifest核验，共175个文件。该版本的`VehicleAPI._pump`调用`spin_once(timeout_sec=0.02)`，ACTIVE额外drain调用`spin_once(timeout_sec=0)`且预算250 µs。归档的安装版Humble `rclpy/executors.py`明确在kwargs改变时重建ready-callback迭代器。两种timeout交替、每轮额外只执行一次回调、较晚创建的四个观察订阅未被处理，与完整记录的连续空白相互支持。这个机制比单纯“depth4000导致延迟”更具体。

修复应消除同一executor不断切换参数的行为，并为首fault配置独立、优先、可锁存的最新值监督通道。完整深队列CDR仍需保留，用于审计，不应直接改成depth1而丢掉中间证据。新实现需要在高频前置订阅下验证故障通知上界；本档案不宣称修复已经通过物理实验。

## 故障后实际恢复的独立核对

全部56次send和56次receipt的会话、epoch、指令与dispatch序号逐一匹配。55次normal send包含1次重复发送，另有1次最终stopzero；两关节共112次命令，原始消费计数均从1181连续到1236，速度和消费时间与`ts20_state.csv`一致。正常输出均发生在首fault之前，后续正常输出为零。stopzero在fault后0.146566 ms发出，7.146270 ms获得实际消费确认。3266条旧执行端发布记录与独占接管区间没有重叠。

PX4原始ULog独立记录：offboard信号丢失为43.040 s，failsafe为43.052 s；降落命令之后47.576 s报告landed，49.592 s报告`arming_state=1`，末条状态仍未武装且已落地。这些是**ULog原始微秒时间除以1e6**，没有拟合转换到host单调时间。末条flight sample同样为`armed=0, landed=1`。56个参数的原始类型及raw值恢复一致，同步保存返回0；六个独占进程均正常关闭，launch记录`runtime_closed=true`。

这证明本次故障后的零速消费和恢复链完成；不等于所有pending-zero故障排列或真机失联场景均已合格，也不等于完成了全窗口PVA／轨迹性能验收。

## 文件与复核范围

- `original_sources.zip`：服务器原始字节，包含原始观察CDR、worker记录、ingress／worker阶段、关节消费、ULog、参数恢复、启动源码等51个顶层条目；嵌套`launch/sources.zip`保留175个启动源码。
- `manifest.json`：每个原始条目的服务器路径、bytes与SHA-256，以及本目录交付物的bytes与SHA-256。
- `failure_report.json`：独立派生分析，明确`passed=false`。
- `cdr_review.json`：实际ROS反序列化后与全部materialized字段逐项比较，dispatch3088、send56、receipt56、event54均一致；55个候选全部通过其数值状态检查。
- `ulog_recovery_review.json`：原ULog恢复状态提取，保留未拟合的原时间。
- `review_cdr.py`、`review_ulog.py`、`review_archive.py`：只读证据复核程序；前两个需要已有ROS／pyulog环境，最后一个使用本目录档案重算失败报告，不运行仿真。

原件不包含Remote登录日志、密码、私钥或认证配置。原始数据与派生分析分开标识；源记录、门槛和manifest均未因失败而改写为通过。
