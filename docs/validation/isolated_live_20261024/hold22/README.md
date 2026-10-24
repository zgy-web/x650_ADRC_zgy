# hold22：上下文准入失败与恢复的小型证据

运行：`x650_isolated_live_v1_20261024_120535_11859`；启动：`x650_isolated_launch_20261024_120448_11858`。**本轮未完成30 s末端保持，不授予普通或快速八字准入。** 本目录保留小型原始文件及派生分析，不包含新的仿真结果或大型原始日志包。

18个worker候选全部通过数值检查；第19次在进入worker之前被gate以`CONTEXT/reason15`拒绝。独立故障监督在producer发出拒绝后 **1.262165 ms** 于回调中锁存错误。这是producer到监督回调的时间，区别于hold21的记录进程sink接收时间。

| 本轮实际测量 | 中位数 | P95 | 最大值 |
|---|---:|---:|---:|
| worker回调入口→engine返回，墙钟 | 12.883 ms | 15.889 ms | 17.675 ms |
| 整个worker回调，线程CPU | 13.418 ms | 16.451 ms | 18.241 ms |
| snapshot发出→worker回调 | 3.667 ms | 6.597 ms | 6.849 ms |

这些只有18次实际样本，不能替代持续时序验收。dispatcher随后报告`GATE_REJECTED/reason4`，指令年龄46.058 ms，本次不是50 ms寿命超时。资源监测覆盖完整378.742 ms活动窗口，包含窗口内7个样本、两侧包围样本共9个；包围区间内cgroup新增节流次数和节流时间均为零。约50 ms的频率快照不能确定每次回调的实际CPU／频率。

## 上下文重建与限制

[context_analysis.json](context_analysis.json)由[audit_hold22_context.py](audit_hold22_context.py)读取原始记录并使用未改变的因果关节provider重建。被接受的第18次上下文与其原始CDR的全部已比较字段精确一致。对source77／第19次准入的重建发现：末个packet相对时间比任务相对时间大`7.105427357601002e-15 s`，重建检查中的唯一严格谓词违反是`packet_future`。已有Python/provider历史比较使用`1e-10 s`数值容差，故与原C++严格`<=task_time`存在边界差异。

**被拒绝的第19次完整snapshot CDR没有保留。** 重建读取完整的事后CSV，不能恢复原始逐步读取游标；原始拒绝消息的padding也无法直接核对。因此该结果标记为重建，不表述为恢复出的原始拒绝CDR，也不把独立报告中16 ms的时钟观察差直接认作拒绝谓词。`manifest.json`还区分了与独立复核交叉匹配的5项输入哈希和仅由重建报告记录的3项输入哈希。

## 实际仿真恢复已核验

19次发送＝18次正常指令＋1次stopzero，与19次receipt逐一对应；两关节共38次命令的原始计数1183～1201、速度和消费时间均与TS20物理插件记录一致。gate拒绝后未再发正常指令；stopzero在拒绝后19.818 ms发出、24.194 ms获得实际消费确认。3290条旧执行端发布记录无接管区间重叠。

原始PX4 ULog全部已记录`vehicle_status.failsafe=0`：42.492 s进入AUTO_LAND，早于43.032 s的offboard信号丢失；46.820 s报告landed，48.828 s报告未武装。这些是ULog原始微秒时间直接换算为秒，没有拟合host时间。56个参数的类型及raw值恢复一致，同步保存返回0，六个独占进程正常关闭。上述证明本次失败后的仿真恢复完成，不证明完整末端保持或真机故障工况已合格。

## 文件阅读与复核

- [review_summary.json](review_summary.json)：独立性能、监督、消费、资源覆盖及恢复汇总，明确`full_hold_passed=false`。
- [context_analysis.json](context_analysis.json)：单独的上下文重建结果与输入哈希。
- [physical_consumption_review.json](physical_consumption_review.json)、[remote_cdr_ulog_review.json](remote_cdr_ulog_review.json)：对服务器原始TS20消费、CDR及ULog的只读派生复核；较大的原始日志仍留在报告所列路径，并保存精确输入哈希。
- `controller_metadata.json`、`flight_samples.csv`、`isolated_live/`、`launch/launch.json`及参数恢复文件：原始服务器字节。
- `hold_v22_resources.jsonl`：取回时的原始监测文件快照；不宣称监测过程已结束或快照包含其全部后续数据。
- [manifest.json](manifest.json)：每份交付的bytes、SHA-256和`original/derived/script`等角色；`retrieval_manifest.json`与`review_delivery_manifest.json`原样保留。

可在已有Python＋NumPy环境运行`summarize_review.py`重算小型汇总。ROS CDR和ULog提取脚本需要原运行环境及对应原始日志，不会启动控制节点。原始文件和重建结果没有为获得通过结论而改写。
