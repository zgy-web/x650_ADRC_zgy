# C++ 最新值指令流：完整 30 s 末端保持证据

本目录对应实际运行 `x650_cpp_live_v1_20261024_163442_43018`，启动记录为 `x650_cpp_launch_20261024_163355_43016`。完整保持窗口41.808–71.808 s已通过独立后验，真实起飞、保持、退出、降落、解除锁定及参数还原均已核对。这是该配置下的仿真验收，真机、普通八字和快速八字尚未验收。

| 证据 | 用途 |
| --- | --- |
| [report.json](report.json) | 完整独立验收报告；包括原始输入哈希、软件资格、源／worker／发送／消费／ULog与物理误差检查 |
| [archive_manifest.json](archive_manifest.json) | 本地报告和新旧审计代码原件的SHA256、字节数及路径映射 |
| [audit_x650_cpp_hold.py](audit_x650_cpp_hold.py) | 生成通过报告的审计代码原件 |
| [test_x650_cpp_hold_audit.py](test_x650_cpp_hold_audit.py)／[17项测试输出](audit_tests_17.txt) | 对数值、时序、实际发送屏障、独立恢复和严格等价值视图的正负回归 |
| [首次后验代码映射](audit_version_before_value_view_fix/mapping.json) | 首次失败报告对应的旧代码路径、SHA及保存位置，未把原失败报告覆写为成功 |
| [完整原始归档](hold_stream_v2_full.zip) | 本次运行、启动与后验的完整原件；以逐文件清单和回执核对内容 |
| [完整原件清单](hold_stream_v2_full_manifest.json)／[服务器归档回执](hold_stream_v2_full_receipt.json) | 完整归档的逐文件身份，以及服务器生成的包大小与SHA |
| [本地完整归档验证](local_full_archive_verification.json) | 下载后4731件原件的SHA／大小及内外清单一致性核验 |

完整 `report.json` 的 SHA256 为 `701a666b79c4f2bd4ca037ce0f2989f60fda907617d2724cfcf6024f1592a8b4`。服务器通过报告保存于 `artifacts/cpp_pipeline_20261024/hold_stream_v2_audit_v2/report.json`；首次失败保存在 `hold_stream_v2_audit/report.json`，旧审计脚本与测试另存于其 `audit_version_before_value_view_fix` 子目录。

完整归档含4731件原件，下载后已逐件核对SHA256和字节数，并核对包内外清单一致。ZIP大小49,019,171字节，SHA256为 `3a25ec4a9e0a76a24119386a0903ff1a1f6f80e920c86a3fb2ebbcd5fe244518`。

7501个原生物理末端样本覆盖完整30 s，同时刻三维 RMSE为 **2.765780 mm**，峰值为 **3.807266 mm**，最大倾角 **0.096728°**。完整 worker 回调最大12.081510 ms，实际发送间隔最大37.997054 ms，发送时原指令年龄最大41.313981 ms。验收保留原20 ms绝对周期、40 ms最大发送间隔、50 ms指令有效期与25 ms源状态年龄条件。

记录数需要区分：1499次求解中1498个候选进入 Gate，最后一个是测量结束时已完成但未进入 Gate 的尾部候选。1500次正常发送包含2次重复发送；加STOP零命令后总计1501次发送、1500条真实消费回执。每个关节均有1500条发送在物理更新边界采用，1条被后续同值发送提前覆盖。该覆盖项不计为执行，报告保留 `all_sends_applied=false`，最终零命令则要求两个关节都真实采用。

第一次审计失败是旧 ULog 匹配视图要求严格递增仿真时标，而两条相同候选的重复发送共享同一个已观测状态时标。修复仅对 ULog 值匹配折叠这一个严格等价重复项：1499行值视图与完整1500条实际发送分别保留；命令、反馈、时间戳和物理原件不变。不同候选、不同payload／feedback、非重复发送均不得折叠。通过报告中的值匹配标志不代表逐条PX4 ACK。

本轮相关Python回归合计66项通过，其中本目录保存17项专门审计回归。更多结构、历史失败及后续推进顺序见 [C++控制链说明](../../../X650_CPP_PIPELINE.md)。
