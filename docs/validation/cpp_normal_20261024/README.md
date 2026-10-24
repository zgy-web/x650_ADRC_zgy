# X650＋二连杆机械臂：完整 C++ 普通八字验证

本次普通八字在服务器 PX4／Gazebo 中完成，并通过完整独立后验及第二次只读复核。机架为用户导入的 X650 URDF，机械臂沿用原二连杆和 TS20-50 条件执行器模型，定位为现有精密 EV 模式。控制仍为三个机体平移通道、两个关节通道的上层协同 MPC，底层由 PX4 与舵机执行。

普通参考在世界 XZ 平面：X±0.25 m、Z±0.10 m，12 s/圈、2圈。原 C2 相位律包含2 s起始保持、首末各2 s平滑加减速及4 s末尾保持，总窗口32 s，运动区间2–28 s。实际 LiveStart 确定中心和原点；没有用实际轨迹拟合位置、相位或时间。

保持阶段已经接受的 Core、QP、TS20 预测库、物理配置、MPC 权重和限制沿用。新工作进程把当前目标及每个预测节点的 p/v/a 接入同一 C++ 计算链，预览时刻为 `t + cumulative([actual_dt, node_dt, ...])`。正常输出为定周期最新有效值，关节采用进展异步监督；停止零指令仍需真实采用确认。

## 本次结果

| 窗口 | 原生样本 | 三维 RMSE | 三维峰值误差 |
| --- | ---: | ---: | ---: |
| 完整0–32 s | 8,001 | **4.562884 mm** | **7.790075 mm** |
| 运动2–28 s | 6,501 | **4.908519 mm** | **7.790075 mm** |
| 末尾保持28–32 s | 1,001 | **1.614122 mm** | **3.107416 mm** |

指标来自原生物理末端的同一时刻误差，完整记录无丢失。机体倾角峰值0.622335°。本次Core计算中位数6.958085 ms、最大20.334062 ms；完整worker回调最大20.536500 ms。实际正常发送间隔中位数19.997912 ms、最大24.109010 ms，发送时原指令年龄最大41.876339 ms。原20 ms输出周期、40 ms最大发送间隔、50 ms指令有效期和25 ms源状态年龄条件均保留。

1,598次求解全部接受，其中正常实际执行1,597个唯一候选。候选188、777各重复发送一次，因此正常发送1,599次；窗口结束后，最后的候选1598被停止零命令取代。加上STOP共1,600次发送、1,600条真实消费回执。两个关节各有1,600次实际采用记录，1,598次位于相同物理步边界；没有声称每次双关节都严格同时采用。

机体X/Z实际峰峰位移为0.513307/0.206073 m；两个关节的角度峰峰值为14.692583°/15.562722°，确认机体和机械臂均参与实际运动。这些幅度不是控制贡献分配比例。原ULog独立确认活动与恢复阶段failsafe均为0，最终已降落、解除锁定、还原参数，6个子进程全部退出码0。

## 图、视频与原件

- [完整32 s视频](X650_normal_CPP_MPC.mp4)：世界固定视角＋机械臂细节＋同步末端误差曲线。黄色为参考，蓝色为实际末端经过的路径。同一组MPC的两个视角。
- [媒体原件包](normal_media_v2.zip)、[媒体逐项验证](media_local_verification.json)、[完整解码记录](media_v2/full_decode.json)。最终视频为1600×1140、24 fps、768帧，完整解码32 s；视频SHA256 `0b2c79473ca045716db9add48fadb1ddfa21012564b76c7a676c5331c1f26059`。
- [预览](preview.png)、[轨迹与误差图](endpoint_validation.png)、[矢量PDF](endpoint_validation.pdf)、[逐样本图表数据](endpoint_chart_data.csv)。
- [完整物理验收报告](report.json)、[图表记录](figure_manifest.json)。报告SHA256：`a8588defbd3ce237a04e1403d7f895aecde4008fa4ee4ca78c591ddb711be474`。
- [独立复核摘要](independent_review_v1/review.json)及[运动范围与候选计数](independent_review_v1/ranges_and_candidate_counts.json)。复核再次读取原始CDR、原生轨迹及ULog，以独立相位表达式重算三段误差。
- [完整原始归档](normal_full_raw_v1.zip)、[逐原件清单](normal_full_raw_v1_manifest.json)、[服务器归档回执](normal_full_raw_v1_receipt.json)、[本地逐项验证](local_archive_verification.json)。共8,616件原件、324,723,066字节未压缩内容；ZIP为68,629,383字节，SHA256 `a15ab0c3bbbb017c9d60da5f8c7eb6dde5bfd72d162780dc6c8f7e03df68ccaf`。

视频根据本次同一物理步记录的机体位姿及关节角度回放，不重新积分动力学；误差数值来自原始物理样本。原URDF全部281个visual mesh保留，未记录的旋翼转角保持原零位。离线渲染质量与输入身份记录在交付的媒体清单中。图像不承担物理验收作用。

容器实际使用Mesa软件绘制。最终显示采用显式`software_fast_v1`选项，关闭阴影、反射及MSAA以加快离线渲染；全部370,110个三角形、记录位姿、轨迹及误差不变。原渲染默认行为保持；旧未完成v1没有作为最终视频交付。媒体ZIP另保存35件原件及新旧源码映射，逐件本地复验通过；体积8,013,330字节，SHA256 `ebaa4331f50851fe3b7669adbcea32943679e3202e2bb7ef119323f03e980484`。

正式运行：`runs/x650_cpp_normal_v1_20261024_172622_46636`；启动记录：`runs/x650_cpp_normal_launch_20261024_172532_46633`。实际物理窗口为仿真时间42.848–74.848 s。原始服务器结果位于 `artifacts/cpp_normal_20261024/normal_audit_v1/report.json`。

## 动态参考与软件验证

| 证据 | 覆盖范围 |
| --- | --- |
| [动态参考与预览](software/reference_proof.json) | 编译后的C++轨迹与原Python公式，3,370组／53,920个当前与未来目标；包含完整0–32 s、C2边界与12/16/20/24 ms首步 |
| [48个动态数值工况](software/numerical_proof.json) | 同一冻结配置下的矩阵、优化计划、预测状态、非线性候选、MIT量化字段与输出对照全部通过 |
| [新工作进程实际ROS证明](software/worker_ros_proof.json) | 160步连续0.02–3.2 s动态软件输入，以及错误目标、错误相位、错误配置的三个120步锁存退出场景；不是32 s物理飞行替代物 |
| [构建与CTest](software/build_and_ctest.log) | 独立Release工作区构建及4项C++契约测试通过 |
| [地面准入](software/ground_admission.json) | 重新核对先前完整保持、新动态证明、原流式执行证明及15类实际ROS消息序列化往返 |

相关本地控制／准入回归共105项通过，原渲染测试8项通过，[验证摘要](validation_summary.json)保存各层范围。软件验证保留两次失败设置：首个合成序列的前史包早于声明总线起点；之后一次校验错误要求跨语言CDR填充字节也相同。仅修正软件输入及校验前提，未放宽生产门控或数值误差。最终保留两端各自CDR/SHA，全部声明字段严格相等；原始失败版本与报告都保存在ZIP中。

## 复现入口与边界

见 [独立普通场景](../../../simulations/x650_cpp_normal_v1/README.md)及[完整C++控制链](../../X650_CPP_PIPELINE.md)。先加载匹配的ROS interfaces、PX4、状态源、网关和新worker overlay，再依次执行：

1. 用原保持task及其完整验收报告构造normal task；绑定相同数值库和新的normal startup manifest。
2. 执行 `validate_x650_cpp_normal_reference.py`、`export_x650_cpp_normal_inputs.py`／`validate_x650_cpp_pipeline.py`，以及 `x650_cpp_normal_sequence.cpp`／`probe_x650_cpp_normal_worker.py`，得到真实软件证明。
3. 通过 `simulations/x650_cpp_normal_v1/launch.py --dry-run` 的全部检查后，使用同一输入启动实际仿真。
4. 用 `scripts/audit_x650_cpp_normal.py` 对完整原件验收；通过后，`scripts/render_x650_cpp_normal.py` 分别用 `--plots-only`、`--preview-only` 和完整模式制作交付物。

原始ZIP保存了本次实际命令、库／源码身份、输入、成功报告及失败尝试。跨机器恢复证据时需要保持其原件结构和身份，不能直接修改已绑定路径／SHA后继续使用原证书。

本结论限定于本次精密定位条件仿真的完整普通八字。快速八字、Orin机载时间分布和24 V负载真机条件还需各自验收。下一阶段先推进快速八字；之后再开展扰动、模型偏差、通信延迟及机载计算测试，并据相同工况与全身NMPC比较。
