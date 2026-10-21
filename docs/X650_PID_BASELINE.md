# X650 带臂 PID 基线：工程冻结与正式整定的边界

当前上层 MPC 的前序调试需要一套实测可稳定运行且参数固定的 PID。正式宣称 MPC 优于“充分整定的 PID + 前馈”，还需要单独的原生参数选择与独立验证。本页将这两种证据分开；不能把默认参数复制出来就称为最优基线。

## 当前只读工程冻结

`scripts/freeze_x650_pid_baseline.py` 只读取六个已成功完成并降落解除武装的 native run：`hover_folded`、`joint_probe`、`tip_hold`、`small`，以及两次独立 `response_probe`。它不改参数、不启动仿真、不执行优化。

```bash
PYTHONPATH=/usr/lib/python3/dist-packages \
  /root/autodl-tmp/amsim-venv/bin/python3 scripts/freeze_x650_pid_baseline.py \
  --hover /absolute/hover/run \
  --joint /absolute/joint_probe/run \
  --tip /absolute/pid_ff_tip_hold/run \
  --small /absolute/pid_ff_small/run \
  --response-train /absolute/response_probe_run_1 \
  --response-holdout /absolute/response_probe_run_2 \
  --output generated/x650_pid_engineering_new_pair
```

原始 stage、metadata、控制 CSV、参数前后快照、native/flight CSV 与 manifest 都要能回查哈希。两次 response run 不能是相同 CSV 或重新保存的同一规范化数据。六次全部已捕获的实际 MC/MPC 值须一致，且每次前后读回一致；必须包含原定义的位置/速度/姿态/角速度 26 个增益名称，但值来自新机读回，不抄旧机参数。这不是 PX4 全量参数导出。组合模型、URDF、airframe、flight profile、准备报告以及共用模型/PID/FF 源码哈希也要一致。适应性悬停推力或控制限值若改变，同样不能被当成同一冻结 MC/MPC 配置。

工具从原始独立 `native_trace.csv` 重算位置/末端、姿态、关节与采样指标。目标由运行时保存的固定目标、基座锚点、旋转、显式物理 profile 和冻结小八字协议重构；实际末端使用 native 插件测得的 `tip_x/y/z`，不用控制器 FK 冒充测量。没有时间平移或事后重定义目标。

| 项目 | 预先固定工程容差 | 含义 |
|---|---|---|
| 无动作带臂悬停 | native 三维位置 RMS ≤ 0.10 m、峰值 ≤ 0.20 m | 比原 commissioning 0.25/0.50 m 更紧，仍是可用性检查 |
| 末端定点保持 | native 末端 RMS ≤ 0.05 m、峰值 ≤ 0.10 m | 初步末端稳定，不等于精密操作指标 |
| 小八字 | native 末端 RMS ≤ 0.04 m、峰值 ≤ 0.10 m | 幅值 75/30 mm；完全静止在中心的理论 RMS 约 57 mm，40 mm 能排除“未追踪但稳定” |
| 共同倾角 | 最大 8° | 初步小动作包线，严于原飞行异常门槛 |
| 原生采样 / 控制间隔 | 最大 0.008 / 0.05 s，覆盖完整任务区间 | 0.05 s 是异常上限，不是严格 50 Hz 的证明 |
| 关节与机体指令 | 沿用 0.45/0.35 rad/s 和 0.55/0.35/0.45 m/s 上限；实际 q 在原物理限位内 | 不放宽既有控制/物理边界 |
| 关节 probe | 原有双向运动、符号相关、RMS ≤ 0.02 rad、峰值 ≤ 0.05 rad、幅值/速度/基座位移检查 | 从 native 原始数据复核，不仅相信 cached `passed` |
| 两次 response probe | 三轴完整正负激励、实际速度跨度、原生/控制采样及共同包线 | 不把“是否恰好能用零延迟一阶模型拟合”混为 PID 稳定性的定义 |

这些容差是上层 MPC 调试所需的工程 acceptance；是否通过应以六项任务的真实运行和冻结工具输出为准，本页不宣告整套工程冻结已经验收。它们不能支持“PID 已最好”“已消除全部 PID 影响”或“已充分整定”的说法。`tip_hold` / `small` 当前原生 stage 的完成标志本身没有末端误差阈值，所以本工具另外重算并验收误差，不把“飞完了”当成“效果已经合格”。

通过后输出：

- `pid_profile.json`：完整实际 MC/MPC 参数及规范化哈希，可供 runner 的只读参数核对。
- `pid_baseline.json`：`stage="pid_baseline"` 的原生工程证据，绑定全部原始文件和参数 profile 哈希。

默认工程指标通过时，stage 可以 `passed=true`，用于上层 MPC 前置调试；同时始终写入：

```json
{
  "baseline_scope": "engineering_stability_only",
  "tuning_claim": false,
  "formal_fully_tuned_comparison_ready": false,
  "globally_optimal_pid": false
}
```

失败时保存 `passed=false` 及具体指标或证据缺口，退出 2，不覆盖旧输出。`--strict-tuning` 还要求 `--tuning-evidence`：一份独立外部有限搜索 audit，必须列出至少两个实际参数不同的 native response 训练配置（包含所选配置），原始哈希与 PID 读回可查，并与六次验证数据无重叠。这个选项检查整定证据的存在与来源一致性，**不会自动把本工具的输出升级为充分整定或全局最优声明**；候选范围、评分和覆盖是否充分仍需单独审查。

## 不存在的循环依赖与实际缺口

当前 `task.required_gates` 对 `pid_ff` 的 `tip_hold` 和 `small` 只要求已有 native ground/hover/joint 等前序证据，不要求 `pid_baseline`；所以可以先完成它们，再生成工程 baseline。`mpc` 的 tip/small 和正式 normal/fast 才继续要求 PID 与响应证据。没有必要伪造一个先验通过文件来解锁验证。

`pid_profile` 仍然只读核对实际 PX4 参数，不能用来应用新增益；给 Python allocator 传一组 gains 也不会自动修改 PX4。现在另有显式 `pid_tuning_candidate` 入口，通过 `diagnostic_parameters.py` 的可逆参数事务应用已审查候选。该入口只允许 `hover_folded / pid_ff`，必须配合固定世界位置参考，不能同时传正式冻结的 `pid_profile`。

三个可选 profile 与候选字段各有独立作用，默认均为 `null`：

| 字段 | 当前范围与作用 |
|---|---|
| `hover_reference_profile` | `fixed_world_position_v1` 加实际世界 SDF 的绝对路径；可用于 hover/joint commissioning。固定世界目标一次性转换到实际 PX4 原点，直接启用位置/yaw，禁止每步 truth-minus-estimate 改写目标 |
| `diagnostic_profile` | 仅 hover 诊断；接受代码中明确的水平 EV 辅助或 EV XYZ+速度精密两种配置。开启时使用运行期独立观测插件及原生 GZBridge，必须确认实际融合，记录 `ground_truth_aided=true` |
| `precision_test_profile` | 仅 hover 诊断，值为 `hover20_steps36_v1`；主悬停仍为 20 s，之后增加 36 s 位置阶跃 |
| `pid_tuning_candidate` | 仅 hover 诊断；接受固定九项目录中的名称或完全相同的四参数字典，应用真正 PX4 参数。`diagnostic_profile=null` 时保持原 GPS/气压估计来源 |

候选只写 `MPC_XY_P`、`MPC_Z_P`、`MPC_XY_VEL_P_ACC`、`MPC_Z_VEL_P_ACC`。位置组为 baseline `(0.95,1.0)`、low `(0.75,0.8)`、high `(1.2,1.2)`；速度组为 baseline `(1.8,4.0)`、low `(1.5,3.2)`、high `(2.2,4.8)`，组合形成九项固定目录。名称和权威数值见 `diagnostic_parameters.PID_CANDIDATES`。这不是自由传入任意 PID，也没有改变 I/D、姿态/角速度环、执行器、动力、速度/倾角限制。

每次独立运行先确认新鲜的 landed/disarmed 状态，保存全部 42 项控制参数的原值、类型与原始字节，再写候选四参数并读回。待恢复日志在写入前落盘；落地解除武装后恢复原值、读回并同步保存 PX4 参数存储，成功后才清除日志。失败保留日志并阻止下一次飞行。参数采集器继续只读，实际事务由单独模块执行；`parameter_writes=true` 与写入记录会保存在 metadata 中。

固定位置参考绕过原有按期望速度编码的上层入口，使 PX4 原生位置环承担位置误差反馈，因而位置 P 候选能够被实际激励。六段阶跃分别为 X +0.15 m、归零、Y +0.15 m、归零、Z +0.10 m、归零，各 6 s；用首次实际发令时刻分析，不把测得位置重新设为下一段中心。这是位置指令响应，未注入外力。原生位置误差 ≤0.02 m 且速度 ≤0.03 m/s 连续保持 2 s 才计为观察到恢复；完整 6 s 内未达到应报告未稳定。

`scripts/run_x650_precision_candidates.py` 可从已核实的精密 EV 基线顺序运行固定目录候选，校验任务哈希、物理身份、42 项原值、候选实际读回及复原；任一飞行或分析失败即停止。理想 EV 是定位影响诊断，选定候选仍需在原 GPS 条件复核。GPS 任务可使用相同固定目标/阶跃协议并令 `diagnostic_profile=null`；GPS 有效参数与全窗融合审计见只读 `scripts/audit_x650_gps_crosscheck.py`，坐标分解仍使用 WGS84 方法。

启用估计诊断、PID 候选或附加阶跃后，结果标为 `diagnostic_only=true`、`eligible_as_stage_gate=false`，不能替代正式 MPC 的 `pid_baseline`、响应辨识或 pilot gate。有限目录中的局部比较不等于充分整定，也不宣布全局最优；选择完成后仍需原 GPS 复核、独立验证和正式工程冻结。

## 正式比较前的最小原生整定/验收矩阵建议

以下是后续正式比较的可执行工作计划，不是现有工程调试 gate 的新增强制条件，也没有被伪装成已经完成的实验。

| 阶段 | 工况与参数变化 | 决策依据 |
|---|---|---|
| 起点 | 当前工程 baseline 的 hover、joint、tip、small 与双 response probe | 先确认模型、执行器、反馈和包线；不能用它声称最优 |
| 水平训练 | 保持 MC 姿态/角速度与输出固定，在已声明参数范围内比较位置/速度 P；当前实现为上述九项成对 XY/Z 目录和固定位置阶跃。若另采用独立 XY 九点网格，应先声明新协议，不能混称为当前目录 | 同原生时间轴的位置/速度响应、超调/恢复、倾角与饱和；保存全部失败，不只保留胜出视频 |
| 垂向检查 | 依据预声明触发规则检查 Z 响应；若超调、恢复或偏差不合格，按相同 ±20% 单因素规则检查 Z 位置/速度 P | XZ 八字必须有垂向证据；不默认 XY 最好就足以代表 Z |
| 曲线训练 | 合格候选中预先规定的前三名跑 `pid_ff small`；权重/合成评分在看结果前固定 | 同步 native 末端误差、动作幅度和约束余量，避免仅追求快而振荡 |
| 选择与独立验证 | 选定后冻结所有实际 MC/MPC，在未参与选择的新 hover/joint/tip/small 与双 response probe 上验证 | 独立重复、同模型/前馈/反馈/包线；不把训练曲线作为验证 |
| 正式对照 | 固定该选定 PID 与响应模型，普通/快速八字对 PID+FF 和 MPC 使用相同条件，保留配对重复 | 只能说“在声明参数范围和工况内选定的基线”，始终不宣称数学全局最优 |

若胜出参数在搜索边界，或姿态/rate 动态本身不合格，应报告需要扩展/分阶段整定，不能停止搜索后直接称为充分。若要改变积分/微分或内环参数，应先依据原生 ULog 的速率/超调和扰动残差提出另一个明确训练阶段，避免在最终八字上随意调到最好看。

本轮上层 MPC 调试可以在工程冻结后继续；正式“优于充分整定 PID”的结论仍由上述独立工作支撑，工程 gate 的通过不会自动补齐这项证据。
