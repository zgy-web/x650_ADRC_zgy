# X650 新记录的响应模型正式验证

本轮把之前的模型开发结论固定下来：水平 X/Y 使用单位静态增益、零纯时延的二阶速度响应，Z 保留一阶速度响应。原有训练记录和已经查看过的旧验证记录只属于开发证据，不参与本轮正式参数拟合或独立验收。

当前文件说明已实现的实验流程；是否实际完成飞行、参数拟合和验收，以新运行目录中的原始日志、独立审计和最终报告为准。代码与测试通过不能代替仿真实验通过。

## 事先固定的激励

每条记录的测量阶段为 60 秒。X、Y、Z 分别占用 0–18、18–36、36–54 秒，最后 54–60 秒恢复固定悬停位置。各轴开始先保持 2 秒零速度指令；只激励一个轴，另外两轴速度指令为零。

| 记录 | 每轴局部时间内的四个脉冲（秒） | 脉宽（秒） |
| --- | --- | --- |
| 新训练 | 正向 2–2.8；反向 5.3–6.7；正向 9.2–10.6；反向 13.1–13.9 | 0.8、1.4、1.4、0.8 |
| 新独立验证 | 正向 2–3.1；反向 5.4–7.1；正向 9.4–11.1；反向 13.4–14.5 | 1.1、1.7、1.7、1.1 |

X/Y 幅值均为 0.08 m/s，Z 为 0.05 m/s。正反向指令积分平衡，每个脉冲后至少有 2.3 秒零速度段。这里的“回零”指速度指令回到零，不等于瞬时回到原位置；只有最后 6 秒执行原有固定位置恢复控制。

物理模型、二连杆折叠姿态、42 个飞控控制参数、精密外部视觉条件和原有安全包络保持既定配置。实验序列有独立的协议哈希，不能借修改协议重新定义物理工况或沿用旧实验的正式验收结论。

## 参数选择与验收顺序

1. 首次飞行前保存实验预注册文件：两组完整协议、任务模板、分析与控制源码哈希、PX4 二进制哈希和预检结果。
2. 单独运行新训练实验，完成降落、解锁状态核查、参数恢复、完整 ULog 与独立物理记录审计。
3. 仅由训练记录选择 X/Y 的自然频率和阻尼比、Z 的时间常数。保存训练报告和冻结候选模型。
4. 验证任务在起飞前绑定候选文件的路径和 SHA-256，并将候选原始字节存入该次启动目录。然后才采集不同脉宽的新独立验证记录。
5. 验证时固定模型参数、单位增益、零偏置、零纯时延和初始加速度约定。整段连续推进，不在各轴或各脉冲重新用实际状态初始化。分别检查动态段、零速度段和整段误差，不能用长时间静止稀释动态误差。
6. 完整证据审计和所有预先规定的模型误差门槛通过后，才生成可接入控制器的响应模型与验收证据。失败记录原样保留；不根据验证结果修改门槛或反复拟合该验证组。

精确参数边界、优化设置、评分窗口和误差门槛存放在 `simulations/x650_wholebody_mpc_v1/response_protocol.py`，随预注册一同绑定。新工具独立于旧 30 秒实验的审计器和辨识器，避免改变旧证书依赖的源码。

X/Y 在正参数的对数空间用固定的三个初始点拟合，范围为自然频率 0.3–30 rad/s、阻尼比 0.1–5；Z 的时间常数范围为 0.01–1.5 s。只在训练数据上选择误差最小且收敛的结果，检查参数是否贴边及对参数变化是否有足够敏感度。独立验证阶段没有参数优化入口。

协议规范 JSON 的 SHA-256：

- 训练：`2f30b52f768fb785738aae7244031197fa032fc6911ccc7d27442d05479c2305`
- 验证：`860e9cbec5b3991c0e9800fb3df749257b7d7c01dc46e9686b8d2d5a5e3c6dd2`

## 运行入口

下列变量必须指向服务器上的真实路径。所有输出目录都必须是新的；工具拒绝覆盖已有证据。`TRAIN_TASK`、`HOLDOUT_TASK` 是按上述协议构造的任务模板，后者在预注册时尚未绑定候选。`PREFLIGHT` 保存训练任务的实际服务器预检结果。预注册工具自动核验并纳入预检列出的全部控制源码、分析器与 driver；`--source` 可补充其他执行依赖。

预注册同时生成不可覆盖的 `.sources.zip` 源码快照。训练和独立验证期间仍核对当前源码与快照一致；最终证据保留快照及其哈希，后续接入 MPC 时无需为了维持历史证据而冻结整个工作目录。

```bash
python3 scripts/identify_x650_response_protocol.py freeze \
  --train-task "$TRAIN_TASK" --holdout-task "$HOLDOUT_TASK" \
  --px4-binary "$PX4_BINARY" --preflight "$PREFLIGHT" \
  --source scripts/run_x650_formal_response.py \
  --source scripts/run_x650_precision_candidates.py \
  --output "$PLAN"

python3 scripts/run_x650_formal_response.py \
  --task "$TRAIN_TASK" --preregistration "$PLAN" --output "$TRAIN_JOB" \
  --launch-python /root/autodl-tmp/amsim-venv/bin/python3 \
  --analysis-python /usr/bin/python3 \
  --pyulog-site /root/autodl-tmp/amsim-venv/lib/python3.10/site-packages

python3 scripts/identify_x650_response_protocol.py fit \
  --train "$TRAIN_RUN" --train-audit "$TRAIN_AUDIT" \
  --preregistration "$PLAN" --output "$TRAIN_FIT"
```

只有 `fit` 成功生成 `frozen_response_candidate.json`，才把其绝对路径及原始文件 SHA-256 写入验证任务的 `frozen_response_candidate`、`frozen_response_candidate_sha256` 两字段；其余任务字段必须与预注册模板完全一致。随后用同一个单次 driver 启动验证任务，降落和独立审计完成后运行：

```bash
python3 scripts/identify_x650_response_protocol.py validate \
  --frozen-candidate "$TRAIN_FIT/frozen_response_candidate.json" \
  --holdout "$HOLDOUT_RUN" --holdout-audit "$HOLDOUT_AUDIT" \
  --preregistration "$PLAN" --output "$VALIDATION"
```

这些命令的监督/分析进程使用现有系统 Python、NumPy 和 SciPy；飞行进程由 `--launch-python` 指向原仿真虚拟环境。若分析进程需要现有 PyULog，应将其 site-packages 追加到搜索路径末尾，避免虚拟环境的 NumPy 覆盖系统数值库。长时间运行的单次 driver 应放在 `screen` 等已有后台会话内。driver 每次只运行一条记录并审计，任何失败立即停下，不自动重试或推进到下一个实验。

## 与后续 MPC 的边界

本轮识别的是折叠机械臂、零加速度前馈条件下，PX4 闭环从逻辑速度指令到实际基体速度的响应。它不是整机力矩动力学模型，也不能自动证明活动机械臂与非零加速度前馈的预测精度。

通过后，上层 MPC 将在现有位置、速度和关节状态之外增加水平加速度状态，使用实际估计器反馈初始化；独立物理真值仅用于评价。需先检查反馈语义和离散模型，再运行末端保持，随后按小幅、普通八字的顺序验证。摆臂和前馈开启的适用范围要在这些阶段单独核查，不将本轮通过解释为全身 NMPC 或实机性能通过。
