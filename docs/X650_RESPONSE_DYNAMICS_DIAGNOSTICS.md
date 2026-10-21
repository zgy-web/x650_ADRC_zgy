# X650 水平响应链与二阶模型开发诊断

这一步分析已经完成并独立审计的两轮 `precision_ev_upper_v2` 响应实验，不启动新飞行、不修改 PID、不覆盖原始报告，也不生成正式控制模型或阶段放行证书。研究范围仍是 X650 加折叠二连杆机械臂、理想外部视觉定位、零加速度前馈下的闭环基体响应。

## 模型结构开发与解释范围

比较单位静态增益、零拟合纯时延和零偏置的一阶模型与以下稳定二阶候选：

\[
\dot v_i=a_i,\qquad
\dot a_i=\omega_{n,i}^{2}(u_i-v_i)-2\zeta_i\omega_{n,i}a_i.
\]

全部参数只用训练记录选择，再冻结并重放旧验证记录。旧验证记录已经用于发现一阶不足和选择新结构，因此这种复用属于模型开发，不能称新模型的全新独立验收。实际候选参数、误差与判定保存在每次运行的开发报告，数值不硬编码进控制器。

两个候选都从首个原生状态连续自由推进到 14 s，使用相同训练 1–14 s 窗口拟合，没有在每个脉冲或轴窗口注入实际状态。报告同时提供 0–14 s、1–14 s、各轴四秒激励区间的误差，避免长静止段掩盖动态误差。原有一阶辨识器使用每轴四秒拟合，不能把不同窗口得到的参数与本工具结果混作同一实验。

响应链按原始时间展示逻辑速度指令、内部速度目标、加速度需求、投影姿态目标、实际估计姿态与独立原生物理状态。阈值首次达到时刻属于闭环响应过程，不等同于通信或纯姿态环延迟；通信范围仍以已有逐条 PVA 收发审计为准。姿态目标的日志分辨率与实际姿态不同，必须结合各主题的实际记录间隔解释。

物理加速度来自记录器 WorldLinearAcceleration，并用原时间梯形积分与物理速度交叉检查。速度 PID 的积分/微分、姿态与动力响应、臂载荷都可能贡献误差。二阶候选只是当前工况的输入输出近似，不证明完整系统真实阶次只有二阶，也不能把一阶延迟扫描的边界最优值称为已识别通信延迟。

## 复现工具

在仓库根目录使用已有 NumPy、SciPy、Matplotlib 的 Python。链路分析还需要 PyULog。先给 `TRAIN_RUN`、`OLD_HOLDOUT_RUN`、`TRAIN_AUDIT`、`OLD_HOLDOUT_AUDIT` 设置两个原始运行目录和各自独立上层审计目录；`NEW_CHAIN_OUTPUT`、`NEW_MODEL_OUTPUT` 必须是尚不存在的新目录。

```bash
python3 scripts/analyze_x650_response_chain.py \
  --train "$TRAIN_RUN" --holdout "$OLD_HOLDOUT_RUN" \
  --train-audit "$TRAIN_AUDIT" --holdout-audit "$OLD_HOLDOUT_AUDIT" \
  --output "$NEW_CHAIN_OUTPUT"

python3 scripts/compare_x650_response_models.py \
  --train "$TRAIN_RUN" --old-holdout "$OLD_HOLDOUT_RUN" \
  --train-audit "$TRAIN_AUDIT" --old-holdout-audit "$OLD_HOLDOUT_AUDIT" \
  --output "$NEW_MODEL_OUTPUT"
```

本次服务器分析使用系统 `/usr/bin/python3` 的 NumPy 1.21.5、SciPy 1.8.0、Matplotlib 3.5.1；未下载依赖。现有仿真虚拟环境中的 NumPy 与系统 SciPy 不兼容，不应直接用该组合跑优化。PyULog 使用数据盘上现有包；必要时仅在分析进程中将它所在的 site-packages **追加**到系统 Python 的搜索路径末尾，避免优先覆盖系统数值库。示例（路径变量指向已存在的 PyULog 包目录）：

```bash
env -u PYTHONPATH PYULOG_SITE_PACKAGES="$EXISTING_PYULOG_SITE" \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 -c \
  'import os,sys,runpy; sys.path.append(os.environ["PYULOG_SITE_PACKAGES"]); script=sys.argv.pop(1); sys.argv[0]=script; runpy.run_path(script,run_name="__main__")' \
  scripts/analyze_x650_response_chain.py \
  --train "$TRAIN_RUN" --holdout "$OLD_HOLDOUT_RUN" \
  --train-audit "$TRAIN_AUDIT" --holdout-audit "$OLD_HOLDOUT_AUDIT" \
  --output "$NEW_CHAIN_OUTPUT"
```

链路分析输出两幅 PNG/PDF 和 `response_chain_diagnostics.json`。每图三行依次为速度、加速度、世界轴投影倾角；两列分别放大 X、Y 的脉冲区间，所有数据仍保持原始时间。原始状态采用 `timestamp_sample`，需求采用 `timestamp`，物理状态采用 `sim_s`；没有曲线平移或相关性对齐。投影倾角由四元数转换后机体向上轴计算，不把 Euler roll/pitch 直接当世界 X/Y。

链路报告 schema 2 保留真实相邻采样的阈值跨越区间。区间跨越命令时刻会标记因果起点未定，不能剪裁区间来制造因果证据；搜索在本次命令实际结束时停止，回零后才达到阈值不算本次脉冲上升时间。开发时的 schema 1 输出保留，不作最终时间区间证据。

模型比较输出 `response_model_development.json`、PNG 和 PDF。每次仅取首个实际速度初始化，固定 `a0=0`；首秒零输入、速度峰值和趋势检查支持该近似，首秒不参与拟合分数。优化用训练记录，不用旧验证记录选参数、初始状态、时移或偏置。输出明确标记 `development_only=true`、`new_model_independent_validation=false`、`eligible_as_stage_gate=false`。

两工具均复用原始文件哈希、完整 ULog/独立物理审计及工况一致性检查。分析不修改旧辨识器、PID、飞行 runner、验收阈值或旧失败结果。

## 下一轮正式验证

冻结 X/Y 二阶、Z 一阶结构和参数选择规则后，使用新的训练记录与完全未参与结构选择的新验证记录。激励应包含事先确定的不同脉宽和回零段，保持现有幅值、关节/飞行包络、PID 与精密定位条件。整段连续自由预测；验证记录不能调整参数、初值、时移或偏置。完整物理/ULog/参数恢复审计与固定误差门槛都通过后，才更新控制包的模型契约和阶段准入，再做末端保持及八字。活动机械臂、非零加速度前馈和实机实时性仍需单独验证。

回归命令：

```bash
python3 -m unittest discover -s tests/unit -p 'test_x650_response_*.py' -v
```

当前 19 项测试覆盖不规则时间精确传播、已知二阶系统辨识、训练/验证隔离、初始状态不重置、坐标方向、时钟语义、数据缺口和因果事件边界。合成测试只验证工具，不替代上述服务器记录。
