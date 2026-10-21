# X650 精密上层响应辨识：结果重放与绘图

本工具重放 `precision_ev_upper_v2` 条件下的上层速度响应辨识。训练与独立保留验证各自完成 30 秒响应探测，必须先通过完整 native／ULog、实际 PVA 收发、估计状态反馈及参数恢复审计。**飞行审计通过与响应模型通过是两个独立结论**，模型判定以所选辨识目录的 `response_identification.json` 为准。

## 重放已有辨识结果

在服务器仓库根目录运行。使用已检查可用的系统 Python；清除本次进程继承的 `PYTHONPATH`，避免混入其他虚拟环境的 NumPy／SciPy。无需安装或修改任何环境。

```bash
env -u PYTHONPATH /usr/bin/python3 scripts/plot_x650_upper_response.py \
  --identification-dir artifacts/x650_precision_upper_20261021/response_identification_v1 \
  --output artifacts/x650_precision_upper_20261021/response_identification_figures_v1
```

输出目录必须尚不存在。脚本生成 PNG、PDF 和 `plot_manifest.json`，并记录输入、输出、绘图源代码及预测器的 SHA-256。运行依赖 NumPy、SciPy、Matplotlib，以及仓库现有辨识和研究条件模块。

六个子图分别展示 X、Y、Z 三轴的训练与保留验证响应，采用原始任务时间窗口 2–6、6–10、10–14 秒：灰线为实际记录的逻辑速度指令，蓝线为独立 native 物理速度，橙色虚线为**冻结训练时间常数**的预测。预测只在各轴窗口起点采用该次运行的首个实际速度初始化，之后自由推进；没有重新拟合时间常数、移动曲线、去均值或曲线降采样。图中 RMSE 取自已有辨识报告，并用相同原始样本重放检查。

完整但未通过的辨识报告仍可绘图，会醒目标注 **MODEL REJECTED**。这表示现有模型不满足预定验收条件，图不能解锁下一控制阶段，也不是 MPC 或末端主动补偿效果。时延诊断只用于解释模型为何被拒绝；诊断搜索上界处的最优值不能直接当成已识别的真实时延。缺少完整输入绑定的失败报告不能绘制有依据的响应曲线。

## 在本地用精简副本重放

无需为绘图重新传输整个 ULog。保留以下原文件内容不变：

- 辨识目录的 `response_identification.json`、`response_model.json`。
- 每次运行对应的 `upper_stage_audit.json` 和 `research_stage.json`。
- 每次运行的 `controller_metadata.json`、`controller_task.json`、`stage_result.json`、`native_trace_manifest.json`、`native_trace.csv`、`control_trace.csv`。

另外建立一个 JSON 路径映射，将报告中每个已复制文件的**原始完整路径**映射到本地副本的绝对路径；模型键保留为 `response_model.json`。通过 `--input-map /absolute/path/input_map.json` 传入。脚本逐件核对原始 SHA-256 和原始样本指纹，不改写报告中的服务器路径。这是对已有完整审计的可核验重放，不是重新解析或重新验收 ULog。
