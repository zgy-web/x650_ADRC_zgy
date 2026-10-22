# X650 正式响应实验的证据绘图

`scripts/plot_x650_formal_response.py` 展示已有训练或独立验证记录。它读取冻结参数，按原始指令时间零阶保持、从每条记录最初的物理速度和固定零加速度连续计算预测。它不调用优化器，不移动时间轴，不在每个轴或脉冲开始时重置状态，也不输出新的验收证书。

训练报告必须已完成数值拟合，并保存三个轴的参数与误差；`passed=false` 的完整报告仍可以绘图，标题明确标为 **REJECTED / TRAIN ONLY**。只有不完整错误记录、没有数值拟合结果时，脚本会拒绝生成不存在的预测曲线。训练通过仍标为 **TRAINING ACCEPTED / TRAIN ONLY**，不会被显示成独立验证通过。若提供独立验证报告，其标题显示原报告的 **HOLDOUT PASS** 或 **HOLDOUT REJECTED**。

## 运行方法

在已有 NumPy、SciPy、Matplotlib 的系统分析 Python 环境执行，避免混用仿真虚拟环境的数值库。绘图不启动 Gazebo 或 PX4；完成飞行后再运行。

```bash
env -u PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 scripts/plot_x650_formal_response.py \
  --training-report /absolute/train_fit/training_report.json \
  --output /absolute/new_training_plot
```

独立验证完成后：

```bash
env -u PYTHONPATH OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /usr/bin/python3 scripts/plot_x650_formal_response.py \
  --training-report /absolute/train_fit/training_report.json \
  --frozen-candidate /absolute/train_fit/frozen_response_candidate.json \
  --validation /absolute/validation/response_identification.json \
  --output /absolute/new_validation_plot
```

输出目录必须不存在。每条记录输出一张 PNG 和一份 PDF，另生成 `render_manifest.json`，记录所读输入、绘图源码、输出文件的 SHA-256，以及原报告判定和 `eligible_as_stage_gate=false`。其中 `axis_reported_diagnostics` 原样保存每个轴的拟合诊断、完整评价结果、各脉冲/零速度段判定及失败原因，便于解释“整段 RMSE 很小但仍然 REJECTED”的情况。

若只将必要的 CSV、JSON 和预注册源码快照复制到本地，可额外提供 `--input-map /absolute/path_map.json`。映射格式为原始文件绝对路径到实际副本路径的 JSON 对象。每份被读取的副本仍必须符合原始记录中的哈希；不得修改报告内的路径或哈希来迁就副本。

## 图中内容和验证范围

每条记录为三行、两列。每行对应一个轴的完整 18 秒区间；左列为实际记录的逻辑速度指令、独立物理速度及冻结模型预测，右列为“预测减物理速度”的残差。浅黄色为事先确定的脉冲，浅灰色为零速度指令段，水平参考线为零。X 轴原报告评分从第 1 秒开始，但图仍保留第 0–1 秒静止检查段。

预测在 0–54 秒连续推进；图只按各轴的原始窗口裁切显示，不重新初始化。54–60 秒的位置恢复段不属于本轮模型评分，未混入这些曲线。图中的 RMSE 和最大残差来自原报告，并用重放结果逐项核对。脚本只核对既有整体、逐轴及嵌套拟合/评价判定是否一致，不重新执行验收门槛，也不根据画面或重放误差修改原判定。

读取过程检查原始物理审计与证书的绑定关系、采样完整性、模型与全部 42 个 PID 参数身份、任务协议、CSV 数据指纹、预注册源码快照及预测器源码哈希。独立验证还检查候选与训练报告的字节绑定及起飞前的候选归档。这里重复校验用于显示的原始数据与已有验收链，不重新执行完整 ULog 审计，也不代替正式模型验收。

该图表示基体速度模型的预测表现；它不能证明 MPC 跟踪精度、机械臂主动补偿效果或硬件性能。
