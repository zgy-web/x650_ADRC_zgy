# 已通过 mixed-stage 实验的原生状态回放

`scripts/prepare_x650_mixed_native_replay.py` 只接受完整通过的 mixed-stage 审计。它验证该次原始 native CSV、manifest、metadata 和任务的哈希，复现正式时间加权末端指标，并验证同物理步末端与机体/关节 FK。随后逐行映射完整测量窗口，保留原始时间及状态字段文本；不平移原点、不减均值、不拟合时间或相位、不重采样。

```bash
python scripts/prepare_x650_mixed_native_replay.py \
  --audit AUDIT_DIR/mixed_mpc_stage_audit.json \
  --run REAL_RUN --output NEW_DERIVED_DIR

MUJOCO_GL=egl /root/autodl-tmp/amsim-venv/bin/python scripts/render_x650_arm_replay.py \
  --source generated/x650_2link_v2_20261021/robot.urdf \
  --csv NEW_DERIVED_DIR/native_replay.csv \
  --capture-manifest NEW_DERIVED_DIR/native_replay_manifest.json \
  --output NEW_PREVIEW_DIR --preview-only
```

确认预览后，以新的输出目录去掉 `--preview-only`，使用现有网格回放器生成 1440×900、24 fps、1 倍仿真时间的视频。只渲染接受窗口，未加入起降阶段；时间从原记录首样本起显示，但派生 CSV 保留绝对仿真时间。完整窗口内的每个 native 样本都保留，不跳过无效点。参考为已接受任务在原始时间上的解析目标，机体目标列是冻结世界 anchor，不是实时分配的机体指令。

派生 manifest 的 kind 为 `x650_mixed_native_derived_replay_input`，明确 `original_flight_recorder_capture=false`，并记录原始哈希、窗口、字段映射和代码哈希。它只是兼容回放器的派生输入，不能作为新验收证书，也不是原始 FlightRecorder 记录。视频是 Gazebo 记录状态的离屏回放，不是实时 Gazebo 屏幕录像；原网格及实测关节姿态沿用，未记录的桨叶角度不会虚构。

现有回放器会对原生末端与 FK 再做逐样本检查，但其附带 FK/按样本均值指标仅用于展示。正式结论沿用 mixed-stage 审计和 `plot_x650_mixed_stage_comparison.py` 的原生时间加权指标。不得通过调整渲染容差改变实验验收。

## 左右同步八字对照视频（2026-10-22 新版）

用户要求的默认交付格式改为：左侧 PID 协同分配、右侧上层 MPC，上方为实时三维末端误差、完整窗口时间加权 RMSE 和实际 roll/pitch；三维场景中为黄色完整目标八字、同色目标标记和逐步延伸的实际末端轨迹；下方为共用纵轴的两条同时间末端误差曲线。小幅、普通阶段分别输出一个 1600×1320、24 fps、1 倍时间的视频。

`render_x650_paired_figure8.py` 复用经过验证的原机架网格和关节状态回放，同时再次读取原生验收报告、复核输入哈希、重算正式指标、检查成对条件和每个样本的 FK。左右按各自预先声明的测量开始时间同步，使用一个共同固定的世界相机；不根据实际跟踪结果拟合时间、平移原点或调整左右缩放。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
/root/autodl-tmp/amsim-venv/bin/python scripts/render_x650_paired_figure8.py \
  --source generated/x650_2link_v2_20261021/robot.urdf \
  --pid artifacts/x650_figure8_prediction_chain_20261022/pid_normal_native_replay \
  --mpc artifacts/x650_figure8_prediction_chain_20261022/mpc_normal_native_replay \
  --output NEW_PAIRED_PREVIEW_DIR --preview-only --preview-time 10
```

检查预览后，以新的输出目录去掉 `--preview-only --preview-time 10`。输出的 `X650_normal_PID_vs_MPC.mp4` 或 `X650_small_PID_vs_MPC.mp4` 会完整解码验证；`replay_manifest.json` 保留两侧原始记录哈希、相机、时间同步和指标语义。若任一原始证据变化、阶段不匹配、时长不一致或验收不通过，将拒绝渲染。

顶部 RMSE 是固定的整窗正式成绩，不是截至当前播放时刻的累计 RMSE。实时误差与底部曲线来自原生末端和同一时刻目标。用于显示的轨迹线作有限抽样，正式统计仍使用全部原生记录；机架与关节以记录状态插值显示，不进行新动力学积分。未记录的桨叶转动保持静止。
