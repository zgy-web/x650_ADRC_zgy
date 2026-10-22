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
