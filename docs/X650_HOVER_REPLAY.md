# X650 悬停记录回放

`scripts/render_x650_hover.py` 使用 X650 原始 URDF 的全部 visual 网格，并以
`simulations/x650_hover_v1/run_case.py` 输出的 `flight_samples.csv` 中的真实 Gazebo
世界位姿驱动。MuJoCo 仅作为离屏渲染器，脚本不再模拟动力学，不生成或修饰飞行轨迹。

## 输入与坐标

- CSV 使用原始 Gazebo Odometry 消息时间戳 `sim_time_s`，必须严格递增。
- 同目录必须存在采集结束后写出的 `flight_samples_manifest.json`，CSV 哈希和全部
  行的模型哈希必须一致；尚未完成的记录不能直接发布成回放视频。
- 位置为 `world_x_m/world_y_m/world_z_m`，四元数原始字段为 xyzw；内部仅重排为
  MuJoCo 的 wxyz 顺序。四元数表示 base_link 到世界的主动旋转。
- 对原始 X650 URDF，默认先施加 `root_rpy=(π/2,0,0)`、
  `root_xyz=(0,0,-0.17095298)`。如果传入已经转换到 Z 向上的派生 URDF，必须显式传入
  `--root-xyz 0 0 0 --root-rpy 0 0 0`，避免重复变换。
- `target_world_*` 是固定悬停目标，不能当成 PX4 内部起飞或降落的动态设定值。
  图中参考线和误差只显示在 settle/measure/post 悬停阶段；统计仅使用 measure 阶段。

## 服务器命令

复用已安装的 `/root/autodl-tmp/amsim-venv` 中 MuJoCo、NumPy、Matplotlib、Pillow，
以及 `/root/autodl-tmp/flyarm-video-venv` 中现成的 ffmpeg，无须下载依赖。

```bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
MUJOCO_GL=egl /root/autodl-tmp/amsim-venv/bin/python3 scripts/render_x650_hover.py \
  --source /root/autodl-tmp/x650_urdf_source_20261017/robot.urdf \
  --csv /absolute/path/to/flight_samples.csv \
  --output /absolute/path/to/replay \
  --fps 24
```

首次可加 `--preview-only` 检查悬停中间时刻的版式，检查后再运行完整视频。
脚本默认拒绝覆盖已有视频；明确需要重做时使用 `--overwrite`。

## 输出和真实性边界

- `x650_hover.mp4`：1440×900、24 fps 默认、仿真时间 1×。画面包含全程视角、同一
  位姿的机体放大视角、实际高度与固定悬停参考、悬停位置误差、横滚与俯仰。
- `trajectory_and_errors.png/.pdf`：完整真实位置、悬停阶段参考与误差、机体姿态。
  绿色阴影是 measure 统计窗口。
- `replay_metrics.json`：仅 measure 阶段的样本数、时长、三维位置 RMSE/峰值、
  各轴 RMSE、最大横滚/俯仰。直接从记录计算，不使用插值后的视频帧计算指标。
- `replay_manifest.json`：源 CSV、URDF、脚本和视频的 SHA-256，模型哈希、完整网格
  数量、坐标变换、时长、插值说明与最大采样间隔。
- `frame_*.png`：五个时间点的审核帧；`preview.png` 仅在预览模式输出。

视频按原始时间轴逐段线性插值位置、对四元数做最短路径 SLERP，不做平滑、轨迹
对齐或重新定时。若样本间隔超过 0.25 s，该区间保持上一条记录的姿态并显示
`TELEMETRY GAP`，不跨越缺失记录编造连续运动。

全部 281 个 visual 的 370,110 个原始三角面均保留，碰撞模型不重复绘制。没有旋翼角度
记录，因此桨叶保持网格零位，不添加假想转速。地面、金色悬停目标十字和图表是展示
元素，不改变 Gazebo 中的运动。画面始终标注 `GAZEBO RECORDED-STATE REPLAY`，
不可称为 Gazebo 原生录屏或实机视频。

## 检查

```bash
python -m unittest discover -s tests/unit -p test_x650_hover_replay.py -v
```

测试覆盖真实时间保留、时钟不递增拒绝、四元数顺序与 SLERP、四元数双覆盖、无效
姿态拒绝、完成态文件哈希以及遥测缺口保持。原始 URDF、全部 mesh 和已完成飞行的
CSV/manifest 需由使用者提供；历史实验成绩、原始记录与视频单独归档，不随源码发布。
合成单元测试通过不能代替某次飞行或回放的真实记录校验。
