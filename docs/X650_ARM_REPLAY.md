# X650 双连杆实测状态回放

`scripts/render_x650_arm_replay.py` 将真实 Gazebo 记录的机体位置、姿态和机械臂关节角画成视频。MuJoCo 只负责离屏渲染；没有推进动力学、生成新的飞行轨迹或添加未记录的桨叶转动。

完整保留 v2 组合 URDF 的 281 个原始 STL、370,110 个三角面，以及安装适配件和两个机械臂长方体。每个 URDF 链接对应独立 mocap 渲染体；每帧由记录的根姿态和两个实测关节角，通过原 URDF 关节树计算各链接的世界姿态。由此避免把机械臂固定在 q=0 或只用末端轨迹制作动画。

## 输入

`--source` 必须指向 `prepare_x650_arm.py` 生成的 **v2 组合源 URDF**，不是未经转换的原 CAD，也不是 Gazebo 的 SDF。旁边必须保留准备报告、机架间隙证书、自碰撞证书、运动学参数和 STL 资源。脚本验证其哈希、几何检查状态及 `elbow_offset_body_m=[0,0.04,0]`。

CSV 必需字段：

```text
sim_time_s, phase,
world_x_m, world_y_m, world_z_m,
world_qx, world_qy, world_qz, world_qw,
q1_rad, q2_rad
```

位置属于 Gazebo 世界 ENU，四元数是机体 FLU 到世界的主动旋转，字段次序为 ROS `xyzw`。关节角为 Gazebo 实测弧度，`q=[0,0]` 沿用原 150°／-150° 折叠零点。不要把物理绝对杆角直接填入 `q1_rad/q2_rad`。

可选末端实际位置：

```text
tip_world_x_m, tip_world_y_m, tip_world_z_m
```

只要提供其中一列就必须提供三列。每个样本都与机体姿态和实测关节 FK 比较，超出默认 **0.1 mm** 容差会停止渲染。即使没有这些列，也逐样本比较准备报告的解析运动学与实际 URDF 关节树，误差必须小于 1e-10 m。绘制的末端位置由实际机体与关节状态计算，不用末端记录反推机体或关节动作。

可选参考位置：

```text
tip_target_world_x_m, tip_target_world_y_m, tip_target_world_z_m
```

起降阶段没有末端参考时填三个 `NaN`。跟踪误差指标仅统计 `phase` 为 `measure`、`track`、`tracking` 或 `figure8` 且参考完整有效的样本，不自动对齐位置、相位或时间。

真实结果还要求已完成的 `flight_samples_manifest.json`，默认与 CSV 同目录，或者通过 `--capture-manifest` 显式指定：

```json
{
  "complete": true,
  "file": "flight_samples.csv",
  "sha256": "完整 CSV 的 SHA256",
  "model_sha256": "这次 Gazebo 实际 SDF 的 SHA256",
  "source_urdf_sha256": "本次组合源 URDF 的 SHA256"
}
```

模型身份由最终 manifest 的实际 SDF 哈希与完整 CSV 哈希绑定。如果 CSV 有 `model_sha256` 列，则每一行还必须与 manifest 一致；当前 ROS `FlightRecorder` 不逐行重复此固定值，允许只由最终 manifest 绑定。未完成、文件变动、不同源 URDF 或合成记录会被拒绝用于正式视频。ROS 记录器在无参考阶段写出的三列空白会仅作为可选参考解析为三个 `NaN`；实际位置、四元数、关节角和可选实际末端列不接受空白。

## 操作

先审核 CSV 与运动学，不需要 MuJoCo：

```bash
python3 scripts/render_x650_arm_replay.py \
  --source generated/x650_2link_v2_20261021/robot.urdf \
  --csv runs/REAL_RUN/flight_samples.csv \
  --output runs/REAL_RUN/replay_audit --audit-only
```

服务器已有 `amsim-venv` 的 MuJoCo 3.15、NumPy 2.2.6、Matplotlib 3.10.9；`flyarm-video-venv` 没有 MuJoCo。使用前者的同一 Python 解释器，另需其已有 Pillow／PyYAML，设置 `MUJOCO_GL=egl` 后先输出一帧预览：

```bash
MUJOCO_GL=egl /root/autodl-tmp/amsim-venv/bin/python scripts/render_x650_arm_replay.py \
  --source generated/x650_2link_v2_20261021/robot.urdf \
  --csv runs/REAL_RUN/flight_samples.csv \
  --output runs/REAL_RUN/replay_preview --preview-only
```

确认几何和画面后，选择新的输出目录，不加 `--preview-only` 即可生成 1440×900、24 fps、**1 倍仿真时间** 的 MP4。使用已有 FFmpeg，可通过 `--ffmpeg` 指定；脚本不下载渲染依赖。视频编码后会完整解码校验。

输出包括 `x650_arm.mp4`、轨迹与误差 PNG/PDF、五张抽样帧、`fk_audit.json` 和 `replay_manifest.json`。主视图与细节窗来自同一时刻的同一组记录；图表展示末端 X/Z、三维跟踪误差和实测关节角。参考金色十字使用记录中的末端目标。

关节 commissioning 使用专用 `--joint-probe` 模式，避免在没有末端目标的实验中显示空白末端误差：

```bash
MUJOCO_GL=egl /root/autodl-tmp/amsim-venv/bin/python scripts/render_x650_arm_replay.py \
  --source generated/x650_2link_v2_20261021/robot.urdf \
  --csv runs/JOINT_RUN/flight_samples.csv \
  --output runs/JOINT_RUN/replay_joint_preview --joint-probe --preview-only
```

该模式读取 CSV 同目录的完整 `controller_metadata.json`（也可用 `--controller-metadata` 指定），核对模型身份和任务窗口，复用独立分析器的原 `sin^3` 关节位置参考。只有实际 `measure` 阶段有参考，其他阶段留空。右侧依次显示两关节实际／参考、关节角跟踪误差、机身相对固定 hover anchor 的 XYZ 位移，状态栏显示实际倾角。不生成末端目标或把机身目标当作末端目标。图表来自分别带时间戳的真实 ROS 记录；正式主指标仍由原生同物理步分析给出。

参考使用 flight CSV 的 `sim_time_s`（Odometry truth header 时间），实际 q 带有独立 `joint_stamp_s`。每次回放应报告这两个消息时间戳的实测差异，不能用时差估计扣除或对齐误差。视频曲线用于观察过程，正式指标应由 [原生关节分析器](../scripts/analyze_x650_joint_probe.py) 对该次独立物理记录计算。私有 URDF/STL、原始 CSV、报告和视频均为外部输入或本地生成结果，不随本源码提供。

主相机每帧跟随实际机体位置／姿态，距离 2.3 m；近景对准实际肩、肘和末端的中心，距离 0.80 m。两幅图在同一次 `mj_forward` 后更新，保留原网格与真实臂姿态。固定合理的场景 extent 和裁剪距离，未提供末端参考时通过透明度隐藏 marker，避免用 z=-100 的远点隐藏方式扩大场景范围并裁掉近景。预览默认选择实测关节幅度最大的一帧，也可用 `--preview-time-s` 指定记录相对时间；完整视频仍覆盖原始记录并保持 1 倍时间。

机体平移与关节角采用分段线性重采样，四元数采用最短路径 SLERP，不平滑轨迹。采样间隔大于 0.25 s 时，机体和两个关节共同保持前一个实测状态，并在画面标出遥测缺口。

## 代码验证与限制

```powershell
python -m unittest discover -s tests/unit -p test_x650_arm_replay.py -v
```

九项测试使用明确标记的 **SYNTHETIC** 数据验证四元数（含 180°）、逐样本 FK 与错层参数、源证据哈希、完整可动 MJCF 与裁剪设置、遥测缺口保持策略、最终记录身份、真实 ROS 记录器的空白参考和 manifest 单一身份绑定约定、相机随实际平移／旋转移动，以及关节参考仅出现在对应测量窗口。

合成输入只允许 `--synthetic-test --audit-only` 或 `--synthetic-test --preview-only`，预览显著标记 **SYNTHETIC CODE TEST — NOT AN EXPERIMENT**，不能导出实验视频。`generated/x650_arm_renderer_SYNTHETIC_codecheck` 是本地纯 XML/FK 测试文件，不能当成实验结果。

截至本次审查，已验证纯 XML 场景包含全部 281 STL／370,110 三角面、3 个新增实体、7 个可动链接。服务器已完成原生折叠悬停 `115937_5063` 和小幅关节检查 `120236_5925`；关节完整成功 CSV 已本地归档并独立分析、验图。根任务已验证 v2 EGL 预览的主视机体和臂近景清楚，并启动完整视频，视频下载与编码检查由根任务归档。此审查 agent 没有操作服务器渲染。早期两次中止悬停的未完成 manifest 保留为失败历史，不替代成功运行。地面稳定性窗口单独绘图，不能当作飞行记录。
