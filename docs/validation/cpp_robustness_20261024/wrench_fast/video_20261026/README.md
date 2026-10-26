# 外扰失败实验视频回放

视频来源为 2026-10-24 的原始 `wrench_fast` 实验，2026-10-26 在新容器 `autodl-container-3187449845-10f5a45f` 离屏渲染。**没有重新运行物理实验，也没有继续失败后的 MPC 轨迹。**

![原始失败实验回放](X650_wrench_failed_original.mp4)

两个画面属于同一次实验：左侧固定世界视角，右侧跟随已记录机械臂的近景。黄色是预定完整八字，蓝色是原始实际末端历史。第 7 秒出现世界 +X 方向 1.5 N 加力标记；箭头为图示，数值来自原插件同一步 ECS wrench 指令增量，不代表测力传感器读数。

源任务时间与视频播放时间分开记录：

| 源任务时间 | 展示方式 |
| --- | --- |
| 0–7 s | 1 倍速 |
| 7–7.544 s | 0.25 倍速，突出第一次加力后的变化 |
| 精确 7.544 s 末态 | 定格 3 秒，任务时钟停止，标记 `QP INFEASIBLE` |

视频为 1600×1140、24 fps、293 帧，播放时长约 12.208 s。源物理记录只有 0–7.544 s、1887 个同一步样本；视频的慢放与定格不会增加物理样本或延长控制窗口。[display_frame_clock.csv](display_frame_clock.csv) 保留逐帧视频时间、源任务时间和播放阶段。

末态末端误差 49.778 mm，0–7.544 s 失败前缀 RMSE 9.188 mm，末态倾角 3.026°。第一脉冲原定 7.6 s 结束，控制提前退出，因此不展示撤扰恢复，不将之后的停止／落地样本计入 MPC 跟踪。这段视频不授予完整 24 s 验收。

原始 URDF 网格全部保留，实际机体位置与两关节角线性插值，四元数使用 SLERP；全体原生样本先通过 FK 核验。使用 MuJoCo 作为离屏绘图器，只做 `mj_forward`，不调用 `mj_step`。未记录的旋翼转角保持源模型零位。软件渲染关闭阴影、反射与多重采样，不减少模型网格。

[replay_manifest.json](replay_manifest.json) 保留原始数据、模型和脚本 SHA、失败边界、FK 审计及完整 FFmpeg 帧解码结果；[delivery_manifest.json](delivery_manifest.json) 保留输出文件身份。[local_video_verification.json](local_video_verification.json) 核对本地交付文件字节身份、293 帧时间表和暂停边界。

入口：[render_x650_wrench_failure.py](../../../../../scripts/render_x650_wrench_failure.py)。它使用独立失败 manifest，不修改正常／快速成功视频入口的证书要求。可用 `--preview-only` 先查看末态；不提供该选项则编码视频，`--output` 必须是新目录。实际渲染命令保存在 [render_video_v1.sh](render_video_v1.sh)。
