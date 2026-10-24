# C++ 快速八字独立入口

该场景严格承接已经完整通过的C++普通八字。X±0.25 m、Z±0.10 m，8 s/圈、2圈；2 s起始保持、原C2平滑起停、4 s末尾保持，共24 s。运动窗2–20 s。普通12 s工况保持原入口，不能混用。

`task.build_task_config(normal_binding, certificate_binding)` 绑定完整普通task与物理证书，逐项保留原控制／物理配置。资格清单另绑定新快速reference、数值对照及实际新ROS worker证明；历史普通worker的证明不能替代新二进制证明。

```bash
python3 simulations/x650_cpp_fast_v1/launch.py \
  --task /absolute/qualified_fast/task.json \
  --startup-manifest /absolute/qualified_fast/startup_manifest.json \
  --startup-sha256 <exact-startup-sha256> \
  --qualification-manifest /absolute/qualified_fast/qualification_manifest.json \
  --protocol /absolute/qualified_fast/protocol.json \
  --dry-run
```

通过地面准入后，用同一输入去掉 `--dry-run` 开始独立仿真。实际反馈、求解、定周期执行、异步消费监督、停止零命令确认与降落恢复沿用既有C++链路；Python承担准备、监督与恢复。结束后仍需 `scripts/audit_x650_cpp_fast.py` 核对完整24 s原始物理记录。

[本次完整结果、视频与原始证据](../../docs/validation/cpp_fast_20261024/README.md)。这是上层协同MPC的仿真验证，Orin与真机条件另行验收。
