# C++ 普通八字独立入口

本场景沿用已验收的完整 C++ 反馈、MPC 与连续输出结构，只新增普通八字参考。X 幅值 ±0.25 m、Z 幅值 ±0.10 m，12 s/圈、2 圈；2 s 起始保持、原 C2 相位加减速，运动于 28 s 结束，完整记录至 32 s。单组 MPC，不追加 PID 对比，不开放快速八字。

`task.build_task_config(hold_binding, certificate_binding)` 从通过完整后验的 30 s C++ 保持 task 重建普通 task。两个 binding 都为原文件绝对 `path` 与 `sha256`，控制参数、物理模型、定位和 MIT 参数逐项保留。新 qualification manifest 另外绑定本轮保持证书、动态数值证明和实际动态 worker/source 软件证明；旧保持 worker 的历史证明不能替代新二进制证明。

启动入口为：

```bash
python3 simulations/x650_cpp_normal_v1/launch.py \
  --task /absolute/qualified_normal/task.json \
  --startup-manifest /absolute/qualified_normal/startup_manifest.json \
  --startup-sha256 <exact-startup-sha256> \
  --qualification-manifest /absolute/qualified_normal/qualification_manifest.json \
  --dry-run
```

先完成地面资格和实际 15 类消息 serdes 检查，再由相同入口去掉 `--dry-run` 启动独立场景。缺证据会拒绝，入口不会自动重试或跳过保持阶段。完整运行之后仍必须使用普通八字独立后验，不能把 launch 成功当性能通过。

现有 hold 场景保持原默认。普通子类只指定严格参考、原始保持配置和新的阶段门；SourceBinding、LiveStart、实际发送屏障、异步消费、零指令释放、参数恢复和落地流程全部复用。Python 只负责准备/监督/恢复；ACTIVE 的状态构造、轨迹预览和求解由 C++ 完成。
