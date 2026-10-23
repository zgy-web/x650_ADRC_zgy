# 冻结 C++ MPC 的普通及快速八字入口

本入口承接 `native_numerics_20261023` 的完整末端保持，保持原来的 C++ 动态库、预测模型、PID、MPC 权重、限制和时序门限。只变更实验阶段、目标轨迹、持续时间及其显式准入信息。

普通八字必须持有指定的完整末端保持证书。快速八字还必须持有本入口生成的普通八字完整证书；失败或只有部分时间段的记录不能解锁快速八字。

| 项目 | 普通 | 快速 |
|---|---:|---:|
| X 幅值 | ±0.25 m | ±0.25 m |
| Z 幅值 | ±0.10 m | ±0.10 m |
| 匀速段名义周期 | 12 s | 8 s |
| 圈数 | 2 | 2 |
| 起始保持 / 相位加减速参数 | 2 s / 2 s | 2 s / 2 s |
| 完整实验时间，包含末尾保持 | 32 s | 24 s |
| 计算及更新间隔上限 | 50 ms | 50 ms |

轨迹复用原 `FigureEight` 的 C2 相位规律。普通和快速的区别仅为参考轨迹的周期及相应实验时长。沿用原末端 RMS 50 mm、峰值 100 mm 和最大倾角 8°门限；审计同时检查完整声明区间和运动区间。实际误差按相同时刻独立 Gazebo 末端减去解析目标计算，不拟合位置原点或相位。

`ts20_protocol` 保留基线中描述原悬停准入的原始内容，新的 `figure8_protocol` 才是本入口的轨迹准入规则。控制律不会读取这个新增字段来改变参数。原有悬停入口仍只允许原来三个阶段。

## 文件

- `task.py`：以已验收任务和证书的固定 SHA256 建立基线；拒绝任何控制参数、后端或轨迹协议变更；校验每个前序证书的原始文件。
- `run_case.py`：复用原 TS20 执行器、因果状态源、C++ 控制器和收尾流程。轨迹激活使用原来的实际发送边界与指令历史。
- `launch.py`：从已验收启动器复制并限定为新的独立入口，保留执行源码归档、软件证明、运行前后源码比较和运行实例关闭检查。没有修改或猴子补丁原启动器。
- `audit.py`：复用独立 ULog、参数恢复、执行器和反馈检查；为完整轨迹生成不同类型的证书。

当前固定基线任务 SHA256：`f8d96cb8d70b55a9885b4723f5ea614d1978f4e50663921a2e9fe440b4357ae1`。

C++ 数值库 SHA256：`773241c1a3fe858f9d4b0e60f7f6b7465c6c0b9ef50b4ac1adac8f55590f7fef`。

基线完整末端保持证书 SHA256：`3a02609b58ef462590b76138131003fdeaed93aa6bea3528a3761e14e4564602`。

服务器已有任务文件下的只读起飞前检查：

```bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
export PYTHONPATH=/usr/lib/python3/dist-packages:/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
/usr/bin/python3 simulations/x650_ts20_figure8_v1/launch.py \
  --task artifacts/native_figure8_20261023/normal_task_v1.json \
  --software-proof artifacts/native_numerics_20261023/software_v4/software_acceptance.json \
  --software-sha256 f12af2fadef68e8650972d909b0c6c3f164954b2910b7a37ea57a8d893842430 \
  --dry-run
```

去掉 `--dry-run` 会启动一次新的完整仿真；实验使用独立目录，不覆盖历史记录。当前冻结版本在第一次普通八字中触发 52 ms 间隔保护，快速八字尚未获准执行，见[本轮实验报告](../../docs/validation/native_figure8_20261023/README.md)。
