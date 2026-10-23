# 实际 PX4 / Gazebo 的隔离执行入口

本入口使用 X650 URDF、原二连杆、条件 TS20 plant 和冻结的上层 MPC。
它将真实反馈、算法工作进程、C++ 周期下发、实际消费回报和大诊断分开。
默认 shadow 工具不具有执行权限；只有本场景的显式 live 配置能使用本机 SITL socket 和关节桥。

**2026-10-24 状态：hold_v15 是原数值库/执行协议第 7 版的历史完整通过；普通八字首试未完成，
第 8 版 hold_v16 又暴露 QP 收敛问题。当前第 9 版数值修复候选尚待完整回归、新保持和普通八字，
不具有新的完整实验通过结论。**

## 输入和边界

- 历史原任务：`quiet_task_cpp_v5.json` 原字节 SHA-256
  `f8d96cb8d70b55a9885b4723f5ea614d1978f4e50663921a2e9fe440b4357ae1`。
- 原数值库 SHA-256：
  `773241c1a3fe858f9d4b0e60f7f6b7465c6c0b9ef50b4ac1adac8f55590f7fef`。
- 第 9 版候选库 SHA-256：
  `b9cbdf78c76e92f973800e07c6219611759fcb4471a1a39192c5cbf30f5e2e48`。
  经用户授权仅修正 QP 数值收敛；模型、代价、限位、PID/MPC 参数和外层 `2e-7` 门限保持不变。
  新库必须绑定新的任务及资格证明，历史任务哈希和 hold_v15 不能替代新库准入。
- 状态：原 PX4 EV 估计接口、实际关节 q、条件舵机末端角速度、实际 MIT 包和消费历史。
  独立 Gazebo 末端真值用于验收。
- 输出：现有 PX4 MAVLink LOCAL_NED PVA、固定 yaw、mask 2048、零加速度前馈；
  ROS–Gazebo 关节速度桥。未迁移到飞控原生 DDS 输出。
- 20 ms 周期、50 ms 指令原始年龄和 25 ms 来源年龄门限保持不变。
  模型更新的 20 ms 使用实际仿真源时钟，输出节拍及新鲜度使用同机 monotonic 墙钟；
  两种时钟不能互相替代或叠加为两个最小更新间隔。

20 ms 是定周期下发；50 ms 是从原快照开始的指令寿命，也是保留的完整计算/模型 dt 上限。
一次计算不足 50 ms 仍可能错过可用输出槽，重复旧指令不会续期。失效由下发 tick 检测，
并非恰在第 50 ms 异步硬停；正常 PVA 停发后 PX4 暂时保持最后目标，随后由原恢复流程接管。

## 接管与退出

起飞前构造算法对象并验证原数值证明。构造模板不运行优化；起飞及预流后，
一次性 `LiveStart` 注入本次真实参考、状态基准和已发送指令历史。
全部 DDS 端点匹配后才转移输出所有权。旧入口负责起飞与恢复，活动段只有 C++ 执行端能写指令。

下一轮计算以**上一候选的实际发送通知**为前提，使用已收到的最新有效真实快照，
无需等待发送之后再产生一份样本。所有来源原时间戳及年龄门限不变。下一次执行器下发以
**两关节上一条指令均已实际消费**为前提。两种确认分别记录：既不把候选被接受
当作已执行，也不让优化器串行等待物理消费回执后才开始采样。
`consumption_source` 独立读取插件完整物理步记录，末端预测继续使用原冻结的因果状态适配器。

源反馈/ROS 消息和算法大诊断使用两个独立记录进程。接收端只保留各类最新消息，
不累积复杂对象；二进制非阻塞复制的次数、丢弃、持久化序号及哈希均在降落后核对。
记录丢失使本次试验不合格，不会阻塞控制等待落盘。

执行协议第 8 版在 worker 构造及 executor 注册完成后、READY 和起飞之前显式执行一次
`gc.collect(2)`，并保存 `worker_preflight_gc.json`。构造调用栈此时已返回，临时对象可被回收。
控制阶段仍保留原自动 GC、原阈值及回调；没有禁用或冻结 GC，也没有用预飞试算改变优化器状态。
这是对首次普通八字中 11.53 ms gen2 停顿的执行层修正，必须以新版完整保持及普通轨迹重新验收，
不能视为未来停顿已被保证消除。

第 8 版 hold_v16 在约 28.38 s、第 1416 次求解时被原始行可行性检查拒绝，1415 次已接受。
原违反量 `3.270557920181938e-7` 经范数约 `4.98172` 归一化后约为 `6.565e-8`，
低于旧求解器聚合终止阈值 `7.6189e-8`，因此求解器提前返回，而外层 `2e-7` 检查正确拒绝。
第 9 版增加原始每行 `1e-10` 的内部收敛要求，同步变换为 `1e-10 / row_norm`，
并保留聚合检查、迭代上限、全行 KKT 和非线性候选检查。非有限残差或未达收敛要求均抛错。
故障输入复算已得到零违反量、独立复核 KKT 约 `2.78e-17`，16 项 native 测试含 12 个子场景通过；
这些针对性结果不能替代完整数值资格化和新的真实仿真窗口。

执行资源由协议中的 `cpu_roles` 显式声明，起飞前验证本机可用性，接管前核查每个线程。
当前服务器选择的 CPU 编号不能直接用于其他主机；只限制本次自有仿真进程，
让后台仿真/记录避开关键角色及其 SMT 配对，不修改宿主机调度策略。

停止时发送一次关节零速并核对实际消费，然后终止新写入者，恢复原调度、收臂、降落、
确认上锁并恢复参数。无消费证据或恢复不完整均不能签发通过结果。
历史 hold_v15 的正常退出已取得实际证据；若此前还有未确认指令，消费观察直接跨过普通指令到停止计数的
边界仍需单独故障测试。该边界也可能出现在末条普通命令仍未确认的正常任务结束时。
此时程序会终止新写者并恢复，但缺失的零速回执必须使验收失败；这不构成真机停止/故障退出资格。

## 运行与审计

先完成当前数值库的完整资格化并绑定新的任务，再构建 ROS 2 工作区。`MAVLINK_INCLUDE_DIR` 指向现有 PX4 build 中的生成头文件，
无须重新安装 Node 或下载另一个仿真平台。使用独立 build/install 目录，避免覆盖历史构建。

```bash
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/ros2_px4_ws/install/setup.bash
source /root/autodl-tmp/flyarm_ros2_live_v1_20261023/install/setup.bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
export PYTHONPATH=/usr/lib/python3/dist-packages:/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages:$PYTHONPATH
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
# 指向当前完整数值资格证明绑定的任务；第 9 版不能直接沿用 quiet_task_cpp_v5.json。
# REVIEWED_TASK=/absolute/path/to/current_qualified_task.json

/usr/bin/python3 scripts/probe_x650_live_transport.py --output artifacts/NEW_transport_run
/usr/bin/python3 simulations/x650_isolated_live_v1/launch.py \
  --task "${REVIEWED_TASK:?Set the current qualified task path first}" \
  --transport-proof artifacts/NEW_transport_run/report.json --dry-run
```

预检成功后去掉 `--dry-run` 执行。本次启动目录和实际运行目录会新建并保留全部失败；
运行前后核验来源 ZIP、任务、模型和实际二进制身份。正在运行的实验不允许修改其绑定文件。

```bash
/usr/bin/python3 scripts/audit_x650_isolated_live.py \
  --launch-dir runs/ACTUAL_LAUNCH --run-dir runs/ACTUAL_RUN \
  --output artifacts/NEW_complete_hold_audit
```

完整 30 s 记录、原时轴末端精度、实际关节消费、PX4 全窗 ULog、参数恢复、
独立关节自由预测以及逐输入冻结核心复算均通过，才能生成完整保持结果。
短时失败可用 `audit_x650_live_commissioning.py` 分项审计，但其 `hold_accepted` 永远为 `false`。
软件传输 probe 使用合成消费器，不能冒充实际关节实验。

当前执行入口首先验收保持；没有新执行路径的完整保持通过证据时，不允许开启普通八字。
更换数值库或执行协议后，这个保持门槛必须重新获得，历史 hold_v15 的通过不会跨版本转授。
快速八字需另等普通八字通过。每次结果的具体状态见
[实际接入记录](../../docs/validation/isolated_live_20261023/README.md)。

真机迁移沿用输入/输出/消费/停止契约与审计流程，替换具体传感器和总线适配器并重新验收。
服务器条件模型不能证明 TS20-50 在 24 V 下的真机响应，x86 耗时也不能代表 Orin NX。
