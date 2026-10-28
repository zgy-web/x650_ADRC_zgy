# NX 完整 C++ 接口离线回放

2026-10-28，已在实体 Orin NX 上通过 1,281 个历史快照的完整接口回放。这里的“完整”指状态消息类型、坐标适配、原 MPC、指令编码及本地提交事务这条软件路径；没有向真实飞控发送设定值，也没有重新运行连续物理闭环。逐项结果见[本次验收记录](validation/nx_full_interface_20261028/README.md)。

## 先沿着这一条路径看代码

```text
既有完整实验窗口的 State / Memory / input / oracle
  ↓ 离线准备：保留原记录，构造 PX4 float32 与 JointState 消息字段
C++ typed message → ROS CDR 序列化 → CDR 反序列化
  ↓ Px4BodyReceiver：与硬件诊断程序共用的消息接收逻辑
Px4ReplayAdapter：NED/FRD → ENU/FLU，关节按名称映射，时钟与有效性检查
  ↓ Controller.prepare：原 ABI 4 矩阵组装、QP、候选检查
StepResult：飞机 PVA + 两关节速度
  ↓ encode_mpc_output：ENU → NED、float32、NaN 字段
TrajectorySetpoint → CDR 往返 → 独立逐字段核对
  ↓ 仅模拟本地发布成功
commit_published → 与同输入直接 Core 的 Memory 核对
```

| 阅读顺序 | 文件 | 重点 |
|---|---|---|
| 1 | [输入准备](../scripts/prepare_x650_hardware_replay.py) | 从哈希绑定的真实仿真快照重建消息；独立矩阵变换作为预期值 |
| 2 | [共用消息接收](../ros2_ws/src/flyarm_hardware_state/include/flyarm_hardware_state/px4_message_adapter.hpp) | 姿态、位置、速度、加速度、角速度、飞控状态和重置计数；不创建 ROS 节点 |
| 3 | [MPC 输入与输出适配](../control_packages/x650_mpc_controller_v1/include/x650_mpc/px4_replay_adapter.hpp) | 机体坐标转换、关节身份、采样时间检查、PVA 输出映射 |
| 4 | [可调用控制接口](../control_packages/x650_mpc_controller_v1/include/x650_mpc/controller.hpp) | prepare / commit / discard；求解成功不等于已执行 |
| 5 | [全链回放程序](../control_packages/x650_mpc_controller_v1/examples/hardware_replay.cpp) | 原历史输出、同输入直调 Core、消息字段、提交前后状态分别比较 |
| 6 | [适配器故障测试](../control_packages/x650_mpc_controller_v1/tests/test_px4_adapter.cpp) | 无效定位、陈旧/未来时间、估计器重置、关节错名、NaN 与重复前馈 |
| 7 | [运行器](../scripts/run_nx_full_interface_replay.py) / [NX 内存任务](../scripts/nx_full_interface_replay_remote.py) | 编译与测试只用 NX RAM，结果回到电脑 |
| 8 | [验收器](../scripts/summarize_nx_full_interface_replay.py) | 完整批次、每行结果、输入/库身份、退出与清理证据缺失均拒绝通过 |

原硬件诊断程序 [px4_link_probe.cpp](../ros2_ws/src/flyarm_hardware_state/src/px4_link_probe.cpp) 已在源码中改用相同 `Px4BodyReceiver`。本次仅在 RAM 中编译并运行模拟测试，没有替换 NX 用户目录里之前安装的诊断程序。

## 哪些量要特别看

- 世界系位置/速度/加速度：NED 到 ENU 为 `[y,x,-z]`；机体角速度 FRD 到 FLU 为 `[x,-y,-z]`。姿态需同时变换世界系和机体系。
- `VehicleLocalPosition.ax/ay/az` 是世界系速度导数；不是原始加速度计比力，不能再加减重力。
- 真实路径沿用 PX4 本地原点，忽略仿真 `ref_alt` 偏置。测试故意给非零 `ref_alt=123.5` 检查这点。
- 两关节按显式名称对应，单位 rad / rad/s。测试故意反转消息中的两个关节排列；目前输入为重建的 JointState，不是已验收的 TS20 实机驱动反馈。
- `velocity_feedforward` 已是最终飞机速度字段，不能再加 `command[0:3]`；两关节输出直接对应 `command[3:5]`，不再补一层位置 P。
- jerk、yawspeed 未使用时保持 NaN；JSON 证据用 null 表示 NaN，并带明确标记，实际 ROS 消息仍为 NaN。
- 时间新鲜度先用整数纳秒比较，再转换为 Core 的秒值。用统一的除法转换避免同一物理时刻因浮点舍入被误报为未来；不是放宽截止时间。

`Px4ReplayAdapter` 保留未解锁台架保护；它不是可以直接接管飞行的在线下发器。接收时间、采样时间与控制时刻的时钟关系必须由未来实机采集接口建立，本回放用明确标注的夹具模拟它们。

## 判据要分成三类

1. **原输入、原库与历史输出**：沿用原容差，位置参考 1e-9、其余已记录输出 1e-7；不改原 oracle。
2. **同一份适配后输入，封装与直接 Core**：输出、矩阵、QP 结果、计划、候选包络及提交后的 Memory 必须完全一致。消息编码与 CDR 往返核对实际 float32 字段、时间戳和 NaN 掩码。
3. **float32 输入相对原 double 输入的变化**：单独记录各轴指令差，不能声称仍是同一输入。此次最大飞机速度变化 2.6871e-7 m/s，最大关节速度变化 4.3280e-7 rad/s；原 double 输入历史验收仍单独通过。

此处不是性能压测，不输出实时性认证。每行恢复该时刻真实保存的 Memory，不是让控制器沿新的量化反馈连续推进。

## 为什么绑定头文件和库

仓库当前原生 `mpc_core.hpp` 与已验证 NX 库对应的历史头文件存在结构变化，但都声明 ABI 4。只看 ABI 数字不足以保证 C++ 对象布局匹配。

回放使用[保存的历史头文件](../control_packages/x650_mpc_controller_v1/compat/abi4_20261027/mpc_core.hpp)，SHA-256 为 `3ce12c395eb047167b5eda8fefb502fe56622e610cdfd3e82a4302f4a53c0093`。CMake 在开启硬件回放选项时强制核对该头文件；运行器校验三库内容哈希，回放程序另记录实际加载路径和哈希。它不覆盖当前开发头文件，也不重新编译原算法库。

这一测试绑定的是历史 `Controller` 与 ABI 4 后端，不包含后续可选 `CommonGateController` 或 NMPC。若更换算法版本，必须连同头文件、库、输入及预期结果建立新的版本记录。

## 在电脑上复现

从仓库根目录执行。需要已有历史输入文件、NX 上已验证的三库及 ROS 2 / px4_msgs / sensor_msgs；源码仓库不包含历史原始数据和预编译库。`--output` 必须是新目录，避免覆盖之前的失败证据。

```powershell
python scripts/prepare_x650_hardware_replay.py --output artifacts/interface_inputs_new
python scripts/run_nx_full_interface_replay.py --inputs artifacts/interface_inputs_new --output artifacts/interface_run_new --host 192.168.30.145 --ssh C:/Windows/System32/OpenSSH/ssh.exe --identity C:/Users/USER/.ssh/id_ed25519 --known-hosts artifacts/NX_KNOWN_HOSTS --library-report artifacts/orin_pil_20261027/nx_no_fma_pil_build_report.json
python scripts/summarize_nx_full_interface_replay.py --run artifacts/interface_run_new --inputs artifacts/interface_inputs_new --output artifacts/interface_summary_new.json
```

`prepare` 只在电脑准备输入。运行器将源代码与输入传入 `/dev/shm/flyarm_full_replay_*`，复用已有依赖，关闭 core dump，所有编译、日志、测试输入均置于内存临时目录。结束回传结果并删除临时目录；不执行安装，不改变 NX 配置，不启动 Agent，不打开 USB。原三库执行前后哈希保持一致。异常中断时以清理证据是否完整为准，不能将没有结尾的运行判为通过。

单独构建 C++ 程序时，在已有 ROS 环境下开启 `-DX650_MPC_BUILD_HARDWARE_REPLAY=ON`，将 `X650_NATIVE_INCLUDE_DIR` 指向上述 `compat/abi4_20261027`，并显式提供匹配的库路径。默认普通控制包构建不增加 ROS 依赖。

## 下一层验证

离线回放通过后，用[飞控内部设定值回读方案](NX_PX4_SETPOINT_READBACK.md)检查真实 USB/DDS 接收。它不要求先恢复动捕。真实位置闭环、关节总线消费、在线时钟、接管与退出仍要在各自前提具备后验收，不能由本次离线结果替代。
