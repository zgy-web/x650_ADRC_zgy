# X650 上层全身 MPC：可调用 C++ 控制接口 v1

本包将已在 X650＋二连杆＋TS20-50/24 V 仿真中使用的 **ABI 4 MPC 核心**整理成独立 C++ 调用接口。它继续调用已有的矩阵组装、QP、恢复分支、节点间硬约束和 TS20 候选预测库；本次封装不重新整定模型、权重、限位或数值门槛。

控制器输出 **3 个飞机速度控制量＋2 个关节速度控制量**，同时给出飞机位置、速度、加速度参考。底层姿态/速度环由 PX4 执行，关节由现有 TS20 接口执行。它是上层全身协同 MPC；不是直接输出四旋翼推力和两关节力矩的全动力学 NMPC。

公共头文件为 [`include/x650_mpc/controller.hpp`](include/x650_mpc/controller.hpp)，当前仿真配置工厂为 [`include/x650_mpc/selected_profile.hpp`](include/x650_mpc/selected_profile.hpp)。本包不依赖 ROS 或 Python 执行控制计算。Python 示例只把既有记录转换成离线回放输入。

## 构建与运行

从完整仓库根目录，在已有 Linux 仿真环境中运行；复用已验证的三个共享库，**不编译、不替换原算法库**：

```bash
R=/root/autodl-tmp/safe-aerial-manipulator-testbed
M=$R/artifacts/cpp_multirate_20261026
B=$R/artifacts/nmpc_interface_20261027/mpc_api_build
cmake -S "$R/control_packages/x650_mpc_controller_v1" -B "$B" \
  -DCMAKE_BUILD_TYPE=Release \
  -DX650_BACKEND_LIBRARY_DIR="$M/build" \
  -DX650_TS20_LIBRARY=/root/autodl-tmp/libts20_prediction_v3.so
cmake --build "$B" -j2
ctest --test-dir "$B" --output-on-failure
"$B/x650_mpc_one_step"
```

需要 C++17、CMake ≥3.16。离线 JSON 回放示例另外使用现成的 JsonCpp/OpenSSL；只要基础 C++ 调用时可设置 `-DX650_MPC_BUILD_REPLAY=OFF`。将本目录 `add_subdirectory` 到自己的 CMake 工程，然后链接 `x650_mpc::controller`。调用者自行传入 TS20 预测函数指针，并保证对应共享库在控制器整个生命周期内保持加载。

`one_step` 是可执行的合成接口示例，没有启动物理仿真，也没有发送执行器指令；它因此在计算后调用 `discard`。示例的合成几何参数不应直接用于 X650 飞行。实际选定配置使用 `selected_x650_simulation_profile()`，或传入由对应实验清单验证过的完整 `x650::Config`。

## 在应用程序中调用

```cpp
#include "x650_mpc/controller.hpp"
#include "x650_mpc/selected_profile.hpp"

x650_mpc::Controller controller(
    x650_mpc::selected_x650_simulation_profile(), predictor_function);
controller.reset(actual_anchor, actual_pre_task_publications, task_origin_s);

// input 必须包含新测量、正确时钟、实际发送历史和本轮参考预览。
const auto& candidate = controller.prepare(input);
const auto token = candidate.token;
const auto command = candidate.output.command;

// 由应用执行既有的源新鲜度、候选有效期、编码与输出监控。
// publish_both() 代表你的执行适配器，不是本包提供的函数。
if (publish_both(candidate.output)) {
    controller.commit_published(token, command);
} else {
    controller.discard(token);
    // 若已部分发出或实际发送结果未知，应用应退出任务并核实状态，
    // 不得把 discard 当成“撤回已经发出的物理指令”。
}
```

| API | 约定 |
|---|---|
| 构造函数 `Controller(Config, predictor, mode)` | 检查原 core ABI 4 / optimizer ABI 2；每架飞机、每次独立任务一个实例 |
| `reset(anchor, actual_history, origin)` | 使用真实预发送历史建立初始内存；清除已有候选与上一任务计划 |
| `prepare(StepInput)` | 在私有 Core 副本计算，返回只读候选及递增 token；已提交状态保持不变 |
| `commit_published(token, exact_command)` | 两侧完整指令已由下发器成功发布后，提交该候选的原生 Memory；匹配 token 与未编码五维命令 |
| `discard(token)` | 候选未采用时丢弃，不增加步数、不记录为已发送指令 |
| `committed_memory()` / `config()` | 只读查询；可用于记录与离线比对 |
| `restore_for_replay(memory)` | 明确的离线历史回放入口；不能用来伪造在线执行历史或跳过重新接管 |

`prepare` 返回的引用在提交、丢弃、重置或回放恢复后失效。一个实例同一时刻只保留一个待处理候选；先明确处理该候选，再准备下一个。调用方需要串行访问实例；库内部不创建线程或锁，也不执行磁盘日志。

**这里的提交是一次本地应用事务，不是每周期等待一次关节消费确认。** 下发器已完成成功发布即可提交，随后继续计算；消费编号和实际执行反馈仍由异步执行监控维护，下一轮通过实际历史进入预测。重复下发同一候选也不应重复调用 `commit_published`，重复发布时间应记录到执行适配器的实际历史。

当候选未发送时，下一次 `input.dt` 应反映与**上一条已提交控制步**之间的真实时间；不得通过改小 `dt` 或伪造时间戳掩盖外层停顿。若该间隔超出原核心允许范围，应进入应用现有的退出流程。

## 输入、输出与时间

沿用原 `mpc_core.hpp` 的类型，完整字段以头文件为准：

- `State`：世界 ENU 位置/线速度/加速度、机体 FLU 角速度、FLU→ENU 行优先旋转矩阵、两关节角和角速度。单位 m、m/s、m/s²、rad、rad/s。
- `State.control_time / stamp / joint_stamp`：绝对仿真或设备时间，分别表示本轮控制时刻、机体状态测量时刻、关节测量时刻。`StepInput.t` 是相对于 `origin` 的任务时间，必须与绝对时刻一致。
- `reset` 的 `Command5.time` 是**绝对时间**；`Context.reference_time / commands[].time / packets[].send_time / bus_origin` 是**相对于任务起点的时间**。预发送历史可对应负的任务时间。
- `Context.commands` 为真实已发布的关节命令历史；`packets` 为真实已发送的 TS20 包（包括 q/dq 参考、前馈、增益）。不能用优化器计划或尚未发布的候选代替它们。传感器状态、实际包和未来生成包的区别由原预测模型保留。
- `Target` 为世界系末端目标位置、速度与加速度；`preview` 要么包含准确的 15 个采样，要么留空使用原恒加速度外推。采样时刻为 `t + cumsum([dt, node_dt, ...])`。
- `StepResult.position_reference / velocity_feedforward / acceleration_feedforward` 是原飞机参考输出；`command[:3]` 是优化后的飞机速度控制量，`command[3:]` 与 `joint_velocity` 一致。

飞机执行适配器必须沿用既有 PX4 估计坐标系与 NED 编码。不要把 `command[:3]` 再与 `velocity_feedforward` 相加；不要对最终关节速度再次叠加一层位置 P。真正的消费确认、故障退出、收臂、降落与停桨仍属于执行层，不能仅靠一次 `prepare` 成功就宣告物理控制成功。

## 当前配置与边界

`selected_x650_simulation_profile()` 精确复制已验证的 `mpc25_dispatch50_v1` 配置：15 节点、40 ms 名义节点间隔、75 个优化变量、25 Hz 上层准入/50 Hz 下发、实际模型步长最大 100 ms、条件式首条命令保持预测 160 ms、总线 5 ms、TS20 积分 1 ms。原核心求解总预算 50 ms；封装及调度时间不包含在核心内部计时中，应用仍需测量端到端延迟。

配置字段 `trust_radius=0.15` 保持原清单原值；ABI 4 核心实际采用 `min(trust_radius, 0.02)` 作为局部起始半径。关节实际速度预测内界仍为 0.42 / 0.32 rad/s，不增加松弛变量。本包没有重新设计恢复分支或节点间检查。

原清单 SHA-256：`ee6d3c300717b13b21b6a7a0eecf257e628664ff822f8683baf11b731fa4009b`。

| 后端 | 已验证 SHA-256 |
|---|---|
| `libx650_mpc_core.so` | `8c3a874ac57e7f442d28fb4f4e7ae11633addef0c36edb2d88e6937a7db38a19` |
| `libx650_optimization.so` | `e31a8060e131309306419d7bf4e26490d7045b2c7e49aa872e4f39cdd75ac7aa` |
| `libts20_prediction_v3.so` | `9ff66d6db9afb3eb4c24ca6c0c3fcfc9453c6ab4384af134ec22b6f5c38701d8` |

ABI 检查只识别接口版本，不能代替以上内容哈希校验。离线回放报告会记录实际加载的三个库路径和哈希。公开接口是 C++ ABI，需要与后端匹配的编译器/标准库；它不是跨编译器稳定的 C ABI，不能把 `.so` 文件直接复制到 Orin ARM64 上运行。

## 可选公共候选保护入口：legacy_mpc_common_gate_v1

新增 [common_gate.hpp](include/x650_mpc/common_gate.hpp) 提供 `x650_mpc::CommonGateController`，不修改上面的旧 `Controller`、选定参数或旧 `Core::step`。它补上独立事件模型候选保护，尤其是旧普通接受分支未无条件复核的实际节点 `q+r·dq` 制动量。算法仍是原局部 QP MPC；新增的是候选发布准入政策，不能把新运行叫作与历史验收完全相同的版本。

调用顺序为：原 issue 时刻检查 → 旧 `Controller.prepare` → 原 issue 截止复查 → 公共 `evaluate_physical_plan` → 再次核验总截止 → 返回待发布候选。验证使用原始 committed Memory、测量、实际已发送历史和原优化器物理计划，覆盖所有五通道命令界/变化率、参考积分界、原TS20报文与事件传播、连续 q/dq 包络、实际节点制动量、首指令 pending guard 和160 ms保持；不是重新求解QP，不放宽速度内界。

```cpp
#include "x650_mpc/common_gate.hpp"

x650_mpc::CommonGateController controller(actual_config, predictor_function);
controller.reset(actual_anchor, actual_pre_task_publications, task_origin_s);

// original_issue_ns 来自状态请求生成时；不得在这里重新打点刷新预算。
auto candidate = controller.prepare(input, original_issue_ns); // 默认上限50 ms
if (candidate.accepted) {
    // 发布方紧接其真实I/O前，仍需检查原 deadline_monotonic_ns。
    // 把 candidate.token 和 candidate.output 交给现有下发器。
    // 匹配的飞机和关节指令实际发布完成后，writer通知回调才调用：
    // controller.commit_published(candidate.token, exact_published_command);
    // 确认未发送而取消时：controller.discard(candidate.token);
}
```

`prepare(input, issue_monotonic_ns, budget_ns)` 返回 owning `CommonGateResult`：`accepted/status/detail/token`、可选原控制输出、完整 `PlanValidation` 及每阶段单调时钟/耗时。预算必须在 `(0, 50 ms]` 内，零预算、未来/零时间戳、已经过期的请求直接拒绝。预算包括旧Core、公共验证和结果复制；最终接受后，应用排队时间仍需在实际发布前再次检查。Linux上时间戳使用同一主机 `CLOCK_MONOTONIC`，可调用 `CommonGateController::monotonic_ns()`；不要混用ROS仿真时间、UTC或其它设备时钟。

普通错误返回 `accepted=false`；该次新proposal自动discard、token归零、committed Memory不变。拒绝结果可能保留原候选输出用于审计，**不能因为存在output就发布**。`bad_alloc`等异常在清理新proposal后重新抛出。已有未解决的accepted候选会使下一次prepare返回 `unresolved_pending_candidate`，原候选保留，避免隐式取消一个可能已发送的计划。

`commit_published`仍验证token和精确的五维实际发布命令；旧token、拒绝token、错误命令和重复提交均拒绝。它记录的是已经发生的真实发布，因此不因writer通知本身晚到就擦除实际历史；原issue截止限制的是发布准入，不是消费ACK返回时刻。应用负责可信发送通知、发送时刻/编码检查、部分发布的故障处理及发布前最后一次截止检查。本适配器无ROS I/O，也不证明所有未来预测节点真的执行了。

构建时必须使用含新增 `evaluate_physical_plan` 符号的**隔离新库及匹配头文件**；旧归档库不含该符号。默认 `X650_MPC_BUILD_COMMON_GATE=OFF`，原入口仍可用旧三库构建。新target为 `x650_mpc::common_gate`：

```bash
S="$R/artifacts/standard_nmpc_20261027"
B="$S/mpc_common_gate_v1/build"
cmake -S "$R/control_packages/x650_mpc_controller_v1" -B "$B" \
  -DCMAKE_BUILD_TYPE=Release -DX650_MPC_BUILD_COMMON_GATE=ON \
  -DX650_BACKEND_LIBRARY_DIR="$S/native_build_v1/legacy_native" \
  -DX650_NATIVE_INCLUDE_DIR="$S/source_v1/control_packages/x650_wholebody_v1/native" \
  -DX650_TS20_LIBRARY=/root/autodl-tmp/libts20_prediction_v3.so
cmake --build "$B" -j2
ctest --test-dir "$B" --output-on-failure
"$B/x650_mpc_common_gate_one_step"
```

`common_gate_one_step`是使用合成几何的可执行调用示例；没有实际发送，因此主动discard。公共保护仍是条件事件模型检查和工程制动余量，不能宣称真机保证制动力或连续时间制动不变量已经获证明。

2026-10-27，该新入口在隔离服务器构建中 **15/15项CTest通过**：原接口7项不变，加上预算前检查、安全事务、错误/过期token、原生拒绝、pending/reset、验证后超时与异常清理7项，以及一项包含两条真实历史输入的回归。验证阶段注入55 ms停顿后，50 ms总截止正确拒绝且不提交；注入分配异常同样不残留pending。真实历史 `hold_000303`通过并提交与直接Core完全相同的Memory；`wrench_000363`由旧Core接受原计划，但新入口以 **0.002172563902 rad** 实际节点制动缺口拒绝，其余保护通过，committed Memory不变。测试核对重新求得的计划与后继Memory中绑定的历史计划，未编造执行历史。

固定输入存于 [common_gate_recorded_v1.json](tests/fixtures/common_gate_recorded_v1.json)，SHA256 `566404fbf4f974d85a9b947a4df7ba655d9c63c1930a498d665484c38edf70fc`。其中保留完整原Memory/input及来源审计/批文件哈希；测试不依赖再次读取服务器绝对来源路径。以上均是可调用接口与固定记录验证，没有运行新的物理飞行。

## 旧可调用入口验证方法

2026-10-27，在新服务器容器上复用上述三个已验证共享库完成验证：**7/7 项 CTest 通过，24/24 条固定真实记录回放通过**。这 24 条输入只覆盖原任务 **0.048–0.992 秒的初始化/保持段**，尚未到八字开始时刻；不能视为完整八字或外扰回放。包装器与直接调用 Core 的控制输出、矩阵、计划、包络及提交后 Memory 相等；选定配置全部字段匹配原记录，历史输出最大绝对差为 **0**，实际加载的三个库 SHA 与原版本一致。回放报告为 `artifacts/nmpc_interface_20261027/mpc_api_replay_report.json`。这些结果是接口和固定输入数值验证，没有启动新的飞行或物理闭环实验。

随后将回放扩展到已有全量任务快照：保持 711 条、受扰快速八字 570 条，共 **1,281/1,281 通过**。facade 对直接 Core 仍完全相同；对原历史输出的最大绝对差为 `1.943e-16`。全量 Memory 沿用已完成来源绑定的确定性重建结果，与上述 24 条直接 oracle Memory 的来源不同。输入覆盖原任务全程，但仍是逐快照离线检查，不是新接口驱动的飞行。完整证据见 [接口与 NMPC 原型验证](../../docs/validation/nmpc_interface_20261027/README.md)。

七项 CTest 测试使用真实 native 算法，覆盖未下发不提交、显式丢弃、直接 Core 数值等价及提交后的 Memory 等价、token 过期/复用、错误历史、状态时钟倒退、原生异常、错误下发内容与重置。矩阵、QP 解、计划、连续包络和控制输出不受封装影响；计算耗时本身不作逐位相等要求。

用原 24 条记录及原 C++ 顺序 oracle，进一步验证真实输入：

```bash
python3 control_packages/x650_mpc_controller_v1/examples/prepare_recording_replay.py \
  --recording "$M/wrench_fast_inputs_v2/recording/recording.json" \
  --oracle "$M/wrench_fast_inputs_v2/oracle/report.json" \
  --output /absolute/new/path/mpc_api_fixture.json
# 上一条打印输出文件的 SHA；传入同一个值。
"$B/x650_mpc_replay" /absolute/new/path/mpc_api_fixture.json <printed_sha256>
```

数据适配器核验记录与 oracle 的逐文件大小和 SHA，保留实际测量与包历史，使用 oracle 直接保存的 `memory_before`，不拟合或重新构造动力学。C++ 回放同时检查包装器与直接 Core 的逐值相等、历史输出容差、prepare 不提交和提交后的 Memory 相等。报告明确标记 `physical_validation=false`；它不能替代新封装接入后的完整物理闭环验收。

旧 ROS worker 和历史实验入口保持原样。本包先建立明确、可测试的复用边界；以后将 MPC/NMPC 接到同一执行层时，需要分别完成源新鲜度、连续下发、关节实际消费及接管/退出验收。当前仿真结果不能直接视为真机可部署认证。
