# 数学模型与 C++ 输入映射验证

本目录的测试只执行数值计算。它们不启动 Gazebo、不发布执行器指令，也不提交控制器 Memory。求解器的 NLP/KKT、运行时间及物理闭环资格是后续独立项目，不能用这里的结果替代。

## 冻结模型与数据

采用四份预先选择的原始反馈文件，各四例，共 16 例：`hold_000_selected.json`、`hold_001_selected.json`、`wrench_000_selected.json`、`wrench_001_selected.json`。文件包含真实状态、已提交 Memory、已发送报文和原始时间戳；报告保存源路径、SHA256 和原记录索引。测试覆盖实际 0.052 s 首段，以及非零运动和待生效报文，未将输入改成静止合成状态。

模型物理参数与摩擦平滑宽度保持不变，`epsilon=0.05 rad/s`。当前数值版本采用：

- 未来每个至多 5 ms 事件区间 10 个 RK4 子步，最大步长 0.5 ms。
- 原始测量的已发送历史对齐采用 20 个子步，最大步长 0.25 ms。
- 步长减半版本同时使用未来 20、历史 40 个子步。
- 无松弛变量，原关节预测速度内界仍为 0.42 / 0.32 rad/s。

`qualify_model.py` 构造独立的全局报文时间线，用 DOP853 积分相同光滑方程，核对固定 stage 事件安排及 RK4 传播；独立增广矩阵指数核对机体 ZOH，非零偏航率/姿态残差导数的中心差分核对末端 `Rdot*r`，并核对六项代价及 AD 方向导数。原事件模型由单独原生程序重新预测；其误差保留为模型差异。

`test_model_math.py` 的四项测试另外覆盖总线相位、恰落节点边界的发送/施加事件、0.02/0.031/0.04/0.052/0.1 s 首段、参数切片，以及真实历史的保留和坏输入拒绝。

## 可复现命令

在项目根目录运行，`DATA` 指向四份原始选择文件所在目录，输出目录必须是新目录：

```bash
source /root/autodl-tmp/x650-nmpc-standard-runtime/environment.sh
export PYTHONPATH="$PWD/control_packages/x650_nmpc_standard_v1/python"
export X650_STANDARD_FIXTURES="$PWD/artifacts/standard_nmpc_20261027"
DATA="$X650_STANDARD_FIXTURES"
python control_packages/x650_nmpc_standard_v1/tests/test_model_math.py
python control_packages/x650_nmpc_standard_v1/tests/qualify_model.py \
  --selection "$DATA/hold_000_selected.json" \
  --selection "$DATA/hold_001_selected.json" \
  --selection "$DATA/wrench_000_selected.json" \
  --selection "$DATA/wrench_001_selected.json" \
  --event-replay "$DATA/native_build_v1/x650_standard_event_replay" \
  --output "$DATA/model_qualification_new"
python control_packages/x650_nmpc_standard_v1/tests/qualify_case_builder.py \
  --selection "$DATA/hold_000_selected.json" \
  --selection "$DATA/hold_001_selected.json" \
  --selection "$DATA/wrench_000_selected.json" \
  --selection "$DATA/wrench_001_selected.json" \
  --cpp-builder "$DATA/case_builder_v1/x650_standard_case_replay" \
  --output "$DATA/case_builder_new"
```

原生程序也可由本包 CMake 的 `x650_standard_event_replay` 和 `x650_standard_case_replay` 目标构建。测试版本的纯 C++ builder 使用 Eigen、JsonCpp（仅 CLI）和 OpenSSL（仅 CLI）；核心 `build_case(Config,Memory,StepInput)` 不依赖 Python、ROS 或优化器。

## 已取得的数值结果（2026-10-27）

完整报告为 [model_qualification_v5.json](../../../docs/validation/standard_nmpc_20261027/model_qualification_v5.json) 与 [case_builder_mapping_v1.json](../../../docs/validation/standard_nmpc_20261027/case_builder_mapping_v1.json)。原文件分别来自服务器的 `model_qualification_v5/report.json`、`case_builder_v1/qualification/report.json`，复制时未改内容。

| 检查 | 结果 |
|---|---:|
| 模型积分、全时轴事件安排及对齐 | 16 / 16 通过 |
| RK4 节点角速度与独立 DOP853 最大差 | 1.262e-7 rad/s |
| 历史对齐状态最大差 | 1.359e-7（q、dq 分量各自物理单位） |
| 同时将历史与未来步长减半，最大角速度差 | 1.281e-7 rad/s |
| 上述预先声明绝对数值容差 | 2e-6，未放宽 |
| 非零转动末端速度中心差分最大差 | 3.239e-11 m/s |
| 离散状态/代价 AD 方向导数归一化差 | 6.074e-11 |
| C++ 与 Python 原始输入映射 | 16 / 16 通过 |
| C++ / Python 最大 x0 差 | 2.776e-17 |
| C++ / Python 最大 stage 参数差 | 4.441e-16 |
| C++ / Python 初始计划及步长差 | 0 |
| 映射预先声明绝对容差 | 1e-10，未放宽 |
| 两种映射对不支持/错误输入的共同拒绝 | 12 / 12 |

拒绝测试包含未 prime 的 Memory、缺失/倒序/未来报文、驱动增益变化、过期/未来关节状态、错误 epoch、超范围 dt、无效旋转、缺失命令和不覆盖原测量的历史。输入不是在构造器里用人工历史补齐的。

v5 模型源码 SHA256 为 `ef82964be55f0bfb14caaf8acdb21231d9cf076e0f78be54032d8ad4f129db7b`；配置源码 SHA256 为 `9295bc40c1a9b4648170096aa9c3cb82f9e36051e36a9ac61128697884fe80c6`。服务器 `source_model_v2` 保留这一版本；报告还保存测试脚本及原事件验证程序的身份。

## 必须保留的限制与探索记录

光滑模型与原事件模型的差异没有被积分精度测试消除。同一组原已提交计划的移位初值上，关节预测最大差约为 **0.01095 rad / 0.04233 rad/s**；在相同机体预测上换算为末端，最大位置差 **3.225 mm**、速度差 **13.204 mm/s**。低速时原事件模型存在静止黏着，光滑模型不含该离散状态。这不是实际传感器实测跟踪误差，也不构成物理拟合通过。

16 条原移位初始计划中，15 条通过原事件候选检查；`wrench_000486` 因实际节点制动保护余量失败而保留拒绝。这里没有求解新 NMPC，不能把这 15/16 当成新优化器通过率。新求解结果仍须逐条通过原事件连续包络、保持及历史保护验证。

服务器探索目录 `model_qualification_v1`–`v4` 保留，不覆盖：v1 因测试脚本对 Python list 直接 `abs` 提前报错；v2 在步长减半断言处退出；v3 补充收集后在历史对齐断言处退出；v4 完成全表，发现原先历史与未来都用 10 子步时一例超过 2e-6 容差，历史对齐差 2.356e-6、步长减半差 2.220e-6。随后明确新增独立 `history_rk4_substeps_per_bus=20`，未来仍 10，得到新 v5。物理参数、平滑宽度和判据均未借机调整。

数值映射和积分通过只说明当前声明方程实现一致；不代表 SQP 收敛、实时截止期、原事件安全或新保持/八字物理实验已通过。

## 生成 C 求解器的跨语言复现

`qualify_native_solver.py` 在求解器生成完成后分别验证两条路径：导出的完整数值输入→C++ 调用生成 C，以及原始反馈/实际历史→C++ 构造器→同一生成 C。每个工况均独立 reset 后重复三次。SQP 与 SQP-RTI 必须使用各自的编译目录和报告，不能只靠相同的图 SHA 判断算法模式。

```bash
python control_packages/x650_nmpc_standard_v1/tests/qualify_native_solver.py \
  --selection "$DATA/hold_000_selected.json" \
  --selection "$DATA/hold_001_selected.json" \
  --selection "$DATA/wrench_000_selected.json" \
  --selection "$DATA/wrench_001_selected.json" \
  --numerical-dir "$DATA/numerical_v3" --mode sqp \
  --output "$DATA/native_reproduction_new" --prepare-only
python control_packages/x650_nmpc_standard_v1/tests/qualify_native_solver.py \
  --prepared-dir "$DATA/native_reproduction_new" \
  --binary-dir /absolute/path/to/matching/sqp/build
```

脚本先绑定 16 份原始工况与合并输入，然后要求 Python 全部数值记录完成、源文件及生成物身份保持不变，才执行原生程序。预声明容差为输入映射 `1e-10`、最终控制和状态复现 `1e-6`；不同模式、状态码、拒绝标志及三次重复均保留，未达容差不能自动扩大。错误输入 SHA、同维度错误 Config、错误 graph 身份及短数组须显式拒绝，且不得生成声称有效的结果文件。

Python 求解报告中的 `states` 是精确非线性 rollout，并非求解器原始 multiple-shooting 状态。因此脚本对比 C++ rollout 与 Python rollout，同时对比两条 C++ 路径的原始 shooting 状态；不会混淆 RTI 的局部动力学缺陷和跨语言复现差异。CPP 仅输出状态码成功时，也不替它编造完整 NLP 可行性或 KKT 通过标志。

setup、solve、postprocess 的原始耗时分别记录为观测值，不构成实时调度证明。这项复现本身不授权发布，也不替代原事件候选验证与新的物理闭环。

2026-10-27 已完成的两模式报告分别为 [SQP 原生复现](../../../docs/validation/standard_nmpc_20261027/native_reproduction_sqp_v1.json) 和 [SQP-RTI 原生复现](../../../docs/validation/standard_nmpc_20261027/native_reproduction_rti_v1.json)。每种模式均为 16 例 × 两条 C++ 路径 × 三次独立 reset，共 96 次调用；各自附带七项身份或输入错误拒绝测试。报告旁的同名 ZIP 保留全部输入、原始输出和进程日志，`_manifest.json` 保存逐文件 SHA256 及交付时源文件身份；本地下载已逐个校验 83 个证据文件。

| 复现检查 | SQP | SQP-RTI |
|---|---:|---:|
| 16 个工况的三次重复、两条路径 | 16 / 16 | 16 / 16 |
| 最大输入映射差 | 4.441e-16 | 4.441e-16 |
| 最大计划或状态复现差 | 7.272e-15 | 5.336e-13 |
| 错误身份/短数组拒绝，无结果文件 | 7 / 7 | 7 / 7 |
| 原始反馈→全 C++ 的 solve 中位 / 最大耗时 | 165.697 / 353.128 ms | 22.482 / 26.051 ms |
| 同一路径 postprocess 中位 / 最大耗时 | 15.318 / 24.867 ms | 23.476 / 26.587 ms |

这些时延是共享服务器上本轮观测，未声称独占测试，也未包含真实反馈原始发出时刻至执行端的完整截止期。SQP-RTI 的复现通过只证明与 Python 同模式输出一致；原 Python 独立检查仍为 **0 / 16 非线性可行**，不能因状态码为 0 或 solve 较快而发布。SQP 的原独立可行性及 KKT 为 16 / 16，通过后仍需原事件模型的最终检查。

## 构建模式切换与错误报告回归

`test_native_contract.py` 验证两项不改变数学模型的封装修复：CMake 在同一构建目录重新配置 SQP→SQP-RTI 时清除生成库的旧缓存；批量原始输入回放只要有一个 case 异常，就返回 2 并在全局报告标明 `complete=false`、`failed_cases` 和实际完成样本数。测试还用 `ldd` 确认运行时确实加载所选模式的生成库，不只读取新模式头文件。

```bash
python control_packages/x650_nmpc_standard_v1/tests/test_native_contract.py \
  --source-dir "$PWD/control_packages/x650_nmpc_standard_v1" \
  --acados-dir "$ACADOS_SOURCE_DIR" \
  --numerical-dir "$DATA/numerical_v3" \
  --original "$DATA/native_reproduction_sqp_v1/inputs/original_000.json" \
  --output "$DATA/native_contract_new"
```

已取得 [native_contract_v2.json](../../../docs/validation/standard_nmpc_20261027/native_contract_v2.json)：两模式的缓存与实际加载库均匹配；各一条正常输入输出与原 Python 同模式结果一致，最大差分别 `3.817e-17` 和 `1.180e-16`。混入一条未 prime 的真实 Memory 后，两模式都保留正常样本、将错误样本标失败、全局 `complete=false` / `failed_cases=1` / `completed_samples=1`，并返回 2。无发布授权。对应 ZIP 保留 24 个已核验文件，含输入、回放输出、构建日志和本次封装源文件。

此前全部 16 例的正常数值资格绑定原二进制并原样保留；本次独立 `source_native_contract_v2` 只修 CMake 与批量 CLI 的报错，未重跑或覆盖它们，也未借回归改变 OCP、参数或验收限值。
