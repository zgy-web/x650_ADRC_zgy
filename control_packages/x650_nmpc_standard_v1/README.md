# X650 上层规范 NMPC 后端

本包单独实现明确的光滑非线性最优控制问题，采用 CasADi 自动微分、acados SQP / SQP-RTI，并以同一数学问题的 IPOPT 解作离线参考。五维执行接口仍为飞机三轴速度与两个关节速度。它不将先前的有限重线性化原型重新命名，也不改变既有实验归档。

**当前阶段：实现与数值资格验证。未取得本包的新物理闭环验收前，不能使用此前 `x650_nmpc_controller_v1` 的保持或八字结果证明本包通过。**

## 模型、求解与执行边界

- 优化模型保留闭环机体响应、非线性姿态/末端几何与舵机响应。控制量仍是速度，不是旋翼推力或关节力矩。
- 优化使用显式声明的光滑执行器代理。未来指令量化按连续量处理，静摩擦/库仑摩擦使用平滑函数；已发送历史报文与时间不可修改。模型近似的幅度与验证误差必须报告。
- 模型离散映射、六项目标、硬约束及其导数来自同一 CasADi 表达式，不能用另一模型的灵敏度冒充该问题的导数。
- SQP、RTI 和 IPOPT 对应同一个明确问题。RTI 是有限迭代；成功返回不自动表示完整 NLP 收敛。
- 独立原生验证器重新生成真实编码报文，检查原事件模型的节点间包络、全部节点停止余量和首指令保持。关节速度预测内界仍是 **0.42 / 0.32 rad/s**，不加入松弛变量。
- 求解器数值资格、候选模型验证、原始请求时效及实际执行确认分别记录。通过其中一项不能替代其余项。
- Python 仅用于模型建立、代码生成和离线分析；最终在线求解接口使用生成的 C 代码与 C++ 封装。固定参数 JSON 上的 C++ 测试不等同于已接入实际反馈。

## C++ 数值调用入口

生成并编译对应模式的 C 求解器后，使用本包的 `x650_standard_acados_backend`。一份实例由一个工作进程独占，配置必须与生成的模型完全匹配：

```cpp
#include "acados_backend.hpp"

x650_standard::AcadosBackend solver(config); // 初始化时构建一次
auto problem = x650_standard::build_case(config, memory, input);
auto result = solver.solve(problem);
// result 保存 plan、shooting/rollout 状态、残差和分段计时。
// 此处尚未获得发布许可，也没有修改实际发送历史。
```

生成目录中的 `x650_model_identity.hpp` 绑定配置、数学图与 SQP/RTI 模式。构建示例：

```bash
cmake -S control_packages/x650_nmpc_standard_v1 -B build/standard_sqp \
  -DCMAKE_BUILD_TYPE=Release \
  -DX650_ACADOS_GENERATED_DIR=/absolute/path/to/generated_sqp/c_generated_code \
  -DACADOS_SOURCE_DIR="$ACADOS_SOURCE_DIR"
cmake --build build/standard_sqp --target x650_standard_solver_replay -j2
```

`x650_standard_solver_replay` 直接读取绑定的原始状态/历史，经 C++ 构造后调用生成 C；`x650_standard_native_replay` 从完整数值输入独立核对同一求解器。后者会读取导出文件的约束边界，SHA 绑定的是完整数值文件，`compiled_config` 身份本身不单独证明任意外部边界符合原保护配置，因此只用于清单绑定的离线复现。两者均不发布命令。后续 ROS 适配必须再执行公共事件保护、原请求时效和真实发送提交逻辑，不能只把 `raw_status==0` 当成准入。

## 命名与对照

| 名称 | 范围 |
|---|---|
| `legacy_mpc` | 已保留的在线局部线性化 QP 控制器及其历史记录 |
| `nmpc_prototype` | 旧 QP 候选生成器加非线性代价下降接受的有限迭代原型 |
| `standard_nmpc_sqp` | 本包显式光滑 OCP 的 acados SQP 实现 |
| `standard_nmpc_rti` | 同一 OCP 的 acados SQP-RTI 实现 |
| `ipopt_reference` | 同一 OCP 的离线数值参考，不作为实时部署结论 |

与旧 MPC 的系统比较必须披露优化预测模型的变化；“相同硬件/被控对象和执行接口”不意味着控制器内部模型相同。若研究求解策略本身，应让局部 QP 与 NMPC 从同一光滑 OCP 派生，并使用相同的公共候选验证。

## 数值验收顺序

1. 固定源版本、数据选择规则、模型近似及容差，核对真实状态、实际报文历史与初始化。
2. 对离散动力学、残差目标、约束的自动微分作中心差分方向检查；对积分器作步长减半检查。
3. 用预先选择的保持/受扰原始状态比较 IPOPT 和 acados SQP；报告目标、原始可行性、驻点、对偶及互补残差。非凸问题不要求不同解逐项相同，也不宣称全局最优。
4. 对两个求解器的候选都作原事件模型验证，报告光滑代理与事件模型的节点差异及连续包络检查。
5. 生成 C 代码，从 C++ 独立调用并核对解与时间；测试非法输入、不可行、迭代耗尽及迟到拒绝。
6. 数值/时效/候选验证通过后，接回 ROS 实际反馈，重新验收完整末端保持，再普通和快速八字。

已发生的拒绝必须保留原因；不可为了让验收通过扩大速度硬界、改写原始时钟，或用旧物理记录拼接新算法结果。

## 隔离依赖

服务器安装脚本为 `scripts/bootstrap_x650_standard_nmpc.sh`。依赖根目录默认 `/root/autodl-tmp/x650-nmpc-standard-runtime`，与 ROS、PX4、原算法 Python 环境分开。

- acados `v0.6.0`，源码提交 `503364817c872d474ab5bed219c26760ac267769`；子模块使用该提交指定的版本。
- CasADi `3.7.2`；NumPy `1.26.4`；SciPy `1.13.1`。
- 完整 Python 依赖、子模块和实际编译身份由安装/验收产物记录。

```bash
bash scripts/bootstrap_x650_standard_nmpc.sh
source /root/autodl-tmp/x650-nmpc-standard-runtime/environment.sh
export PYTHONPATH="$PWD/control_packages/x650_nmpc_standard_v1/python${PYTHONPATH:+:$PYTHONPATH}"
```

完整状态、非线性方程、目标、约束及其实现边界见 [FORMULATION.md](FORMULATION.md)。实测验收结果随资格工作补充；当前文档不将计划中的测试记为通过。

## 2026-10-27 数值资格结果

使用事先选择的 16 份原保持/受扰反馈及实际发送历史，未运行新物理实验。数学模型积分与自动微分检查 16/16 通过，C++ / Python 状态及参数构造 16/16 一致，并共同拒绝 12 类无效输入。完整步骤和误差见 [模型资格说明](tests/README.md)。

同一个显式非线性问题上，IPOPT 完整对偶独立核算和 acados SQP 均为 16/16 数值通过；原第一版审计中的 15/16 结果保留，额外审计证明所有计划、状态和目标逐值相同，修正的是近活跃乘子筛选不足。单步 SQP-RTI 的 16 次调用虽然全部返回 status 0，但完整非线性可行性均未通过，涉及多重打靶动力学缺陷和部分精确 rollout 的约束违约，不能解释成 16 例全部超出速度界。详见 [求解器证据](../../docs/validation/standard_nmpc_20261027/solver/README.md)。

公共原事件模型保护对三个后端均为 15/16 通过。同一 `wrench_000201` 工况中，关节 2 的连续预测速度峰值分别为 IPOPT `0.321808955`、SQP `0.321808972`、RTI `0.321631629 rad/s`，超过 0.32 内界，全部拒绝。位置、节点制动量、待生效命令保护及首指令保持仍通过。这是新候选的模型预测结果，不是新的物理飞行实测越界；不修改速度硬界。见 [独立公共事件审计](../../docs/validation/standard_nmpc_20261027/standard_event_audit_v1/report.json)。

首次 Python 包装下、生成 C 求解器的 SQP 调用观测为约 56.62–351.05 ms，RTI 为约 13.90–16.55 ms。这里没有计算原始请求至最终发布的完整路径，也不是 Orin NX 的实时性能证明。SQP 已超过当前 50 ms 总预算，而 RTI 尚不可行；本版本因此没有取得在线或物理闭环准入。C++ 独立复现、求解数值、公共候选保护与真实闭环分别报告，不使用旧原型视频证明新包通过。

后续纯 C++ 复现中，两个模式均完成 16 例、两条输入路径、各三次独立 reset，最大计划/状态差为 SQP `7.272e-15`、RTI `5.336e-13`，7/7 错误身份或畸形输入拒绝。完整 C++ 路径的求解耗时中位/最大值为 SQP `165.697 / 353.128 ms`、RTI `22.482 / 26.051 ms`，另有后处理开销；共享服务器负载与测量路径不同，不用两轮计时反推语言本身的快慢。原数值资格结论保持不变。同一 build 目录切换 SQP→RTI 及混合坏样本的失败返回也已回归。详见 [原生复现与契约测试](tests/README.md)。

## 依据

- [acados OCP/NLP 与内部 QP 的区别](https://docs.acados.org/problem_formulation/index.html)
- [acados 功能与 SQP-RTI](https://docs.acados.org/features/index.html)
- [CasADi 建模、自动微分和代码生成](https://web.casadi.org/docs/)
- [原 MPC 基线审计](../../docs/MPC_BASELINE_AUDIT_20261027.md)
