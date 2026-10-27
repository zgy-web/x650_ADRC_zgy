# 规范 NMPC：16 个预选状态的求解与独立审计证据

本目录保存同一光滑非线性 OCP 的 IPOPT、acados SQP、acados SQP-RTI 数值记录。**SQP 的 16 个预选状态通过本次离线数值资格，但当前版本未取得在线或物理闭环资格。** 完整 SQP 的实际求解时间超过 50 ms；从这些独立初值只执行一步 RTI 时，16 个候选均未通过本次非线性可行性验收。因此不能用本目录批准发布指令，也不能用此前有限重线性化原型的保持或八字记录替代本包的新实验。

数学模型、五维执行接口及适用边界见 [包说明](../../../../control_packages/x650_nmpc_standard_v1/README.md) 和 [FORMULATION](../../../../control_packages/x650_nmpc_standard_v1/FORMULATION.md)。这里没有修改原 MPC 参数、关节速度内界 0.42 / 0.32 rad/s，也没有增加松弛变量。

## 数据及结果

数据来自事先选好的四份绑定记录 `hold_000`、`hold_001`、`wrench_000`、`wrench_001`，每份四例，合计 16 例。各例包含原始状态、已提交 Memory、实际报文历史和来源哈希。初值取原提交计划的时间移位结果，不使用 oracle 解充当优化初值。所有预选例子均保留，没有根据结果删去困难工况。

| 项目 | IPOPT 离线参考 | acados SQP | acados SQP-RTI，单步 |
|---|---:|---:|---:|
| 成功返回 | 16 / 16 | 16 / 16 | 16 / 16 |
| 独立非线性可行性通过 | 16 / 16 | 16 / 16 | 0 / 16 |
| 独立一阶 KKT 通过 | 完整对偶补充审计 16 / 16 | 16 / 16 | 0 / 16 |
| 实际迭代数 | 13–24 | 2–13 | 每例 1 次 |
| 求解时间最小值 | 6323.983 ms | 56.619 ms | 13.897 ms |
| 求解时间中位数 | 6388.118 ms | 109.831 ms | 15.088 ms |
| 求解时间 P95 | 10931.210 ms | 296.983 ms | 16.231 ms |
| 求解时间最大值 | 11694.658 ms | 351.053 ms | 16.546 ms |

这些时间来自服务器共享容器的一轮观测，是求解调用时间，不是 Orin NX 结果，也不是最坏执行时间保证。单独的 Python 离线独立审计还需要额外时间：SQP 的审计时间中位数 60.393 ms、最大值 76.862 ms，不能把审计藏在求解计时之外后声称完整控制周期合格。原始单例 JSON 同时保存 `solve_elapsed_ms` 和 `postsolve_validation_elapsed_ms`。生成 C/C++ 的独立调用复现属于另外一项验收。

SQP 的精确非线性 rollout 硬约束最大违约为 0；多重射击节点动力学缺陷最大值为 1.6891e-11，节点与精确 rollout 差异最大值为 1.2162e-11。SQP 的目标值均满足事先规定的局部 IPOPT 参考差值条件，绝对差最大约 1.282e-7；这不证明非凸问题的全局最优性。

RTI 的拒绝同时考虑多重射击节点动力学缺陷与精确 rollout 的硬约束，不能说“16 例全部越过了速度界限”。其精确 rollout 约束违约范围为 0–0.00167145，节点动力学缺陷最大值为 0.462073，节点与 rollout 差异最大值为 0.126348。后两项涉及不同量纲的状态分量，不能直接解读成米或 rad/s。`status=0` 只表示本次 RTI 调用正常返回，不表示完整非线性问题已收敛；这些候选全部保留拒绝结论。

## 原始 15 / 16 与补充 16 / 16 的关系

[原始 numerical_v3 报告](numerical_v3/report.json) 保留 **15 / 16** 综合通过，未被覆盖。唯一未通过的是 `wrench_000201` 的 IPOPT 活跃集合乘子重建检查。其预设 `active_tolerance=2e-6` 筛掉了内点解中尚有对偶贡献的近活跃约束。使用同一集合的 TRF、BVLS、NNLS 都得到约 0.030908 的驻点残差；这不是简单更换最小二乘方法便可解决的问题。诊断原件见 [乘子重建诊断](numerical_v3_multiplier_diagnostic.json)。

后续修正的是独立 KKT 审计器：保存 IPOPT 的完整有符号对偶变量，重新用同一模型的目标梯度、约束 Jacobian、全部上下界计算原始可行性、驻点、互补性和对偶符号条件。不能只接受求解器状态或仅核算驻点。原活跃集合重建结果仍作为诊断保留，所有既定容差不变。

[numerical_v4_ipopt_audit](numerical_v4_ipopt_audit/report.json) 对全部 16 例重新求解并执行完整对偶独立检查。所有计划、状态和目标值与 v3 **逐值完全相同**，不是通过放宽比较容差取得的一致。全部 16 例的最大完整对偶驻点残差为 4.071e-9、最大互补残差为 2.710e-9，原始违约和对偶符号违规均为 0。原问题例的驻点残差为 3.412e-11、互补残差为 2.710e-9。

[独立补充报告](numerical_v4_reference_supplement.json) 逐例绑定 v3、v4 的输入、数学图、数值参数、计划、状态和目标；它报告补充数值资格 16 / 16，同时保留原报告的 15 / 16、RTI 的 0 / 16 以及 `physical_acceptance=false`、`online_acceptance=false`。新增的三项反例测试分别检查近活跃内点、驻点满足但互补性不满足、缺失边界上的错误乘子；[全部 12 项测试记录](solver_contract_tests_v4_audit.log) 通过。

## 同一个 OCP，两个真实求解器配置

模型为 23 状态、5 控制、15 个预测阶段、每阶段 161 个数值参数和 113 个硬约束表达式。SQP 与 RTI 使用同一非线性动力学、六项目标和硬界；IPOPT 使用同图的单重射击表达式作独立参考。CasADi 自动微分与中心差分的目标、约束、Lagrangian Hessian 方向检查覆盖全部 16 例的初值和参考解，原报告逐例保存结果。

acados 使用标准 `NONLINEAR_LS` 与 `GAUSS_NEWTON`：`y = sqrt(2h) * residual`、`W = I`、`cost_scaling = 1`，所以代价仍为 `h * residualᵀ residual`。这是 SQP 的 Hessian 近似选择，不是把原非线性目标替换成固定二次目标。IPOPT 和独立 Hessian 检查使用精确自动微分。

依赖固定为 acados v0.6.0、CasADi 3.7.2、NumPy 1.26.4、SciPy 1.13.1。生成 OCP 和模型名均固定为 `x650_standard`。完整 SQP/RTI 配置与构建身份分别保存在：

- [SQP 生成绑定](numerical_v3/generated_sqp/generation_binding.json)、[生成清单](numerical_v3/generated_sqp/generated_solver_manifest.json)、[实际 OCP JSON](numerical_v3/generated_sqp/c_generated_code/acados_ocp.json)。
- [RTI 生成绑定](numerical_v3/generated_rti/generation_binding.json)、[生成清单](numerical_v3/generated_rti/generated_solver_manifest.json)、[实际 OCP JSON](numerical_v3/generated_rti/c_generated_code/acados_ocp.json)。
- [SQP 编译标志](numerical_v3/generated_sqp/compilation_flags.json) 和 [RTI 编译标志](numerical_v3/generated_rti/compilation_flags.json)，各自同目录保留完整编译日志。
- [同进程库身份审计](numerical_v3_loader_audit.json)：两份绝对路径的 `.so` 同时以 `RTLD_LOCAL` 加载，句柄、capsule 入口地址和动力学函数地址不同；生成 JSON/源码分别绑定 SQP / SQP_RTI。数值记录中的迭代数也分别为 2–13 与 1，不仅依靠 Python 标签区分模式。

acados v0.6.0 在 GN 模式仍生成两个精确残差 Hessian 函数；其 NLS GN 执行分支不调用这两个函数。为避免对巨大且未使用的表达式花费大量编译时间，仅这两个对象使用 `-O0`，热路径对象仍使用 `-O2`。数学表达式、梯度、约束和求解参数没有因此改变。先前未完成的 EXACT 构建与统一 `-O2` 构建分别保留 [v1 停止说明](numerical_v1/build_stop_status.json)、[v2 停止说明](numerical_v2/build_stop_status.json)；它们尚未开始求解，不能记成数值求解失败。

## 复现与文件身份

当前 `solver.py` 与 v3 生成 C 使用完全相同的模型图；变化仅为后续完整对偶审计。以下两个源快照都保留在本目录：

- `source_numeric_v3/`：生成现有 SQP/RTI C 代码及原始完整报告的精确源。
- `source_numeric_v4_audit/`：增加完整对偶审计的精确源，数学 `model.py`、`config.py` 与 v3 字节完全相同。

模型图 SHA256 为 `570791124c3b94cd7adb8e1e23ad97ba72459380d12c8a1e23a9df54739f0150`。各生成目录中的 `x650_model_identity.hpp` 同时绑定图、原配置 JSON、平滑代理 JSON 和求解模式。大量生成 C 源与库保留在服务器，文件哈希、生成源和逐对象编译标志在本目录归档，可按同版本重新生成。

在仓库根目录、已安装隔离依赖的服务器上，用新的输出目录重新运行：

```bash
source /root/autodl-tmp/x650-nmpc-standard-runtime/environment.sh
EVIDENCE=docs/validation/standard_nmpc_20261027/solver
python -m unittest discover -s tests/unit -p test_x650_standard_solver.py -v
python scripts/qualify_x650_standard_nmpc.py \
  --selection "$EVIDENCE/hold_000_selected.json" \
  --selection "$EVIDENCE/hold_001_selected.json" \
  --selection "$EVIDENCE/wrench_000_selected.json" \
  --selection "$EVIDENCE/wrench_001_selected.json" \
  --output artifacts/standard_nmpc_fresh_qualification
```

该命令会重新生成和编译 C 求解器，可能需要数分钟。历史精确调用另见 `run_numerical_v3.sh` 和 `run_numerical_v4_audit.sh`；其中绝对服务器路径用于说明本次运行身份，不要求新机器使用同一根目录。原始 JSON 内的服务器路径及哈希保持不变，本地对应关系见 [local_archive_mapping.json](local_archive_mapping.json)。下载时验证了 [139 个原文件的身份清单](solver_evidence_manifest_v1.json)；归档包 SHA256 为 `f123420daf2e100ebb835e00417f036d0bce388b6ad1d1679caca3428d17790a`。

本目录只证明预选状态上的建模、求解和审计结论。原事件模型候选验证、完整 C++ 路径、请求时效与真实闭环执行仍是独立条件；目前没有启动本包的物理实验。
