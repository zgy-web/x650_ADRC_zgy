# 规划节点安全余量核查：2026-10-26

**已统一 QP 节点与最终候选检查的 q／dq 安全余量。第 429 次固定输入仍拒绝，完整受扰实验仍未通过。** 本次没有启动新的 Gazebo 飞行，也没有替换已有验收绑定的运行库。

方法、修改范围、数值和节点间极值解释见[关节安全余量说明](../../X650_MPC_JOINT_SAFETY_MARGIN.md)。独立审计 `passed=true` 表示修改和证据被核实；报告同时明确 `margin_fix_solved_429=false`、`flight_passed=false`。

## 可查看记录

- [独立矩阵／候选／极值审计](margin_audit_v1.json)：A/b/H/g/C 逐元素相同，只收紧 lower/upper 各 60 项；保留命令、参考、制动、trust、pending 与时限。给出 15 节点的线性／完整模型逐项数值及节点间 acceleration_zero 极值。
- [原版固定输入结果](baseline429_v1.summary.json)与[新版固定输入结果](margin429_v1.summary.json)：最低预测 dq1 从 −0.427988411839 变为 −0.420047472385 rad/s；扣除余量和原数值容差后，仍差约 0.000047272 rad/s。
- [原输入与完整 Memory](000429.input.json)：原 428 次完整重建后的固定状态，SHA `0ed8f4a327fc0a00a4f314bc8dc32b2b53b2e0bfe6567074ea0e7d77fd8a0d00`。
- [原始前缀重建报告](original_sequence_reconstruction_report.json)：429 次真实记录；前 428 次完整结果与 proposal 差为 0，包含原来的两次恢复；429 重现拒绝且 Memory 不变。
- [另外七个固定输入](fixed_calls_v1.summary.json)：序号 1/378/379/380/383/384/428 均通过，恢复事件保留。每条都使用各自原 Memory，不能把它们当作新版闭环轨迹。
- [Python/native 测试日志](python_tests_v1.log)、[C++ 构建与 CTest 日志](build_v1.log)、[独立审计反例日志](audit_tests_v1.log)：15+21+6+2 项控制相关测试和 7 项独立审计反例全部通过。

## 原件与复现

[完整归档](joint_margin_evidence_v1.zip)为 4,965,926 bytes，包含 953 个成员，SHA-256：

`0dc678d61d4e0f2db813a5c124bdae383a952da5bb1f7dfbcfd192fdfff33e54`

[逐件清单](archive_manifest.json)、[交付清单](delivery_manifest.json)、[本地校验](local_verification_v1.json)保存源文件及每个 ZIP 成员的 SHA、大小和完整性结果。归档含原 CDR 的哈希绑定导出、原 Core 重建的全部输入、原／新版单次 Full 数值、源码、测试、独立构建库和命令记录；原实际飞行 CDR/ULog 已在[前次物理归档](../cpp_recovery_flight_20261026/README.md)保留。本地核验全部成员 SHA／CRC，7 份新增或改动的执行源码也与本地待提交源码逐字节一致。

原库与新版库身份分别记录在 raw replay 的 `loaded_code` 中：

| 库 | 原版 SHA-256 | 本次离线新版 SHA-256 |
|---|---|---|
| Core | `f90c565fdda137771fd0e85bc22386a3b30c41ac928246e177536bc102947011` | `42cc8c0e672ce2572f87171218aea178bb8ee88ba1d7ca39eadf11c1333778cb` |
| QP/assembly/candidate | `582b0a150787333936b04c4c9b40d461db99fa884577e81e6ff230f49a4b3452` | `5188f6f6af9b5c07f9508d756dc20d09552f597bc2f0bd09ed9e341cf54c3ff6` |
| TS20 | `9ff66d6db9afb3eb4c24ca6c0c3fcfc9453c6ab4384af134ec22b6f5c38701d8` | 相同 |

源码核验顺序：

1. `scripts/export_x650_runtime_replay.py` 对原实际 ABI 2 运行的 CDR、startup、库和 native records 做绑定导出。
2. 用原 `x650_cpp_recovery_sequence.cpp` 重建原前缀，核对每条原输出，取得固定输入。原前缀不是用新版重算生成。
3. 分别以原／新版匹配库构建 `scripts/x650_cpp_recovery_replay.cpp`，同一输入恢复同一 Memory，只执行一轮。归档中的两个 raw 保留退出码和拒绝。
4. 使用只读审计器核查：

```bash
python3 scripts/audit_x650_joint_safety_margin.py \
  --fixture /path/to/reconstruction429_v1/000429.input.json \
  --baseline /path/to/baseline429_v1.raw.json \
  --padded /path/to/margin429_v1.raw.json \
  --output /path/to/new_audit.json
```

运行测试时必须指定新版优化库，避免 native 测试被跳过：

```bash
X650_NATIVE_OPTIMIZATION_LIBRARY=/path/to/new/libx650_optimization.so \
  python3 -m unittest discover -s tests/unit -p test_x650_joint_safety_margin.py -v
python3 -m unittest discover -s tests/unit -p test_x650_joint_safety_margin_audit.py -v
```

精确重建需要原服务器证据和对应环境；归档中的绝对路径保留原始身份。移动到其他目录时应重建库并明确新路径及 SHA，不修改原件伪装成原路径。旧验收文件不能直接用于新的优化库，后续应为新版本重新验证完整闭环。
