# 可信中心恢复：2026-10-26 软件验收

本轮在新容器 `autodl-container-3187449845-10f5a45f` 完成 Core ABI 2、原始数据回放、独立审计和实际 ROS 2 工作进程测试。**已定位的原 7.544 s 故障恢复成功；本索引没有签发新的闭环飞行证书。** 方法、边界和复现入口见 [恢复设计说明](../../X650_MPC_TRUSTED_CENTRE_RECOVERY.md)。

## 可读结果

- [软件验收汇总](software_acceptance_v3.json)：代码／库身份、正常路径一致性、原失败恢复、反例及测试计数。
- [交付清单](delivery_manifest_v4.json)：完整 ZIP 身份、三件分片大小与 SHA、CRC 检查及原始文件计数。
- [工作站独立校验](local_archive_verification_v1.json)：全部分片、完整 ZIP、7,690 件原件 SHA／大小及 15 份当前执行源码身份逐件核对通过。
- [开发阶段原上传源码包](development_source_bundles_v1.zip)及其[身份](development_source_bundles_v1.json)：保留五次实际上传的原 ZIP，包含最初编译旧 ABI 回放器时的确切源码，补充完整归档中的最终源码及构建命令。
- 原始归档：[分片 1](recovery_evidence_v3.zip.part01)、[分片 2](recovery_evidence_v3.zip.part02)、[分片 3](recovery_evidence_v3.zip.part03)。这是同一 ZIP 的按字节切分，按清单顺序拼接后解压。

| 验证 | 实际结果 |
|---|---|
| 原核前缀重建 | 从原 LiveStart 运行 1～379 次；前 378 次全部 10 项输出匹配原记录，379 再现原错误，拒绝后内存不变 |
| 独立旧／新核回归 | **380／380**：378 次正常、1 次原失败恢复、1 次合成安全拒绝，共 760 次独立 Core 调用 |
| 正常输出／内部状态 | 最大输出差 **0**；每次成功后完整内存逐值相同，未触发恢复 |
| 原 379 | 旧核 `QP infeasible`；新核两次正式 QP、一个可信制动中心、0 次辅助 QP，成功只提交一次内存 |
| 新核固定快照计算 | 回归中 **19.186755 ms**；进程内 Core 调用 wall 为 19.214553 ms |
| 正式 QP 原行残差／KKT stationarity | (9.0553\times10^{-16}\)／(1.6914\times10^{-16}\)，原门限保留 |
| 非线性与局部模型 | q 最大差 0.271278 mrad，dq 最大差 0.0122492 rad/s；非线性包络及节点制动约束通过 |
| ROS 2 实际 C++ 工作进程 | **379／379** 条原输入通过，最终正常退出；消息两端所有声明字段核对通过，记录无丢失 |
| ROS 第 379 次计时 | Core **21.407867 ms**；入口到候选生成 **21.427321 ms**；完整回调 **21.504634 ms** |
| 自动测试 | Python **24／24**、ROS C++ CTest **6／6**、benchmark／native CTest **4／4** |

时限仍为原 50 ms；正式关节信赖域半径为 0.02，没有扩大。固定快照回归和实际 ROS 节点测试是不同调用，计时分别列出；单次观测不代表最坏时间保证。

原始物理运行是 `x650_cpp_robust_wrench_v1_20261024_205309_55502`。旧核使用原 LiveStart、实际反馈、命令／MIT 包历史重建完整前缀，proposal 和 result 只用于比较，没有拿原 proposal 拼造新的内存。新核逐次恢复同一原内存后计算，不使用改变后输出生成新的反馈。

## 保留的安全边界

内部制动中心通过原状态／参考积分、native 连续包络、新中心处的完整原约束以及节点制动余量，才允许重新建立正式 QP。最后解还要通过原候选检查和恢复节点制动检查。搜索最多两个中心，三个正式 QP 和一个辅助投影，共同消耗原总时限。失败不提交内部状态，不直接下发中心，也不回退成旧计划。

原失败快照实际只使用了 `slew_limited_brake`；辅助投影在这次恢复中没有执行。合成反例将同一输入的 q2／dq2 改为 −0.349 rad／−0.35 rad/s，被 `joint soft envelope` 在 QP 前拒绝；它验证边界拒绝和完整回滚，没有被称为“找不到可信中心”的搜索覆盖。四份额外合成边界探查也原样保留，未混入已完成物理实验。

本轮没有启动 Gazebo／PX4 新飞行，ROS 测试在私有 domain 231 中仅存在回放节点和实际 C++ worker，未创建执行器发布者。没有注入额外延时。新后端接入真实仿真反馈后仍需要新的完整受扰飞行、误差恢复和生命周期验收；原保持／普通／快速证书继续属于原版本。

## 原件与构建身份

| 原件 | SHA-256 |
|---|---|
| 原 Core ABI 1 | `6af46a510654ff302c2f2dcec2b73c6e752776ff5cbd538254dd0c017d4cc63b` |
| 原 QP 库（本轮未变） | `582b0a150787333936b04c4c9b40d461db99fa884577e81e6ff230f49a4b3452` |
| 原 TS20 库（本轮未变） | `9ff66d6db9afb3eb4c24ca6c0c3fcfc9453c6ab4384af134ec22b6f5c38701d8` |
| 379 原始输入 fixture | `12e911b368f85c3c76ea1dbbd90bb50954f19d58eb3d2008253c50195da79016` |
| 软件验收 JSON | `8a7abf2eb6ce17736b2b1b4c1edc4b4697003ebf896295f64549ce67ae104de1` |
| 重组后的完整 ZIP | `b0ff810fe8f22207651a4baf9bb01c5c073100465c05b08533bdafec0fb4f272` |

新 Core、worker 和 15 份执行源码的实际身份在汇总 `code` 中，不能用文件名推断版本。Core 与 worker ABI 2 配套编译，ROS wire 保留原 schema。原 Core／QP／TS20 ELF 也在归档中。

完整 ZIP **113,641,676 bytes**，含 **7,690 件**带路径、大小、SHA 的原件，未压缩 431,186,330 bytes。按 Git 单文件大小限制切成三件：

| 分片 | Bytes | SHA-256 |
|---|---:|---|
| part01 | 41,943,040 | `fdb5e57d6cff145bdc057867cbcd955b1409ac8e0092ca5db3a229d2a0308f21` |
| part02 | 41,943,040 | `ef2f03ccad14d96d281b3d5a82305c3846e646ca8fa4a2946a15e2fc22e550b1` |
| part03 | 29,755,596 | `9fbf2899e968f62d764454c7fa833a06ed70d0215c5fa94ea79f84d3c2411595` |

归档包括原始 CDR／记录、导出清单、C++ 重建 memory／input、两套 raw 输出、独立 audit、真实 ROS 节点原记录／计时／线程分配、测试日志、源码快照、库和 worker。首次缺少构建头文件的失败日志、开发中间版本及四份额外合成探查均保留，没有覆盖成成功。

## 验证与恢复归档

在仓库根目录运行下列 Python。每件先校验，再在临时目录重组，最后检查原 ZIP SHA 和全部 CRC；不会覆盖已存在的目标文件。

```python
from pathlib import Path
import hashlib, json, tempfile, zipfile

folder = Path('docs/validation/cpp_recovery_20261026')
manifest = json.loads((folder / 'delivery_manifest_v4.json').read_text())
with tempfile.TemporaryDirectory() as recovery_temp:
    archive = Path(recovery_temp) / 'recovery_evidence_v3.zip'
    with archive.open('xb') as stream:
        for item in manifest['parts']:
            data = (folder / Path(item['path']).name).read_bytes()
            assert len(data) == item['bytes']
            assert hashlib.sha256(data).hexdigest() == item['sha256']
            stream.write(data)
    assert archive.stat().st_size == manifest['archive']['bytes']
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive']['sha256']
    with zipfile.ZipFile(archive) as records:
        assert records.testzip() is None
        index = json.loads(records.read('archive_manifest.json'))
        assert len(index['files']) == manifest['entries']
    print('PASS: all parts, complete ZIP and entry CRC verified')
```

复现 C++ 输入重建和 raw 对照见上方方法说明。原始绝对路径与哈希随原件保存；若在不同路径重建，需要另建派生绑定和新报告，不能覆盖原清单或修改原件来假装身份没有变化。私有 URDF／网格不在此软件归档中，固定输入回放不需要它们。
