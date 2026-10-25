# Orin NX 上的 MPC 计算延迟基准

此分支用真实 Orin NX 测量现有 C++ MPC 的计算耗时。第一阶段只测试算法本体，服务器仿真、SSH 往返、ROS 通信和日志保存分别列出；不把它们当作 MPC 计算耗时。首轮板上CPU基准已完成并通过独立复核：12,000次调用中位5.534336 ms、P99为7.048384 ms、最大7.236192 ms，见[实际测量记录](validation/orin_nx_mpc_20261025/README.md)。这组固定合成快照不代表连续飞行或多次重线性化压力；下一步再决定CUDA加速的具体算子。

已知连接信息为 `jetson@192.168.30.145`。截图中的 `4000` 是 NoMachine 的 NX 协议端口；SSH 先检查默认的 `22`，不把这两个端口混用。已只读确认AArch64 Orin NX、Ubuntu22.04.5、Jetson Linux36.5.0与CUDA12.6。未安装nvidia-jetpack元包，因此不据桌面Ubuntu名称或缺失元包猜测完整JetPack版本；功耗与散热条件由原始查询记录。

## 数据保留约束

用户要求“NX 上面只运行代码，日志输出等不要留存到 NX 上面”。本方案据此安排：

| 内容 | 存放或传输方式 |
|---|---|
| 可执行文件及 Core、QP、TS20 预测器库 | NX 的专用代码目录 |
| 源码与编译临时文件 | 通过 stdin 送入 NX 的 `/dev/shm` 临时工作区，构建后删除；构建的 `TMPDIR` 也指向这个 RAM 工作区 |
| CMake 等构建工具自身产生的临时日志 | 仅在上述 RAM 工作区，不在 NX 的持久目录执行 CTest |
| 原始输入、冻结工况、历史结果和验收报告 | 工作站；不复制到 NX 数据目录 |
| 一轮 benchmark 的输入 | 一个 SSH 进程的 stdin，全部解析到 NX 内存后再计时 |
| 每次调用的耗时、结果和错误 | 测量期间保存在预分配内存；整轮结束后通过 stdout 返回工作站 |
| 构建输出、SSH stderr、完整结果及统计 | 只在工作站保存 |

应用不创建远端输入文件、结果文件、日志重定向或 `tee` 文件，远端启动命令禁用应用 core dump。这里约束的是本实验的应用输出；操作系统自身的 SSH 日志、swap 或崩溃收集机制需要另外核实，不能把“程序没有主动写结果文件”说成系统绝无磁盘写入。

## 计时边界

独立可执行程序在启动时完成动态库核对、stdin SHA-256 核对、JSON 解析、样本与结果空间准备，之后才调用 MPC。每个样本先恢复原始 `Memory`，随后单独测量 `Core::step()`。状态恢复、JSON、文件、stdout、网络和进程启动均不在这个计时窗口中。

每次 Core 调用同时记录单调时钟墙钟耗时和调用线程的 CPU 时间。两者差异可提示调度等待，但不能凭这个差值精确拆分所有内存、缓存和抢占开销。NX 的降频、抢占和内存争用仍属于板上运行条件，不能简单从结果中扣除。

第一次调用、预热、正式测量分开保留。正式计时期间不逐样本打印，也不通过 SSH 请求下一个样本；所有样本先在本机内存就绪，再循环计算。整轮测完后才进行 JSON 编码并输出，因此 SSH 背压不进入 Core 计时。工作站记录的整段 SSH 墙钟时间只说明传输与组织成本，不能报告成 MPC 算时。

本阶段采用 `DiagnosticMode::Compact`，与飞行 worker 的生产调用方式一致。原模型、双精度、权重、约束、候选传播检查和 **50 ms 生产计算保护**均保留。超过保护、求解失败或 oracle 不一致的调用同样保留并计入失败；不从分位数样本里静默删除。

## 第一批输入的适用范围

第一批为已验收快速八字归档内的 **48 个合成独立快照**：12 个轨迹阶段 × 4 个首步控制间隔（12、16、20、24 ms）。它们是独立 Python 数值验证生成的 reference-following 状态与平移过的命令历史，**不是那次快速飞行的 1,198 个真实闭环求解记录**。

这些快照用于在同一冻结输入上比较 x86/ARM、CPU/CUDA 的数值与耗时。每次调用恢复相同的历史计划、命令历史、状态、参考预览和 `dt`，避免热启动条件改变。它们不代表连续闭环状态演化，也不能单独证明真机飞行时间预算已合格。后续增加真实闭环记录、重新线性化压力和代表性后台负载时，应建立新的明确输入批次。

工作站准备工具核对以下关系：

1. 原始 ZIP 与已保存收据的字节数、SHA-256 一致。
2. 外部验收报告与 ZIP 内正式证书逐字节一致，原正式快速八字确已通过。
3. 48 个原始 JSON 均与原始 manifest 的 SHA-256 一致，历史数值验证报告也绑定同一 manifest。
4. 每个历史 C++ 结果绑定对应原始输入；命令与独立 Python oracle 在原有 `1e-7` 误差内一致。
5. 输入包中的 `config`、`memory`、`input` 保持原始 JSON 数值；不修改历史、计划或生产保护。

输入 oracle 的 `command` 来自原独立 Python 结果；`position_reference`、`velocity_feedforward` 来自之前已经验收的历史 C++ 结果，不用本次 NX 输出生成自己的参考答案。对应原始输入、历史结果、完整原 oracle 和报告的 SHA 都保存在 provenance 中。比较误差沿用原要求：命令和速度前馈 `1e-7`，位置参考 `1e-9`。

代码包使用显式白名单，只包括 benchmark、Core、QP/QuadProg++、TS20 原生预测器及必要的源文件和许可。生产源码从原快速八字的 `accepted/launch/sources.zip` 逐字节取出，新 benchmark 使用当前工作区源码。工作区版本与归档版本有任何字节差异都会记录；不把源码尾部空行不同宣称为字节相同。Eigen、JsonCpp、OpenSSL 等系统依赖先只读检查，不自动下载安装。

## 工作站操作

所有下列输出目录均在工作站。准备目录和结果目录必须是新目录，避免旧结果混入。

```powershell
python scripts/prepare_x650_orin_benchmark.py `
  --output artifacts/orin_nx_mpc_20261024/prepared_v1 `
  --warmup-rounds 2 --measured-rounds 20 --cpu -1
```

输出包括 `input.json`、`preparation.json` 和 `code_only_sources.tar.gz`。这些文件留在工作站；其中源码包供 RAM 构建使用，`input.json` 只通过 stdin 传入 benchmark。源码包不包括输入数据、仿真模型网格或历史诊断结果。

板上代码构建与依赖检查独立完成后，把实际构建出的四个 ELF 的 SHA-256 填入调用参数。接口允许不绑定CPU；首轮在确认0–7在线后选择CPU2，沿用原MAXN和动态调频，不改变板卡功耗策略。

```powershell
python scripts/run_x650_orin_benchmark.py `
  --input artifacts/orin_nx_mpc_20261024/prepared_v1/input.json `
  --input-sha256 <preparation.json 中的 input.sha256> `
  --output artifacts/orin_nx_mpc_20261024/cpu_run_v1 `
  --host 192.168.30.145 --user jetson --port 22 `
  --identity-file C:/Users/Liym0817/.ssh/id_ed25519 `
  --known-hosts-file artifacts/orin_nx_mpc_20261024/known_hosts `
  --binary /home/jetson/<专用代码目录>/x650_mpc_memory_benchmark `
  --library-dir /home/jetson/<专用代码目录>/lib `
  --predictor /home/jetson/<专用代码目录>/lib/libx650_benchmark_predictor.so `
  --binary-sha256 <实际 ELF SHA256> `
  --core-sha256 <实际 Core SHA256> `
  --optimization-sha256 <实际 QP SHA256> `
  --predictor-sha256 <实际 TS20 SHA256>
```

Runner 启用 SSH 主机密钥核验和非交互密钥登录；不保存或接收密码。它只创建一个 SSH 进程，用二进制 stdin 传入原输入字节，收到完整 stdout 后将原始字节、stderr 和传输记录原子保存到工作站。报告同时核对输入 SHA、case 顺序和数量、运行参数、provenance 以及实际加载的四个 ELF SHA。进程崩溃或超时也在工作站留下失败记录，不能当成通过。

工作站还逐一核查首次、预热和正式调用的 case/round 顺序、完整样本数量、返回或异常状态、输出尺寸与有限性，并用原输入 oracle 重新计算误差。所有原始调用均参与计数和 nearest-rank 分位数重算，再核对板上报告的统计和 `all_calls_valid`；不单独相信程序返回的顶层通过字段。当前 CPU 快照基准若宣称已实现 CUDA、硬件闭环或已经测得完整控制栈延迟，也会被 runner 拒绝。

## 输出判读与下一阶段

先核对 `all_calls_valid`、每种失败数、样本总数和实际代码身份，再看正式测量的中位数、P95、P99、最大值与超期次数。首轮与预热不混入正式分位数，但仍保留原始样本。失败调用有耗时，且不能被统计悄悄排除。某组失败时，其“计算较快”的平均值不能代替正确性验收。

工作站保留板卡型号、系统/编译器/依赖版本、CPU/GPU 频率、温度与功耗模式信息作为测试条件。温度或频率采样也应在内存缓冲后回传；如单独增加后台采样，必须标明它是否与本轮测量并行及其开销。

CUDA 阶段先保留相同输入、双精度与误差标准。GPU event 算子时间和“CPU 输入可用到控制结果可安全读取”的完整调用时间分别报告；后者必须包含必要的数据交接、kernel 启动和同步。只统计异步 kernel 发射返回时间没有意义。小规模 75 变量 QP 不保证 GPU 更快，应由板上实际热点决定迁移哪些算子。

最后才把 NX 接回服务器仿真闭环：分别记录 NX Core 计算、NX 本地输入/输出处理、通信排队、状态年龄与实际执行端采用时间。服务器和 NX 的单调时钟不能直接相减，跨机指标必须明确时钟映射或采用各机本地同钟边界。这一步是全流程迁移参考，不能把本地 CPU benchmark 的通过当成真机闭环已经通过。

本地工具回归测试：

```powershell
python -m unittest discover -s tests/unit -p test_x650_orin_benchmark_tools.py -v
```
