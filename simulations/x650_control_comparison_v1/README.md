# X650 闭环机械臂：PID、LADRC、MPC 位置外环对比

阶段 A 和 B 的执行进展、验证证据和新批次入口见[执行记录](STAGE_AB.md)。

本实验使用 `amsim_urdf_x650_portable_v1_flight`，比较安装机械臂后的飞机位置控制。机械臂是两个主动关节驱动的闭环连杆机构，关节名称为 `upper_arm_joint` 和 `fore_crank_joint`。

三个算法都输出飞机 ENU 三轴速度指令，经过相同 PX4 速度、姿态和角速度内环。机械臂使用相同关节控制器和动作时序。因此结果对应“相同 PX4 内环下的位置外环算法对比”，不代表替换了 PX4 内部 PID，也不代表完成了飞机与机械臂末端协同控制。

本轮结果分析、证据限制与下一阶段实验安排见 [ADRC 仿真总结与下一阶段计划](SUMMARY_AND_NEXT_PLAN.md)。

## 环境安装

在仓库根目录运行：

```bash
./scripts/setup_x650_comparison.sh
```

该脚本在仓库 `.amsim/comparison-venv` 创建独立 Python 3.10 环境，安装 `requirements.txt` 的固定版本，继承本机 ROS 2 Humble 与 Gazebo Harmonic Python 绑定。它不会更新原 `amsim-venv` 或系统软件包。NumPy、SciPy、OSQP、matplotlib 和 pymavlink 从新环境导入；脚本还实际求解一个小型 QP 并验证 ROS/Gazebo 导入。

运行时设置 `PYTHONNOUSERSITE=1`，防止用户目录中的 NumPy 覆盖实验版本。本机缺少 `ensurepip`，脚本使用 `venv --without-pip --system-site-packages` 并继承系统 pip；依赖仍然安装到新虚拟环境内。

## 运行与开发顺序

所有命令均在仓库根目录执行。统一入口 `scripts/x650-compare` 自动加载本机 ROS/Gazebo 配置，并使用独立实验解释器；无需手工激活虚拟环境。默认无界面，单次实验添加 `--gui` 可打开 Gazebo。每条 `run` 命令都会实际执行仿真起飞、测试和降落。

先执行小幅 Z 轴验证：

```bash
./scripts/x650-compare run --algorithm pid --case z_smoke
```

然后辨识速度指令响应。`identify` 按 X、Y、Z 顺序施加正负 0.1 m/s 指令；`--algorithm pid` 标识此次运行，辨识场景实际使用专用激励，不将该数据算作 PID 性能。

```bash
./scripts/x650-compare run --algorithm pid --case identify
```

命令最后输出实际结果目录。将下面的 `<identifydir>` 替换为该目录名称；只有完整结束的辨识实验可以用于拟合。

```bash
./scripts/x650-compare fit "runs/<identifydir>" --replace
```

`fit` 使用正向脉冲及随后的零指令段，对完整递推速度轨迹拟合每轴增益 `gain` 和有效时间常数 `tau`；后续负向脉冲独立用于验证，不参与参数选择。拟合模型为 `p_dot = v`、`tau * v_dot = -v + gain * u`，偏置和纯延迟固定为零。

这里的 `tau` 用于近似当前 PX4 内环与通信链共同表现出的响应过程，纯延迟的影响被近似吸收到有效时间常数中。它不是单个电机或真实执行器的物理时间常数，也没有独立辨识物理纯延迟。采样采用控制接收时间，PX4 源时间戳用于去除重复测量；输入采用实际发送指令并按零阶保持递推。

`--replace` 明确替换默认 `response_model.json`；也可用 `--output /path/to/response.json` 保存独立版本。参数触及拟合边界或任一轴负向脉冲验证 RMSE 达到 0.08 m/s 时，模型不通过资格检查；报告会保留，命令返回失败。正式批次还检查响应模型是否绑定当前物理模型哈希。

独立验证各控制器及场景，例如：

```bash
./scripts/x650-compare run --algorithm ladrc --case arm
./scripts/x650-compare run --algorithm mpc --case step
./scripts/x650-compare run --algorithm pid --case force --gui
```

`--algorithm` 可选 `pid`、`ladrc`、`mpc`。`--case` 可选 `hover`、`step`、`arm`、`force`、`z_smoke`、`identify`。单次 `run` 允许使用临时模型进行开发；它不意味着已经取得正式对比资格。

尚未完成辨识时，可运行明确标记为开发用途的小批次：

```bash
./scripts/x650-compare batch --development --cases z_smoke --repeats 1
```

辨识及调参通过后启动完整正式矩阵：

```bash
./scripts/x650-compare batch
```

默认执行 `hover step arm force`，每个场景下的三种算法各重复三次，共 36 次。批次开始时复制并计算算法配置、响应模型及 Python 源码哈希。每次结果写入批次 `manifest.json`，同时保留独立日志；恢复批次时拒绝混入修改后的源码或配置。

用实际批次目录替换 `<batchdir>`，可恢复尚未完成的实验或重新生成报告：

```bash
./scripts/x650-compare batch --resume "runs/<batchdir>"
./scripts/x650-compare report "runs/<batchdir>"
```

恢复会跳过已经结束的尝试，包括已有的失败记录；重新尝试中断的实验时保留中断记录。完整复测或修改算法之后，应新建批次。

`run` 和 `batch` 都支持独立配置，适合保存不同调参方案：

```bash
./scripts/x650-compare run --algorithm ladrc --case hover \
  --config /path/to/controller_config.json --model /path/to/response_model.json
```

默认配置文件为本目录 `config.json`，默认响应模型为 `response_model.json`。这里 `--model` 指简化的速度响应模型 JSON，不是 URDF。物理模型身份由场景和配置中的模型名、哈希共同固定。可以在独立 JSON 中调整算法参数；公共速度、变化率、关节速度限制及精度门槛属于本版固定协议，运行器拒绝改变这些值。运行 `./scripts/x650-compare run --help` 或 `batch --help` 可查看完整参数。

## 算法与公共接口

| 算法 | 实现 | 关注点 |
|---|---|---|
| PID | 位置比例、积分，基于估计速度的微分，积分抗饱和 | 稳态误差、超调和恢复 |
| LADRC | 二阶控制通道与三阶扩张状态观测器 ESO | 综合扰动估计、补偿和噪声影响 |
| MPC | 三轴位置/速度线性模型，OSQP 约束优化 | 预测、指令约束和计算耗时 |

公共反馈是 PX4 估计的位置与速度。Gazebo 真值只用于独立评分及状态检查，控制器不接收真值。飞机输出单位 m/s，位置为 ENU 米，时间为仿真秒；关节角度和速度为 rad 与 rad/s。

控制器提供：

```python
controller = make_controller(name, config, model)
controller.reset(position, velocity, applied_command)
raw_velocity = controller.step(dt, position, velocity, target)
# 运行器统一执行速度与变化率限制，并发送指令。
controller.accept_applied(sent_velocity, dt)
```

`accept_applied` 必须收到最后实际发送的指令，供 PID 抗饱和、ESO 预测和 MPC 上一次输入约束使用。算法内部计算值不能当作已经发送的值。算法诊断放在 `diagnostics`，不改变输出接口。

公共控制周期为 20 ms；速度逐轴限制为 ±0.4 m/s，速度指令变化率限制为 1 m/s²。机械臂速度逐轴限制为 ±0.15 rad/s。MPC 初始预测范围为 50 步、1 秒，将当前目标在预测范围内保持不变，不提前获知未来阶跃和扰动。

当前参数对水平轴采用较保守的带宽与控制权重，来源于独立的未建模延迟压力检查。数组顺序均为 X、Y、Z：

| 控制器参数 | 当前值 |
|---|---|
| LADRC 控制带宽 `wc` | `[1, 1, 2]` rad/s |
| LADRC 观测器带宽 `wo` | `[4, 4, 8]` rad/s |
| MPC 位置权重 `q_position` | `[9, 9, 25]` |
| MPC 速度权重 `q_velocity` | `[4, 4, 1]` |
| MPC 输入权重 `r_input` | `[2, 2, 0.2]` |
| MPC 输入变化权重 `r_delta` | `[5, 5, 1]` |

这些参数是初步验证配置，不表示已经搜索得到最优控制器。正式批次使用各自冻结的配置快照，修改配置后应创建新的批次。

LADRC 使用速度指令作为输入，不能直接套用“推力输入”的输入增益。简化辨识模型由位置积分和一阶速度响应组成；ESO 的估计值是该通道下的综合等效加速度扰动，单位 m/s²，不能直接解释为机械臂对机体施加了多少牛顿力。

本实验的 MPC 是新建的线性位置外环控制器。仓库原全身 MPC 绑定另一套串联二连杆运动学及不同指令契约，本实验未移植该控制器，也不使用其历史分数评价当前闭环机械臂。

## 实验时序

各场景从稳定进入外环控制后计时；起飞与降落另行记录，不混入实验误差统计。目标悬停高度为 1.5 m。

| 场景 | 时间与动作 |
|---|---|
| `hover` | 30 秒，目标位置不变，机械臂保持零位 |
| `step` | 35 秒；5–10 秒 X 目标 +0.2 m，15–20 秒 Y 目标 +0.2 m，25–30 秒 Z 目标 +0.2 m，其余时间返回初始悬停目标 |
| `arm` | 30 秒；5–13 秒两个主动关节目标为 `[0.15, 0.10] rad`，其余时间为零；飞机目标位置不变 |
| `force` | 30 秒；10–12 秒施加世界系 +X 方向 1 N 外力；飞机目标位置不变，机械臂保持零位 |

外力通过仿真世界的 `ApplyLinkWrench` 接口施加，按仿真时间触发。运行器应记录请求时刻与力值；这些字段表示施力请求，不是独立测得的实际力。正常结束与异常恢复均清除持续外力。

正式矩阵为 3 种算法 × 4 个场景 × 3 次重复，共 36 次。每次重新启动仿真，轮换算法顺序。模型辨识、调参、单轴功能检查与正式实验分别归档；正式实验开始后使用固定参数。

## 指标与结果解释

位置误差以 Gazebo 真值减去对应时刻的世界系目标计算。记录三维和分轴 RMSE、最大误差、稳态偏差、阶跃超调，以及恢复至三维误差 5 cm 内并连续保持 2 秒所需时间。测试结束仍未恢复时，恢复时间为空值，不能填零。

重复使用同一真值时间戳的控制记录不能当作独立真值样本。统计应去重并使用实际采样时间，保留 PX4 时间戳、真值时间戳和控制时间以检查异步反馈。

指令总变化量用于评估平滑性，不等同电机能耗。边界使用情况分开报告：`limited_fraction` 是公共适配器裁剪原始输出的时间占比；`speed_boundary_fraction` 和 `slew_boundary_fraction` 则依据实际发送指令，分别统计触及 ±0.4 m/s 速度边界和 1 m/s² 变化率边界的时间占比。MPC 可在内部满足约束而不触发适配器裁剪，因此不能把零裁剪率解释为未使用约束边界。

计算耗时报告中位数、P95、最大值以及超过 20 ms 的次数。未发送指令的终止调用也计入计算故障证据。求解失败、超时、遥测失效和飞行中止均要保留，不能通过静默切换其他控制器掩盖。

每次结果应同时说明运行是否完整结束，以及精度是否满足设定门槛。三次重复用于初步观察一致性，不能据此断言某种算法在所有工况都优于其他算法。当前推进与关节执行器参数属于仿真配置，比较结论不直接等同实机性能。

单次目录的 `result.json` 中，`execution_success` 表示测试及降落流程完成，`accuracy_accepted` 表示完整测量且精度门槛也通过。固定门槛为三维位置 RMSE ≤0.25 m、最大误差 ≤0.5 m、最大倾角 ≤15°；不要仅看命令退出码判断控制精度。

批量报告入口是批次目录的 `README_CN.md`。`summary.csv` 给出指标数量、均值和样本标准差，`summary.json` 保留逐次结果及哈希，`plots/<case>.png` 绘制位置误差、速度指令和计算耗时。失败或提前结束的实验也留在报告中；它们的测量时长不同，不能直接把这类 RMSE 与完整实验排名。

报告还独立检查 `evidence_accepted`：代码和配置快照哈希、物理模型身份、PX4 参数前后证据以及计划尝试的完成情况必须一致。正式批次在证据校验失败时返回非零状态，不能仅凭成功飞行认定对比有效。可信的失败实验可以通过证据检查，仍然计作运行失败；证据通过也不意味着控制精度达标。恢复前的历史中断会保留，并使该批次不能作为无中断的正式对比；需要另起新批次完成正式矩阵。

## 自动测试

初始实现已通过 40 项自动测试；后续增补后的数量及结果以命令输出为准。以下命令只运行数值、协议和报告测试，不启动 PX4 或 Gazebo：

```bash
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .amsim/comparison-venv/bin/python3 -m unittest discover \
  -s tests/unit -p 'test_x650_comparison_*.py' -v
```

覆盖控制器闭环跟踪、PID 抗饱和、ESO 初始化与扰动估计、MPC 限制及显式失败、未建模延迟压力、坐标转换与场景切换、辨识正负脉冲隔离、异步真值统计、未发送的失败调用以及冻结证据校验。这些数值测试与实际飞行结果分开记录；不将自动测试通过等同于完成 36 次正式飞行。

## 开发入口

- `controllers.py`：控制器、离散模型和算法诊断。
- `run_flight.py`：共用飞行流程、指令发送、外力与状态检查。
- `protocol.py`：场景时序、坐标转换和公共限幅。
- `cli.py`：单次运行、模型辨识、批次冻结与恢复。
- `identification.py`：有效一阶响应模型的递推拟合与负向脉冲验证。
- `config.json` / `response_model.json`：算法配置和简化响应模型。
- `analysis.py`：真值误差统计、批量报告和曲线。
- `requirements.txt`：独立数值环境版本。
- `../../scripts/setup_x650_comparison.sh`：环境安装与导入/求解检查。
- `../../scripts/x650-compare`：加载实验环境的统一命令入口。

本机已完成 40 项自动测试和 36 次正式仿真，全部通过运行、精度和证据校验，并完成进程清理修复后的飞行补测；原批次耗时存在后台桥接负载影响，结果、统计解释及开发阶段记录见 [VALIDATION.md](VALIDATION.md)。
