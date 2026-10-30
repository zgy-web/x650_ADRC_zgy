# POWER验证与首飞参数表：准备记录

2026-10-30，本轮按用户要求准备无桨 POWER 独立供电验证与首飞参数变更表。参数表已形成，硬件供电验证尚未完成。

第一次固定只读采集在 COM13 打开后未收到任何字节，结果为 `No verified NSH prompt`，串口已关闭。没有读到本轮新供电或参数数据，没有发参数写入、解锁、模式或电机命令。完整失败记录保留在电脑 `artifacts/px4_power_firstflight_20261030/initial_inventory/`。

新工具 `scripts/px4_power_readonly.py` 分别记录 USB＋POWER 和仅POWER窗口，检查实际锁定状态、POWER／USB位、5V、过流及RC状态；通过电脑单调时钟与飞控时间戳的对应关系检测窗口之间的复位。combined窗口单独通过不会输出独立供电通过结论。电池监测与供电结论分别记录，缺失读数不填0。

`scripts/px4_first_flight_readonly.py` 增加12项固定参数读取，用于核对手动限制、整体设置覆盖和电源监测。两套工具共 **33项离线检查通过**。合成测试涵盖两阶段成功路径和会造成错误放行的反例；不是实机POWER结果。

参数提案基于同日15:23历史快照和飞控实际版本的源码。源码下载过程中未找到的路径（404）也保留在清单；最终参数定义来自已找到的C文件及Commander `module.yaml`，没有以最新主分支或默认值补齐未知实读。`pinned_source_manifest.json`记录固定版本、来源和源码哈希；源码缓存不自动安装到任何设备。

`summary.json`关联测试XML、两份原始失败记录、源文件和提案的SHA256。硬件状态明确为 `NOT_COMPLETED`，`independent_power_verified=false`，`flight_ready=false`。无参数改动、无NX配置改动。

下一步待现场确认电池连接状态、POWER模块型号和恢复FMU Debug实际回读，随后按 [POWER流程](../../X650_POWER_VALIDATION.md)完成两阶段；[首飞参数表](../../X650_FIRST_FLIGHT_PARAMETERS.md)在新的实读和具体确认前不应用。
