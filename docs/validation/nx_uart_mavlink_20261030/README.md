# NX UART1 / PX4 TELEM1 修复与只读通信验收

日期：2026-10-30。状态：**UART1 修复已生效，被动接收与一次只读参数往返均通过。未进行控制设定值、Offboard 或飞行验收。**

## 最终结果

第二次重启后，运行树出现正确的 `iommus = <0x04 0x04>`。启动配置、补丁和原配置备份的散列均符合预期。串口测试分为独立的20秒被动接收与30秒参数读取两段：

| 指标 | 20秒被动接收 | 30秒只读参数往返 |
| --- | ---: | ---: |
| 实际窗口 | 20.013 s | 30.025 s |
| 接收字节 | 12,803 | 19,186 |
| 当前 MAVLink 方言已识别并校验的帧 | 307 | 463 |
| PX4 心跳 | 20 | 30 |
| 帧同步后解析错误 | 0 | 0 |
| 同窗口 SMMU / VPR / Route Sanity 错误 | 0 | 0 |
| NX 发送 | 0字节 | 1条固定 PARAM_REQUEST_READ，28字节 |
| MAV_0_CONFIG 回复 | 未请求 | PX4 1/1，INT32按字节解码为101，与COM13回读一致 |
| 串口属性恢复、关闭 | 通过 | 通过 |

两次刚打开持续发送的串口时分别出现3条、1条帧同步前解析记录，第二段仅为 Bad prefix；首段另有一次同步前的 CRC 错误。两段在收到首个已知有效帧后均没有解析错误。当前电脑的 MAVLink 方言未覆盖消息ID 8、380、411、436，分别记录22帧、32帧 UNKNOWN；这些帧不计入上述有效帧，也不宣称其扩展CRC已经校验。后续正式 C++ 接口仍应使用与飞控固件匹配的消息定义。

一次参数读取的本机观测往返为12.364 ms，包含串口传输、飞控处理/排队及NX用户态接收；只有一个样本，不作实时性保证，更不代表 MPC 求解耗时。两段都没有参数写入、解锁、模式切换或设定值发送。

最终COM13回读：飞控固件匹配、未解锁、所有DShot输出为零、DDS客户端未运行。飞控时间连续增加，没有随两次NX重启重新计时。NX串口没有遗留使用进程，SSH与NoMachine监听端口均已恢复。所有实验记录保存在电脑；NX仅保留修复代码和必要启动配置/备份。

## 当前范围和下一步

本轮确认的是**UART MAVLink 基础双向通信**。PX4仍为57600 baud、Normal模式，MAV_0_RATE=1200 B/s；实收 ATTITUDE 约4.54 Hz、四元数约3.03 Hz、LOCAL_POSITION_NED约0.303 Hz。这是当前流配置下的频率，不是UART或NX的能力上限，也不满足将这条状态流直接作为现有高频MPC闭环输入的验收要求。

接下来应准备匹配固件的 C++ MAVLink 收发适配器，明确所需消息、时间戳、带宽和刷新频率，并在动力断开条件下验证设定值回读及接管退出。涉及NX/PX4配置修改仍按用户要求先确认。此前 USB 临时协议切换导致的 PX4 Hard Fault 隔离继续保留，本次通信通过不能证明该故障已修复。

## 首次故障记录

首次通信检查仅做只读接收。NX UART 未发送任何字节；PC 经 COM13 发送的只有固定状态与参数查询。没有解锁、模式切换、设定值、电机指令或参数写入。该轮 NX 未安装软件或保存实验输出；临时 UART 设置结束后恢复，串口已关闭。内核自身的错误日志是系统自动产生的日志，已读取到电脑分析。随后获得用户授权，将修复源码和安装脚本部署到指定 flyarm 用户目录；实验记录仍只存电脑。

### 修复前实测

| 项目 | 结果 |
| --- | --- |
| PX4 | 1.17.0，提交 d6f12ad1c4f70ad3230afd7d86e971421e02fef4 |
| TELEM1 | /dev/ttyS6，57600，MAVLink 2，Normal，最大配置发送速率1200 B/s |
| NX | Jetson Linux R36.5.0；达妙物理 UART1 对应 serial@3100000、/dev/ttyTHS1 |
| 接收窗口 | 20.009秒，12732字节，其中12235字节为零 |
| MAVLink | 有效帧0；不能认定心跳、状态或双向通信已经通过 |
| NX 内核 | 接收窗口内记录40行 SMMU context fault、20行 VPR violation、20行 Route Sanity error，另有日志限流提示 |
| UART 配置 | DMA rx/tx已开启；UART节点缺少 iommus，GPCDMA节点的映射为 `<0x04 0x04>` |
| 结束状态 | 飞控未解锁，4路输出为零；两次回读的飞控启动时间连续增加；NX 串口无占用，接收结束后的检查未见新增同类内核错误 |

飞控重启后的实际 kill 标志为 False，本轮未操作遥控开关；后续执行任何动力相关测试前须重新核对实际停机标志，不能把 SB 的机械位置当成该标志已置位。

## 诊断与边界

系统版本、损坏数据特征、SMMU内核错误及缺失设备树属性，与 NVIDIA 已确认的 R36.5 UART DMA 问题一致，构成当前阻塞的强证据。这不证明所有线序均已正确，也不等同于上一次 PX4 USB 协议切换 Hard Fault 的根因已经查清。当前还不能进入 Offboard 或飞行验收。

官方依据：

- [NVIDIA 工程师针对 R36.5 的 UART DMA 映射修正](https://forums.developer.nvidia.com/t/solved-uart-serial-port-not-working-after-upgradint-to-jetpack-6-2-2-orin-nano-nx/363837/7)
- [同类 UARTA 接收损坏及修复反馈](https://forums.developer.nvidia.com/t/dma-on-dev-ttyths1-corrupts-receiving-data/369191)

另发现 TELEM1 的 `MAV_0_FLOW_CTRL=2`（自动），运行时显示 ON；物理连线只有 TX/RX/GND。它不是上述内存映射错误的替代解释。修复后仍应单独核对三线通信的流控状态、消息频率及双向参数读取，不能直接将低速 Normal 流用作已验收的 MPC 状态通道。

## 修复候选与授权范围

候选源文件：`hardware/nx/flyarm-uart1-iommu-r365.dts`。唯一目标是在现有 UARTA 节点增加 `iommus = <&smmu_niso0 0x04>`，保留 DMA、时钟、驱动、引脚复用及其他节点原值。

已使用 NX 既有 dtc/fdtoverlay，在匿名内存文件中编译补丁，并与运行中设备树的只读副本合并。排序后的完整设备树差异只有 UARTA 的一行新增属性。此验证没有把补丁加载到运行内核，没有改 /boot，没有重启，也没有证明实物通信已修复。

用户已明确批准修复及一次 NX 重启。源码和安装/回退脚本已放在 `/home/jetson/flyarm/hw_uart_fix/v1/`，本机只读预检通过；完整设备树合并仍仅新增上述一项属性。安装脚本的五项本地测试通过，覆盖原启动项及参数保留、未知修改拒绝、备份检查。`jetson` 账户没有免密 sudo，由用户在 NX 终端完成安装。首次重启后 SSH 恢复、boot ID 改变，配置和补丁散列均正确；但运行树仍缺少属性，不能判为修复成功。

原因是首版仅写 `OVERLAYS`，遗漏了本机启动路径所需的显式 `FDT`。[NVIDIA 官方说明](https://forums.developer.nvidia.com/t/place-custom-overlay-file-via-extlinux-conf/298287/3)确认 rootfs overlay 需要 rootfs 基础 DTB。现已核对本机 R36.5 的 `kernel_tegra234-p3768-0000+p3767-0000-nv.dtb`：model、compatible 与运行板一致；对照完整属性，运行树差异集中在启动时的 CPU 裁剪、内存 carveout、chosen、固件 overlay 等区域，UART1 节点原值一致。对该基础 DTB 合并补丁也仅增加目标 iommus 属性。不使用导出的运行树作为下次启动基础，避免携带本次启动的内存地址。

第二版已为修复启动项补充该 FDT 路径，不修改原基础 DTB 文件。原 `primary` 启动项仍完整保留，新项为 `flyarm_uart1_iommu`；这是人工回退入口，不宣称自动恢复。原配置 SHA-256 为 `5974f443a57f400acb31eaca5d5e74ec31ff7c97dc46baf79669df9a887025fc`；首版为 `817325ebd59af73eb422da89a83b11adca0383d32e1999ced66e9c0ce92ab2fc`；第二版实际为 `558033e178661ff468acef15094d6fa557c9361781b4da7d83ccc58294a8477f`。用户在NX终端执行第二次 sudo 安装和重启后，已通过真实设备树与通信检查。操作方法见 `hardware/nx/README.md`。

对照两次启动后的完整设备树属性，硬件节点变化仅为 UART1 的 iommus；另有 `/chosen/odm-data` 下三个标签（gbe-uphy-config-8、hsio-uphy-config-0、hsstp-lane-map-3）在显式FDT路径下未出现。因此不能笼统声称两次启动的整个设备树只有一处差异。已有CPU裁剪、内存 carveout、其他外设节点属性的对照未发现新的差异；本轮UART及网络均实测可用，未扩展为所有NX外设验收。

重启后已依次完成：新属性确实加载、采集窗口无新增 SMMU 错误、有效心跳及姿态状态、只读参数往返。MAVLink 控制接口与无动力台架接管测试另行准备。

## 本地证据

完整记录位于电脑项目 `artifacts/nx_uart_mavlink_20261030/`：

- `uart_rx_passive_v1.json`：原始接收字节、NX接收时间与解析结果。
- `nx_uart_inventory_v1.json`：版本、设备树属性、启动配置和限定时间内的内核错误。
- `fc_readonly_130327.json`、`fc_flowcontrol_v1.json`、`fc_final_readonly_v1.json`：COM13独立回读。
- `overlay_verification.json`、`candidate_device_tree.diff`：候选补丁的完整差异检查。
- `flyarm-uart1-iommu-r365.dtbo`：387字节候选，SHA-256 `6aadc1c08fe536cf20b90018867a95bf671f3736c41fbb2aacf64e7009709226`。
- `probe_uart_readonly.py`：首次探测方法；随后仅改进结果摘要与失败退出码，首轮源文件散列仍保存在首轮记录中。
- `deployment_preflight_v1.json`：用户目录部署散列、原启动配置散列、内存合并差异及未执行系统写入的状态。
- `post_reboot_preflight_v1.json`：首版配置已安装、NX 已重启，但目标属性尚未生效的真实回读。
- `base_dtb_audit_v1.json`、`deployment_preflight_v2.json`：基础 DTB 完整属性对照、第二版源码散列及两份设备树合并检查。
- `post_reboot_preflight_v2.json`、`base_dtb_audit_v2.json`：第二次启动已加载映射，以及启动后属性对照。
- `uart_rx_after_fix_v2.json`、`uart_param_read_after_fix_v2.json`：修复后两段原始串口采集、解析及验收结果。
- `fc_before_uart_probe_v2.json`、`fc_final_after_fix_v2.json`：第二次重启前后与最终的独立飞控回读。
- 本目录 `summary.json`：可复核的精简统计与原始文件散列。

修复后的探测使用项目 `scripts/nx_uart_mavlink_readonly.py`：新增固定只读报文白名单、对应参数回复匹配、同窗口内核错误核对及落盘验收字段。相关六项测试通过，连同安装脚本共11项本地测试通过；真实通信结果如上表所示。

诊断数据不用于评估 MPC 求解延时，SSH 网络耗时也不计为串口或算法执行时延。
