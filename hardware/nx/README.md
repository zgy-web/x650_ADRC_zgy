# NX R36.5 UART1 DMA 映射修复

本目录针对已核对的达妙载板 / Orin NX：物理 UART1 对应 `serial@3100000`、`/dev/ttyTHS1`，系统为 Jetson Linux R36.5.0。它不是通用刷机或串口配置工具。

`flyarm-uart1-iommu-r365.dts` 只增加 UARTA 的 `iommus = <&smmu_niso0 0x04>`。该映射来自 [NVIDIA 工程师给出的 R36.5 修复](https://forums.developer.nvidia.com/t/solved-uart-serial-port-not-working-after-upgradint-to-jetpack-6-2-2-orin-nano-nx/363837/7)。启动配置采用 R36.5 官方支持的 [extlinux OVERLAYS](https://docs.nvidia.com/jetson/archives/r36.5/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html#device-tree-overlays)，保留原来的内核、initrd、启动参数。

**必须同时指定 FDT。** NVIDIA 明确说明 [只有 OVERLAYS 而无 FDT 时不会加载 rootfs 补丁](https://forums.developer.nvidia.com/t/place-custom-overlay-file-via-extlinux-conf/298287/3)。首版安装遗漏 FDT，实际重启后属性未生效；后续脚本识别该版本并补齐此项。修复启动项使用本机已核对的 `/boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb`，散列 `4c533869e70e6b0fe1426ad2f00e25b29450dab090997fa42385b8b16a55869f`，不改写该文件。原 `primary` 项继续使用原有的基础 DTB 选择方式。

## 部署和权限

仅在用户批准后，把两个源文件放入 `/home/jetson/flyarm/hw_uart_fix/v1/`。源码和脚本散列须与电脑核对。NX 的安装脚本只输出到终端，不保存实验日志；内核及 sudo 自身的系统记录不在此范围内。

普通用户只读预检：

```bash
python3 -B /home/jetson/flyarm/hw_uart_fix/v1/apply_uart_fix.py --check
```

用户授权的安装与一次重启，在 NX 自己的终端执行：

```bash
sudo python3 -B /home/jetson/flyarm/hw_uart_fix/v1/apply_uart_fix.py --apply-and-reboot
```

该脚本会先检查 R36.5、源码及已审阅启动配置的散列、串口未占用、DMA 通道与 SMMU 符号，并核对基础 DTB 散列、型号和 compatible；再在匿名内存中分别向运行树副本和基础 DTB 合并补丁，确认补丁没有引入其他变化。通过后依次进行：

1. 原启动配置备份为 `/boot/extlinux/extlinux.conf.flyarm-uart1-before`。
2. 安装 `/boot/dtb/flyarm-uart1-iommu-r365.dtbo`。
3. 保留原 `primary` 项，新增并默认选中 `flyarm_uart1_iommu`，明确列出 FDT 与 OVERLAYS。
4. 校验落盘内容并同步，再请求重启。

配置、源码或备份不符合预期时停止；不自动放宽散列检查。脚本不会访问 PX4，不发送电机或 Offboard 指令。操作期间电调动力保持断开。

## 回退与验收

原 `primary` 是可手动选择的回退项，不是自动故障恢复。正常进入系统后，可在 NX 终端恢复原配置并重启：

```bash
sudo python3 -B /home/jetson/flyarm/hw_uart_fix/v1/apply_uart_fix.py --rollback-and-reboot
```

回退只在当前配置与脚本生成的版本或原版本完全一致时执行，不覆盖后来的其他修改。未被引用的补丁文件保留，不删除其他启动文件。若不能进入系统，须从本地启动菜单选择原 `primary`。

重启后先确认 boot ID 已变化且运行设备树出现正确映射，再使用电脑上的 `scripts/nx_uart_mavlink_readonly.py` 做被动接收和固定只读参数查询。该脚本只允许一次 `PARAM_REQUEST_READ(MAV_0_CONFIG)`，无参数写入、解锁、模式切换或设定值；收到的完整记录保存在电脑指定路径，串口属性在退出时恢复。它报告消息频率和接收间隔，不把 SSH 时间当成串口单向延时。

串口验证必须同时核对有效 PX4 心跳、解锁位、稳定后的 CRC、对应参数回复和同一采集窗口内的内核错误。通过这一项仅说明 UART MAVLink 基础通信可用，不代表 MPC 控制、Offboard 接管或此前 PX4 USB Hard Fault 已验收。

## 对应代码

- `apply_uart_fix.py::make_config`：保留原启动项、生成包含 FDT 和 OVERLAYS 的补丁启动项。
- `apply_uart_fix.py::verify_live_tree`：内存编译与完整设备树差异检查。
- `apply_uart_fix.py::verify_base_dtb`：核对现有基础 DTB，检查合并差异；不修改该文件。
- `apply_uart_fix.py::main`：备份、写入顺序、回读、重启与回退。
- `tests/unit/test_nx_uart_boot_fix.py`：启动项保留和异常配置拒绝。
- `scripts/nx_uart_mavlink_readonly.py`：PC 采集入口与 NX 内存串口探测器。
- `tests/unit/test_nx_uart_mavlink_readonly.py`：真实 MAVLink 编解码、错误回复及 CRC/内核错误拒绝。

实机结果记录在 `docs/validation/nx_uart_mavlink_20261030/README.md`，必须区分源码准备、启动修复生效与通信验收三个状态。
