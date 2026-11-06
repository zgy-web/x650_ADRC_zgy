# Orin NX 新板 UART 修复实例

2026-11-06 已在用户确认的同款达妙载板、Jetson Linux R36.5.0 上部署，并通过重启后设备树检查和20秒被动接收。此目录是绑定已审核启动配置的实例，不是任意NX可直接运行的通用安装器。

`apply_uart_fix.py` 为指定 UARTA 增加 `iommus = <&smmu_niso0 0x04>`；保留 DMA、驱动、时钟和引脚配置。它分别在基础DTB及运行树副本中合并补丁，要求只有目标属性变化，再核对配置和源码散列。

部署路径为 `/home/orinnx/flyarm/hw_uart_fix/20261106_v1/`，只放 Python 与 DTS 两份源码。检查命令：

```bash
python3 -B /home/orinnx/flyarm/hw_uart_fix/20261106_v1/apply_uart_fix.py --check
```

安装需要用户明确批准，并由其在自己的终端输入sudo密码：

```bash
sudo python3 -B /home/orinnx/flyarm/hw_uart_fix/20261106_v1/apply_uart_fix.py --apply-and-reboot
```

脚本依次备份原启动配置、安装387字节overlay、回读验证，再选择包含明确 `FDT` 和 `OVERLAYS` 的新启动项并重启。它不改PX4或MPC，不记录实验日志。原 `primary` 项保留，属于人工回退，不是自动恢复。系统仍能登录时可恢复原配置并重启：

```bash
sudo python3 -B /home/orinnx/flyarm/hw_uart_fix/20261106_v1/apply_uart_fix.py --rollback-and-reboot
```

不同启动配置、基础DTB、源码、备份或无关手动修改会被拒绝，不能删除散列检查后强行安装。已有原配置备份和NX可用恢复入口应在操作前核对；测试全程保持电机动力断开。源码留在用户flyarm目录，必要启动文件与备份位于 `/boot`，实验记录只存电脑。

本次完整运行树对照中，UART映射正确；CPU、散热、USB、网络/PCIe、内存保留区等所查静态属性没有未解释变化。显式FDT路径还移除了三个已知 `/chosen/odm-data` 布尔标签，不能声称整棵树只改变一行，或所有外设均已验收。实际数据及边界见 [验收记录](../../../docs/validation/nx_uart_mavlink_20261106/README.md)。

单元测试使用脱敏启动配置fixture，运行时仅对测试模块的 `CONFIG_SHA` 做隔离替换；安装脚本保留真实已审核散列，未放宽保护。运行：

本目录Git属性固定Python/DTS源码为LF，避免Windows检出改变安装散列；fixture关闭换行转换并保留结尾空格与空行，以检验实际配置的字节保留行为。

```bash
python3 -B tests/unit/test_nx_uart_new_board_fix.py
```

依据：[NVIDIA R36.5 UART DMA映射修正](https://forums.developer.nvidia.com/t/solved-uart-serial-port-not-working-after-upgradint-to-jetpack-6-2-2-orin-nano-nx/363837/7)、[rootfs overlay加载条件](https://forums.developer.nvidia.com/t/place-custom-overlay-file-via-extlinux-conf/298287/3)。
