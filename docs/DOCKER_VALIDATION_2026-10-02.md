# Docker 验证报告 — 2026-10-02

本报告记录 `simulation` 分支 Docker 打包的自动化验证结果。

## 验证方式

GitHub Actions workflow：

```text
.github/workflows/docker-smoke.yml
```

成功运行：

```text
run id: 37022060653
commit: a52dab31547f6547e137709dfc1a4b30d4f89edd
conclusion: success
```

## 已通过步骤

1. 从 `ubuntu:22.04` 构建完整 AMSIM 镜像。
2. 在镜像中安装 ROS 2 Humble、Gazebo Harmonic、Micro-XRCE-DDS Agent 等依赖。
3. 在镜像构建阶段完成固定 PX4 commit 的 SITL 构建。
4. 在镜像构建阶段完成冻结 `px4_msgs` 的 ROS 2 构建。
5. 容器内 `amsim setup --check-only` 通过。
6. 容器内 `amsim doctor` 通过（CI 无 GPU，因此显式使用 `AMSIM_REQUIRE_GPU=0`）。
7. 容器内完整 `hover --headless` 飞行场景通过。
8. 容器内软件 HIL transport smoke 通过。

因此当前可以把 **Docker headless 仿真执行链** 标记为已验证。

## GPU Docker 边界

GitHub hosted runner 没有 NVIDIA GPU，所以本次 CI 没有验证：

- `docker run --gpus all`
- NVIDIA Container Toolkit
- 容器内 VirtualGL GPU Gazebo GUI

仓库已经提供对应 Docker/Compose 配置，但 GPU GUI 容器路径仍需要在一台安装了
Docker + NVIDIA Container Toolkit 的 GPU 主机上做最终验证。

这与宿主机原生 VirtualGL 验证是两件事：宿主机 GPU GUI 已通过 clean-room
测试；尚缺的是“GPU 透传进 Docker 后的 GUI”这一项。
