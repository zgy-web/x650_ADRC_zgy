# Docker 使用说明

## 构建

在仓库根目录执行：

```bash
docker build -f deploy/docker/Dockerfile -t safe-aerial-manipulator-testbed:simulation .
```

镜像会在构建阶段完成 ROS/Gazebo 主机依赖安装、固定 PX4 commit
构建以及冻结的 `px4_msgs` 构建，因此运行容器时不需要再次编译。

## Headless 验证

```bash
docker run --rm \
  -e AMSIM_REQUIRE_GPU=0 \
  safe-aerial-manipulator-testbed:simulation \
  ./scripts/amsim setup --check-only
```

完整 headless hover：

```bash
docker run --rm \
  -e AMSIM_REQUIRE_GPU=0 \
  safe-aerial-manipulator-testbed:simulation \
  ./scripts/amsim run hover --headless
```

## NVIDIA GPU + GUI

宿主机需要先安装 NVIDIA 驱动与 NVIDIA Container Toolkit。

```bash
docker run --rm -it \
  --gpus all \
  -p 127.0.0.1:6080:6080 \
  -e AMSIM_DESKTOP_BIND=0.0.0.0 \
  safe-aerial-manipulator-testbed:simulation \
  bash
```

容器内：

```bash
./scripts/amsim desktop-start
./scripts/amsim render-status
./scripts/amsim run hover
```

然后通过宿主机的 `127.0.0.1:6080` 建立 SSH/云平台隧道访问 noVNC。

`compose.yaml` 提供同等的 GPU 配置：

```bash
docker compose -f deploy/docker/compose.yaml build
docker compose -f deploy/docker/compose.yaml run --service-ports amsim bash
```

Docker 不负责安装或修改宿主机 NVIDIA 驱动。

## 验证状态

Docker headless 路径已由 GitHub Actions 完整验证，包括镜像构建、
`setup --check-only`、`doctor`、完整 `hover --headless` 与软件 HIL smoke。

GPU GUI 容器路径仍需要 Docker + NVIDIA Container Toolkit 的 GPU 主机验证。
详见 `docs/DOCKER_VALIDATION_2026-10-02.md`。
