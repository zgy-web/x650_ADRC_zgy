# Host system requirements

Validated baseline: Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic / Gazebo Sim 8,
ros_gzharmonic bridge, Micro-XRCE-DDS Agent 2.4.2, Python 3.10,
git/cmake/colcon, and an NVIDIA driver when GPU rendering is requested.

On a clean Ubuntu 22.04 machine, run:

```bash
sudo ./deploy/host/install_ubuntu22.sh
./scripts/amsim setup --check-only
./scripts/amsim setup
```

The host installer deliberately does not install or modify the NVIDIA driver,
firewall, SSH configuration, or cloud-vendor services.

For remote GPU visualization, `amsim desktop-start` can create a localhost-only
Xvfb + VNC + noVNC display. `AMSIM_RENDER_BACKEND=auto` prefers VirtualGL
when a matching NVIDIA EGL/GLVND runtime is available and otherwise falls back
to direct X/OpenGL.

Remote-desktop setup is unnecessary on a workstation that already has a usable
X display.
