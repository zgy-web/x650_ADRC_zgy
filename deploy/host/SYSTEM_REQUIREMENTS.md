# Host system requirements

Validated baseline: Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic / Gazebo Sim 8, ros_gzharmonic bridge, Micro-XRCE-DDS Agent 2.4.2, Python 3.10, git/cmake/colcon, and an NVIDIA driver when GPU rendering is requested.

The project keeps host-level GPU/X/VNC setup separate from user-space workspace setup. `AMSIM_RENDER_BACKEND=auto` prefers the validated VirtualGL path when available and otherwise falls back to direct X/OpenGL rendering.

Remote desktop software such as TigerVNC/noVNC/XFCE is optional on a local workstation with a usable X display.
