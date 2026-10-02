# Visualization and Rendering

Visualization is a first-class platform capability, not a debugging afterthought.

## Required modes

- Headless mode for automated experiments and batch data collection.
- Interactive GPU-rendered Gazebo GUI for inspection, demos, and debugging.
- Remote desktop access for server deployments.

## Current server backend

The validated server uses:

- TigerVNC on display `:1`
- noVNC/websockify on localhost port 6080
- XFCE
- VirtualGL
- NVIDIA EGL
- Gazebo Sim 8 GUI attached to an independently running headless Gazebo server

The platform keeps the GUI process separate from the physics/runtime process. Closing the GUI must not stop PX4 or Gazebo physics.

## CLI target

~~~bash
./scripts/amsim desktop-status
./scripts/amsim render-status
./scripts/amsim gui-start
./scripts/amsim gui-status
./scripts/amsim gui-stop
~~~

Future packaging will provision the desktop/rendering dependencies rather than relying on pre-existing AutoDL state.
