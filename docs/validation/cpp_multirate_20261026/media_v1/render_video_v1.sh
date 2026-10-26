#!/bin/bash
source /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/cpp_multirate_20261026/environment.sh
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
/usr/bin/python3 scripts/render_x650_multirate_wrench.py --launch /root/autodl-tmp/safe-aerial-manipulator-testbed/runs/x650_cpp_multirate_launch_20261026_220335_24241 --audit /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/cpp_multirate_20261026/wrench_audit_followup_v4/report.json --audit-sha256 4e64cd972546597d77c97fdd6ed601d7fbbbaebb60606e92599370cb824f25e7 --output /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/cpp_multirate_video_20261026/video_v1
result=$?
printf '%s\n' "$result" > /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/cpp_multirate_video_20261026/video_v1.exit
exit "$result"
