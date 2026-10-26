source /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/cpp_fast_20261024/fast_environment_v1.sh
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
/usr/bin/python3 /root/autodl-tmp/safe-aerial-manipulator-testbed/scripts/render_x650_wrench_failure.py --run /root/autodl-tmp/safe-aerial-manipulator-testbed/runs/x650_cpp_robust_wrench_v1_20261024_205309_55502 --failure-analysis /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/cpp_robustness_20261024/wrench_failure_analysis_v1/analysis.json --failure-analysis-sha256 b9655dfebb334ea0b3e3e84250e7909a20570c5021276e1e171e32112195f7e7 --output /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/cpp_wrench_video_20261026/video_v1
result=$?
printf '%s\n' "$result" > /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/cpp_wrench_video_20261026/video_v1.exit
exit "$result"
