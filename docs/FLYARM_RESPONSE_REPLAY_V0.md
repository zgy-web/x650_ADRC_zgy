# Flyarm response replay v0 — initial surrogate, **not Franka**
Source: existing PX4 SITL `arm_motion_20261002_200808_89875/px4.ulg`, first 12 s of overlapping position and attitude data. 100 Hz interpolation. Frame anchored to first sample; Euler unwrapping used only for quick feasibility screening, not a proper SO(3) controller.

The replayed 6-DoF pose drives an *idealized first-order stage* with explicitly assumed 40 ms lag, 0.8 m/s linear speed limit and 1.5 rad/s angular speed limit. It is not Franka dynamics, workspace, IK, singularity or contact response. No training is performed.

Baseline: RMS translation error 0.002245 m, RMS Euler-angle vector error 0.2930 rad, rotational limiter active for 108/1199 steps. Other hypothetical settings show results are parameter-sensitive. Large attitude error may be affected by unwrapped Euler yaw jumps and needs a quaternion geodesic error/validity check before quantitative conclusions.

Use `/root/autodl-tmp/amsim-venv/bin/python scripts/flyarm_response_replay_v0.py --ulog runs/arm_motion_20261002_200808_89875/px4.ulg --out runs/flyarm_replay_v0/summary.json`.

**Next actual test:** implement FR3 7-joint URDF and Jacobian/differential IK, enforce joint position/velocity/acceleration/torque/workspace constraints, validate tool-mount transforms, and compare commanded vs feasible 6-DoF motion. Then integrate dynamic tracking and disturbances in a joint simulator, using real Franka controller limits and measurements when available.
