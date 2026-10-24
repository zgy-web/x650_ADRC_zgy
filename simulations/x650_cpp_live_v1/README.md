# Complete C++ online pipeline in the existing simulator

This opt-in scenario uses `flyarm_state_cpp` for actual estimated feedback and
consumed TS20 history, `flyarm_mpc_cpp` for the complete owning controller, and
the existing C++ ingress, periodic writer and physical-consumption reader.
Python owns ground preparation, the original prestream/reference transaction,
supervision, recovery and offline review. Its lifecycle allocator explicitly
throws if an ACTIVE Python controller step is attempted.

The initial trajectory is **quiet 30-second endpoint hold only**. A completed
launch is not a physical acceptance certificate. Ordinary figure-eight remains
after independent full-window hold acceptance, with fast figure-eight later.

## Prepare and check

Source the actual ROS 2/PX4 workspace, the original gateway overlay, and the new
worker and state-source overlays. The source overlay contains the additional
`SourceBinding` messages. Use the previously verified physical task; preparation
removes only its obsolete `numerical_backend` selection. It changes no model,
controller weight, PID gain, physical bound, bus parameter or deadline.

```bash
python3 simulations/x650_cpp_live_v1/qualification.py prepare \
  --old-task /absolute/path/to/previous/task.json \
  --output /absolute/path/to/new/inputs \
  --protocol configs/experiments/x650_cpp_live_v1.json \
  --matrix-proof /absolute/path/to/30-case/report.json \
  --history-proof /absolute/path/to/1056-case/report.json \
  --worker-proof /absolute/path/to/actual-ros-worker/report.json \
  --source-parity-proof /absolute/path/to/source-parity/report.json \
  --source-build-proof /absolute/path/to/source_build_proof.json \
  --transport-proof /absolute/path/to/current-transport/report.json
```

Preparation creates `task.json`, `startup_manifest.json` and
`qualification_manifest.json` in a new directory. It does not launch a process
or grant acceptance. Use the startup hash printed by preparation:

```bash
python3 simulations/x650_cpp_live_v1/launch.py \
  --task /absolute/path/to/new/inputs/task.json \
  --startup-manifest /absolute/path/to/new/inputs/startup_manifest.json \
  --startup-sha256 ACTUAL_PRINTED_SHA256 \
  --qualification-manifest /absolute/path/to/new/inputs/qualification_manifest.json \
  --dry-run
```

The dry-run checks current artifact hashes, all numerical and ROS proof rows,
the exact startup configuration, source lifecycle proof, current dispatcher
identity and original physical prerequisites. It refuses changed inputs or
missing evidence. Remove `--dry-run` to launch the explicit simulation after
these checks pass; no human approval prompt is encoded in this workflow.

## Actual ownership and evidence

All components become READY while disarmed. The C++ source remains UNBOUND.
After the original lifecycle freezes the world reference and estimator reset
counters, Python publishes that exact binding and checks its one-time ACK while
the original prestream still owns outputs. At the next actual-send boundary,
`LiveStart` transfers the original anchor, target, origin and actual history.
Only the C++ dispatcher owns active actuator output. The original consumed-zero,
writer-exit, fresh Hold ACK, landing and parameter restoration steps are retained.

Each launch archives source bytes, runtime binaries, configuration and admission
evidence before flight, and verifies identities again at shutdown. The worker
records actual received snapshot/proposal CDR and compact prediction results;
the state source records raw input CDR and selection identities. Neither path
silently fills lost records with generated data. Large diagnostic JSON and file
work is outside the active worker callback.

Post-run review must check actual process exits, source and worker final states,
zero dropped evidence, all sends and both joint consumption counters, PX4 ULog,
native physical endpoint error, the entire declared 30-second window and the
safe recovery/parameter transaction. `launch.json.complete` means the lifecycle
finished with unchanged sources and runtime, whereas `physical_hold_qualified`
remains false until that independent review exists.

The simulator state adapter uses TS20 CSV produced by the actual plugin. A real
robot requires a separately tested bus-feedback adapter and new timing/load
qualification; server CPU IDs are not an Orin configuration. See
[the migration and evidence overview](../../docs/X650_CPP_PIPELINE.md).
