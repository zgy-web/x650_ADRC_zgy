# Complete native MPC ROS worker

This package is the `cpp_full_pipeline_v1` integration boundary. It receives the
existing admitted `ControlSnapshot`, calls the complete C++ controller with raw
state, actual command/packet history and an explicit trajectory preview, and publishes
the existing `ControlProposal`. It never publishes PX4 or joint actuator commands.
The existing ingress, actual-send barrier, periodic writer and consumption checks
remain necessary. The separate `flyarm_state_cpp` package supplies actual feedback.

The worker supports the original strict hold and one explicitly configured
ordinary figure eight. Each task requires its own numerical, ROS and physical
evidence. Compilation and synthetic contract checks alone cannot qualify a
flight, Orin hardware, or hard real-time behavior.

## Explicit startup

Build against matching complete core headers and an explicit library:

```bash
colcon build --packages-select flyarm_mpc_cpp --cmake-args \
  -DX650_NATIVE_SOURCE_DIR=/absolute/path/to/native \
  -DX650_MPC_CORE_LIBRARY=/absolute/path/to/libx650_mpc_core.so
```

The executable is `ros2 run flyarm_mpc_cpp isolated_worker`. Required ROS parameters
are `startup_manifest`, `startup_sha256`, `output_dir`, `expected_session_id`,
`expected_epoch`, `simulation_live:=true`, `executor_cpu` and `auxiliary_cpus`.
The process starts with the union of the declared worker CPU roles; READY binds
its executor to the single designated CPU and existing auxiliary threads to the
remaining CPUs. Unknown, reused, disappearing or misplaced threads fail stage
checks. Callback start/end observations prove the executor's observed CPU and
singleton affinity; they do not continuously observe short-lived auxiliary threads.

The startup JSON is strictly parsed by `config_io.hpp` and contains exactly:

- `schema_version: 1`, `kind: "cpp_full_pipeline_v1"`, `simulation_only: true`;
- `task_config`, `core_library` and `optimization_library`, each with an absolute
  `path` and `sha256`;
- `predictor_library` with absolute `path`, `sha256` and the fixed
  `symbol: "ts20_prediction_batch"`;
- nonempty `source_artifacts`, each an absolute path and SHA binding;
- `config`, containing every `x650::Config` field without defaulting or extras;
- `trajectory: {"kind": "hold"}`; or exactly the ordinary trajectory below.

```json
{"kind":"normal_figure8","amplitude_x":0.25,"amplitude_z":0.10,
 "period_s":12.0,"cycles":2,"start_s":2.0,"ramp_s":2.0,"duration_s":32.0}
```

The current target and every future node use the same C++ `Trajectory` law as
the state source. The preview times are `t + cumulative([actual_dt, node_dt, ...])`,
so a delayed or shortened first step cannot silently use a fixed 20 ms preview.
The center and time origin come from the actual `LiveStart`. The source's current
target must match this law exactly; wrong phases are rejected. The separate
`simulations/x650_cpp_normal_v1` entry binds the accepted hold and newly tested
dynamic worker. Fast trajectories require a later explicit entry and validation.

The actually loaded core ABI and optimization dependency are checked with `dladdr`
(including the actual `x650_assemble` symbol); the predictor is loaded
from its hash-checked absolute path. The manifest is prepared offline. The worker
does not invoke Python, import a Python controller or accept caller-built matrices.

READY occurs before any controller step. The actual identity-bound `LiveStart`
provides the anchor, target, origin and complete five-axis actual prestream history.
No candidate is promoted to an actual history entry by this worker. Each solve
requires the next admitted snapshot after the external actual-send barrier.

## Evidence and failure behavior

The worker preserves the original 50 ms outer gate, source-age/dt limits, quality
thresholds, nonlinear candidate check and PX4 gain-cancelling encoding. It latches
on invalid input, unsuccessful numerical output, evidence overflow, CPU identity
change or outer deadline failure. Failed proposals carry nonzero status and zero
command arrays. Downstream gates remain responsible for stop/exit behavior.

Raw received snapshot and published proposal CDR are captured for each callback.
Owned native diagnostic values are moved to a bounded auxiliary queue after
proposal publication. JSON conversion and disk writes run in that recorder thread;
a full queue is a reported failure, never a silently missing row. Offline replay
uses full native diagnostics for matrix/packet equivalence. Online diagnostics use
the explicitly recorded compact mode; omitted matrix/packet arrays are not claimed
as captured full-call evidence.

Evidence files are `worker_startup.json`, `worker_affinity_ready.json`,
`worker_ready`, `worker_live_start.cdr`, per-call `native_records/*`,
`worker_timing.json`, `diagnostic_sender.json`, `worker_affinity_final.json` and
`worker_final.json`. Files are fresh-only. There is no fabricated Python GC or
pickle report. SIGINT/SIGTERM first finish the worker and its thread-stage evidence,
then destroy ROS/RMW entities, preserving the shutdown identity check.

`test_worker_contract` exercises actual history/state/dt copying and rejection of
invalid identity, time, packet history, quality and PVA reconstruction.
`test_config_diagnostics` exercises exact startup schema, SHA bindings, duplicate
JSON keys, unsupported trajectory, nonfinite diagnostics and detached evidence.
Neither test sends actuator commands or proves runtime timing.

`test_normal_contract` and `test_normal_config` cover the exact ordinary schema,
phase boundaries, actual first-step intervals and preservation of strict hold.
`trajectory_probe` is compared against the independent original Python law by
`scripts/validate_x650_cpp_normal_reference.py`. Dynamic full-matrix fixtures and
the actual ROS worker are checked separately. The software ROS test records both
serialized endpoints and compares every decoded field exactly: CDR alignment
padding is not a control field and need not have identical bytes across serializers.
