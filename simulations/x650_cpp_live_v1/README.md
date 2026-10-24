# Complete C++ online pipeline in the existing simulator

This opt-in scenario uses `flyarm_state_cpp` for actual estimated feedback and
consumed TS20 history, `flyarm_mpc_cpp` for the complete owning controller, and
the existing C++ ingress, periodic writer and physical-consumption reader.
Python owns ground preparation, the original prestream/reference transaction,
supervision, recovery and offline review. Its lifecycle allocator explicitly
throws if an ACTIVE Python controller step is attempted. Protocol revision 2
selects `consumption_policy=latest_value_v1`: regular output does not wait for
the previous joint-consumption receipt.

The initial trajectory is **quiet 30-second endpoint hold only**. A completed
launch is not a physical acceptance certificate. Ordinary figure-eight remains
after independent full-window hold acceptance, with fast figure-eight later.

## Prepare and check

Source the actual ROS 2/PX4 workspace, the original gateway overlay, and the new
worker and state-source overlays. The source overlay contains the additional
`SourceBinding` messages. Use the previously verified physical task; preparation
removes only its obsolete `numerical_backend` selection. It changes no model,
controller weight, PID gain, physical bound, bus parameter or deadline.

The revision-2 output contract separates the 20 ms absolute output schedule
from asynchronous joint-progress monitoring. The latter has an explicit
40 ms budget anchored to the oldest unresolved actual send for each joint;
new sends cannot renew that budget. This is a separate watchdog, not another
name for the existing 40 ms maximum output interval. Source freshness remains
25 ms and the original command issue time retains its 50 ms lease.

The C++ worker is still admitted after its previous candidate was actually
sent. That barrier does not wait for a joint-consumption receipt, and prevents
an overwritten, never-sent candidate from becoming the controller's previous
command. Actual joint history and MIT packets still come only from the plant.

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

In tracking mode, the writer keeps a bounded ledger of actual sends. Each joint
is observed separately; a later counter does not prove every earlier command
was adopted. Entries passed over by the online observation are marked
superseded-or-unobserved. A whole-command receipt requires an exact matching
two-joint observation. The independent complete physical trace distinguishes
actual adoption from commands superseded before a physics step. The anonymous
Float64 bridge still requires an exclusive writer and ordered one-to-one
delivery per joint; the trace checks this assumption rather than pretending
the receiver's counter is an explicit end-to-end command identifier.

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

Every send must be accounted for, but revision 2 does not require a receipt
for every send. Covered or not separately observed normal commands cannot be
silently called executed. The final stop-zero still requires actual two-joint
adoption and its matching receipt before recovery takes ownership. Software
tests must cover feedback arriving after the next output slot, skipped counters,
mixed joint observations, stale feedback, lack of adoption progress and stopping.

The simulator state adapter uses TS20 CSV produced by the actual plugin. A real
robot requires a separately tested bus-feedback adapter and new timing/load
qualification; server CPU IDs are not an Orin configuration. See
[the migration and evidence overview](../../docs/X650_CPP_PIPELINE.md).

## ROS overlay preflight

The launch now imports and performs an actual serialize/deserialize roundtrip
for all 15 required interface types before starting Gazebo. In particular,
`SourceBinding` and `SourceBindingAck` must come from the interface workspace
that added them. Building the source package alone does not install those
interfaces. A missing interface or incompatible native type support rejects
the launch on the ground.

For the separately qualified 2026-10-24 workspaces, start a **new shell** and
load the interface-bearing source workspace before the Release source and
streaming gateway overlays:

```bash
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/ros2_px4_ws/install/setup.bash
source /root/autodl-tmp/flyarm_ros2_live_v1_20261023/install/setup.bash
source /root/autodl-tmp/flyarm_cpp_pipeline_build_20261024/install/setup.bash
source /root/autodl-tmp/flyarm_state_cpp_build_20261024/install/local_setup.bash
source /root/autodl-tmp/flyarm_state_cpp_release_20261024/install/local_setup.bash
source /root/autodl-tmp/flyarm_gateway_streaming_20261024/install/local_setup.bash
```

These are the recorded workspace paths for this server, not a portable robot
installation layout. The launch additionally hashes the actually selected
executables against their software evidence, so an old source or dispatcher
cannot silently substitute for the selected build.
