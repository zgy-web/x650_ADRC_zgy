# Offline/shadow Python MPC reference process

This package reuses the existing `ControllerAdapter` and `Ts20MpcController`.
It is not native C++ MPC, a real-time guarantee, a flight adapter, or a hardware
validation. Its only ROS2 output is `ControlProposal` under the offline prefix;
it never publishes PX4 or joint actuator messages. Full legacy JSON diagnostics
are not reproduced by the compact proposal message. The independent recorder
must retain the typed inputs, proposals, gate events and their completeness.

The factory reads an explicit hash-pinned quiet tip-hold task once and executes
its existing read-only `task.preflight(check_airframe=False)`. This still checks
the physical report, frozen PID, motor source, gain-specific oracle, native
predictor and recorded prior gates. It does not start `ScenarioContext` or a
simulation. Missing artifacts fail construction; no test-model fallback exists.

The separately hash-pinned initialization JSON contains:

```json
{
  "schema_version": 1,
  "synthetic_software_fixture_only": true,
  "task_origin_sim_s": 10.0,
  "anchor_position_enu": [0.0, 0.0, 2.0],
  "anchor_rotation_row_major": [1,0,0,0,1,0,0,0,1],
  "anchor_q": [0.0, 0.0],
  "target_position_enu": [0.0, 0.0, 1.6],
  "published_prestream_history": [
    {"control_time_s": 9.98, "command": [0,0,0,0,0]},
    {"control_time_s": 10.0, "command": [0,0,0,0,0]}
  ]
}
```

The example target is illustrative; the launch driver must compute its actual
model's hold target. A synthetic initialization remains explicitly synthetic,
not historical reproduction. The five-axis **published** prestream history
primes the controller exactly once. It is distinct from each snapshot's
two-axis **consumed** joint commands and actually sent MIT packets. Context
history, reference and bus-origin times are task-relative; state/control source
times remain absolute simulation times. The source must supply the original
PX4 body estimate and causal physical-q/modeled-endpoint-dq joint observation.
The worker does not re-read CSV, substitute truth, or reset on each request.

```bash
ros2 run flyarm_mpc_reference reference_node --ros-args \
  -p repo_root:=/absolute/repository \
  -p task_config:=/absolute/quiet_task.json \
  -p task_config_sha256:=<raw-task-file-sha256> \
  -p initialization_manifest:=/absolute/initialization.json \
  -p initialization_sha256:=<raw-initialization-file-sha256> \
  -p expected_session_id:=7 -p expected_epoch:=1 \
  -p prefix:=/flyarm/offline/v1 \
  -p startup_metadata_path:=/absolute/new_reference_startup.json
```

The node subscribes to `admitted/snapshot`, not the untrusted input topic. The
optional startup report is created exclusively before publishers/subscriptions,
with actual installed module paths/SHA, process/dependency information and the
verified loaded native predictor identity. It never overwrites an earlier file.
The gateway owns admission, single-inflight and recorder-drain contracts. The worker
also verifies identity, sequence, clocks, gains, target and padded histories.
It calls the same controller once, retains its quality/nonlinear checks, and
enforces an additional 50 ms outer conversion/controller/proposal deadline.
Any rejection returns a zero-command failed proposal and latches the worker;
an explicit new process/epoch is needed. No retry or previous-command fallback
turns a failed sample into acceptance. Automatic GC policy is unchanged.
The wire `kkt_residual` is the original solver's `kkt_stationarity_inf`;
`primal_violation` is the original full-constraint violation. The compact
message does not add a new numerical acceptance tolerance.

The Python controller advances its own plan/filter memory when it produces a
proposal, before gateway acceptance. Phase 1 permits this only because the
gateway has one request in flight and latches a failed proposal/deadline: no
new accepted request may follow that rejection. This is not the legacy actual
publication/commit lifecycle and is not a native flight-control commit loop.

Pure API (no ROS imports needed):

```python
from flyarm_mpc_reference.factory import create_engine
engine = create_engine(repo_root, task_config, task_config_sha256,
                       initialization_manifest, initialization_sha256,
                       session_id, epoch)
proposal = engine.process(snapshot, proposal_factory)
evidence = engine.metadata()
```

Unit tests use a clearly labelled fake controller and the real state record API;
they certify parsing/dispatch/failure behavior, not MPC numerical performance.
Run `python -m unittest discover -s test -p 'test_*.py'` from this package.
