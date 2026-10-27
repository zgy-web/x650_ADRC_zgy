# Separate ROS 2 upper NMPC worker

This package builds `ros2 run flyarm_nmpc_cpp isolated_worker`. It is a new simulation-only worker alongside the existing MPC worker. It uses the same `LiveStart`, admitted `ControlSnapshot`, `ControlProposal`, execution gateway and PX4/TS20 adapters. It does not introduce a per-cycle consumer-acknowledgement wait.

The manifest selects `backend=nmpc` or `backend=mpc_callable`. The latter calls the retained `x650_mpc::Controller` facade with unchanged numerical core and parameters. Both modes use the same worker, contracts and execution path. In NMPC mode the selected mathematical plan comes from `x650_nmpc::solve_online`. That API evaluates bounded relinearized native proposals against the original nonlinear objective. The worker then independently checks the selected native optimality, interval, recovery and held-command certificates, the original wire contract, and the original snapshot issue-to-publication deadline. No output is published as valid merely because an inner QP succeeded.

## Configuration

All existing startup parameters remain in use:

- `startup_manifest`, `startup_sha256`, `output_dir`
- `expected_session_id`, `expected_epoch`, `simulation_live=true`
- `executor_cpu`, `auxiliary_cpus`, `timing_profile=mpc25_dispatch50_v1`

Two new parameters are mandatory: `nmpc_options_manifest` and `nmpc_options_sha256`. Their bound JSON schema is:

```json
{
  "schema_version": 1,
  "kind": "x650_online_nmpc_options_v1",
  "simulation_only": true,
  "backend": "nmpc",
  "task_config_sha256": "<exact original startup task hash>",
  "worker_executable": {"path": "/absolute/new/worker", "sha256": "<hash>"},
  "options": {
    "max_proposals": 4,
    "absolute_decrease": 1e-10,
    "relative_decrease": 1e-7,
    "proposal_budget_ms": 40.0,
    "total_budget_ms": 50.0
  },
  "source_artifacts": [{"path": "/absolute/source/file", "sha256": "<hash>"}]
}
```

The actual `/proc/self/exe`, the task, every listed source and the options manifest are checked against these bindings. The initial online policy is explicit: at most four outer proposals, a 40 ms proposal budget and the unchanged 50 ms total deadline from the **original snapshot issue time**. Callback entry does not replace that timestamp. The 200 ms offline investigation setting is not accepted by this worker.

## Build

Use the existing installed MPC/gateway/state packages from the accepted multirate environment. Build the new NMPC library first, including `online_solver.cpp`, then select only this new ROS package:

```bash
source artifacts/cpp_multirate_20261026/environment.sh
colcon build --base-paths ros2_ws/src/flyarm_nmpc_cpp \
  --packages-select flyarm_nmpc_cpp \
  --build-base /absolute/new/build --install-base /absolute/new/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DX650_MPC_CORE_LIBRARY=/absolute/verified/libx650_mpc_core.so \
    -DX650_NMPC_LIBRARY=/absolute/new/libx650_nmpc_offline.a \
    -DX650_NMPC_INCLUDE_DIR=/absolute/repo/control_packages/x650_nmpc_controller_v1/include
```

The privately included worker contract is an exact copy of the original committed `flyarm_mpc_cpp/worker_contract.hpp`; its origin and SHA are in `frozen_include/PROVENANCE.json`. This deliberately excludes unrelated, uncommitted processor-in-the-loop source changes. Shared configuration, trajectory, certificate, thread-placement and bounded-queue utilities come from the already installed original MPC package. The old worker and its source tree are not rebuilt or edited by this package.

## State and evidence semantics

`LiveStart` initializes the same anchor and authentic prestream history. Each solve receives an immutable original Memory and the snapshot's actual received TS20 packet/command history. Internal nonlinear iterations are not counted as execution steps. After a selected candidate passes the wire checks, the ROS proposal publication leaves its Memory pending. Only a matching `ActuatorSend` from the actual simulation writer commits that Memory exactly once; the callable MPC backend uses `prepare`/`commit_published` for the same transaction. The receipt must match the session, epoch, sequence, exact TS20 velocity, PX4 ENU-to-NED encoding, mask, reference altitude/yaw and a fresh (at most 25 ms old) writer timestamp. Repeated sends do not commit twice. This confirms publication to the execution interfaces, **not proof of physical consumption**. The unchanged downstream consumer ledger monitors consumption asynchronously.

DDS callbacks may deliver the next admitted snapshot before the send notification. The worker buffers at most that single next snapshot, retains its original issue timestamp and authentic history, and processes it after the matching send commit. It never waits for per-command consumer acknowledgement. A failed/expired candidate or an unsent final proposal is discarded without advancing committed Memory. `worker_commit_timing.json` records each actual-send commit and any reordered next sequence; `worker_final.json` separates accepted proposals, committed controller steps and discarded unsent tail. No shadow mode is exposed; private-domain software tests must label synthetic writer receipts explicitly.

Native full diagnostics are needed to evaluate the nonlinear objective. They are moved into the same bounded four-slot auxiliary recording queue; JSON encoding and disk writes occur on that auxiliary thread. Full records use a lossless `std::to_chars` encoder with `max_digits10`, exact signed/unsigned integers, finite-number enforcement and JSON string escaping. Integral-valued doubles retain a real-valued token, including signed zero. No fields or matrix precision are removed. `diagnostic_sender.json` records the actual per-record pop, CDR/hash, DOM, encoding, file write/close and object-release phases. The single recorder thread is created before affinity capture and startup; neither the queue capacity nor runtime thread count is increased.

This fixes a measured throughput bottleneck: on a saved approximately 501 kB Full record, the original JsonCpp encoder took about 35 ms, in addition to roughly 17 ms rebuilding the DOM and 0.3 ms writing the file. The replacement encoder took approximately 5.6 ms on the same data and CPU, with all decoded values preserved. This static-record timing comparison identifies the bottleneck; fresh timed worker probes and physical experiments are still required.

`native_records/*.json` retains the existing `result` structure and adds the backend/implementation identity (`cpp_upper_nmpc_v1` or `cpp_callable_mpc_v1`), `memory_before_step_count`, `selected_next_step_count`, and `valid_proposal_published`. Both backends use full native diagnostics so matrix audits and recording overhead are comparable; NMPC additionally records a separate `nmpc` object. The retained facade already supports this diagnostic mode without changing controller parameters. The `nmpc` field records:

- first/selected true nonlinear cost and its six components;
- every outer iteration, acceptance, native/stopping checks and rejection reason;
- native proposal time, outer evaluation time and original issue age;
- the separate solver acceptance flag and unchanged offline publication flag.

Wire `subproblems` continues to describe the selected native QP certificate. It is not the total outer NMPC iteration count. Native `diagnostics.objective` is the local quadratic cost; it must not be substituted for the `nmpc.selected_objective` in comparisons.

The usual raw snapshot/proposal CDR, phase timing, affinity, source/binary identities and recorder integrity reports remain available. A successful process or API test is not closed-loop flight acceptance. Fresh feedback, repeated downlinks, actual joint consumption, endpoint error and orderly exit must all be assessed from a complete new simulation run before proceeding to the figure-eight comparison.
