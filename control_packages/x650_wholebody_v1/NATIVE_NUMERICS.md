# Native numerical backend for the conditional TS20 upper MPC

`NativeTs20MpcController` explicitly selects C++ numerical kernels while retaining
the existing Python observation/packet preparation, transactional controller
lifecycle and ROS/PX4 supervision. This is an upper velocity MPC above the frozen
PX4 PID, not torque-level whole-body NMPC or a standalone C++ ROS executor.

## What executes in C++

- Exact ZOH base response propagation, thrust-direction attitude/endpoint output
  and output Jacobians, nominal arm allocation, cost/constraint assembly, H and g.
- Cholesky whitening of the strictly convex QP, an interior-optimum fast path,
  bounded Goldfarb–Idnani active-set solution, and KKT/primal checks against all
  original constraint rows and normalized input bounds.
- Candidate position/velocity error gates, complete substep envelope and
  integrated reference limits. The existing native TS20 event rollout supplies
  the actual nonlinear candidate motion; MIT packet preparation remains Python.

The reference `mpc.py`, `ts20_mpc.py`, and predictor remain unchanged. The native
controller reuses the TS20 transaction and up to three relinearizations within
the original **50 ms total rejection deadline**. It does not dispatch a failed
candidate or substitute the previous plan. PID values, weights, physical/soft
limits, source-age guards and candidate tolerances are unchanged.

The normalized optimization is

```
min_z  0.5 zᵀ H z + gᵀ z
s.t.   lo <= C z <= hi,  -1 <= z <= 1
H = 2 Aᵀ A, g = -2 Aᵀ b
H = L Lᵀ, z = -H⁻¹ g + L⁻ᵀ y
```

No constraint compression or tolerance widening is used by the new solver.
The original Python full-row verifier independently checks the native results
in differential validation. The two optimizers are numerically equivalent on
the admitted cases; their iteration counts are not interchangeable. Native
diagnostics use `solver_iterations=-1` because the vendor solver does not export
an iteration count. Both outer active-set iterations and inner transitions are
bounded, and limit exhaustion rejects the solve.

## Build and call

The Linux simulation server already provides a C++17 compiler and Eigen3.
The pinned MIT-licensed QuadProg++ source is included under `native/vendor`.
Its upstream commit, original hashes and the local iteration-bound modification
are documented in `UPSTREAM.json`; its original license is retained.

```bash
cmake -S control_packages/x650_wholebody_v1/native \
  -B /absolute/path/to/native-build -DCMAKE_BUILD_TYPE=Release
cmake --build /absolute/path/to/native-build -j2
sha256sum /absolute/path/to/native-build/libx650_optimization.so
```

Construct `NativeTs20MpcController` with the same arguments as
`Ts20MpcController`, plus `native_library` and the independently pinned
`native_sha256`. No implicit build, download or Python fallback occurs.
The synchronous C ABI receives contiguous double arrays whose Python owners
remain alive until return. Borrowed integer-address pointers avoid the
owner-dictionary cycles in NumPy 1.21 `ndpointer`/`data_as` conversions.

For the server task, add an explicit `numerical_backend` object containing
`kind: cpp_numerics_v1`, the library path/hash, and a native software acceptance
path/hash. Both task preflight and allocator construction verify the binding.
The existing ROS 2 worker selects the same backend from that task. Its startup
receipt records the actually loaded optimization library, not just a requested
path. Tasks without the object keep the archived reference backend.

## Verification and experiment order

The protocol was saved before the comparisons in
`native/validation_protocol.json`. Relevant limits are matrix absolute/relative
tolerance `1e-9`, normalized-plan/state difference `1e-7`, objective scaled
difference `1e-8`, identical candidate classification/MIT packets, full-row
primal violation `2e-7`, KKT stationarity `0.002`, and the original 50 ms deadline.

1. `scripts/validate_x650_native_numerics.py` executes the hash-pinned original
   30-case constructor and compares matrices, plans, states, costs and gates.
2. `scripts/replay_x650_native_numerics.py` replays every returned control call
   from the pinned prior hold, using the recorded prior plan and only packets
   available at each actual measurement. It does not use future plant states
   or rewrite the original 56 ms interval failure.
3. `scripts/qualify_x650_native_backend.py` runs all existing package, adapter,
   auditor and shared-loop regressions plus the native negative-path tests,
   reruns the matrix suite and binds the actual sources and libraries.
4. ROS 2 healthy/worker-stop tests check transport, gateway decisions and durable
   independent records with the explicitly selected backend.
5. Fresh hover/probe certificates and a **new complete 30 s quiet endpoint hold**
   are required before the fast figure-eight stage. A partial hold is not a pass.

On the 2026-10-23 server, the first admitted source set passed **328 regression
tests, 30/30 frozen cases and 388/388 recorded calls**. The recorded comparisons
had maximum normalized-plan difference `7.36e-13` and matrix difference
`4.16e-11`. Instrumented native calls reached `27.70 ms`, versus `36.54 ms` for
the reference in that replay. These are observed software timings, not a
worst-case or sustained-rate guarantee. The independent physical hold result
is reported separately under `docs/validation/native_numerics_20261023`.

Early replay attempts are retained: the verifier initially retained full
diagnostic graphs and repeatedly constructed dynamic-library objects; those
actions are outside a live control callback. The verifier now uses bounded
retention, one-time library loading, and restores instrumented methods without
creating controller ownership cycles. No GC disable, explicit collection,
deadline widening, PID retuning or deletion of failed evidence was used.

The TS20-50 plant is still a conditional model based on the supplied bench
identification, rather than qualification of the actual 24 V loaded aerial arm.
Base response parameters remain the bound prior. This numerical migration does
not change those physical modeling limits.
