# Standalone, RAM-input MPC CPU benchmark

This Linux C++ executable calls the original `x650::Core::step` without ROS or Python. It accepts one bounded, hash-bound JSON dataset from stdin, preloads all typed inputs and controller memories, and emits one JSON result only after all timed calls finish. There is no output-file option. Failures and late calls remain in the result.

Build with the existing Eigen3, JsonCpp and OpenSSL development packages:

```sh
cmake -S control_packages/x650_wholebody_v1/benchmark -B /absolute/ram/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build /absolute/ram/build -j2
```

The original ABI 1 workstation package omits tests and requires `BUILD_TESTING=OFF`. The ABI 4 package also includes the native and benchmark contract tests, and uses `BUILD_TESTING=ON` on both hosts. Run its build and CTest in NX `/dev/shm`, then remove that RAM directory; CTest writes logs. The build helper supports a server-only build directory override for containers whose `/dev/shm` is mounted `noexec`.

Runtime consists of `x650_mpc_memory_benchmark` plus `libx650_mpc_core.so`, `libx650_optimization.so` and `libx650_benchmark_predictor.so`. Set `LD_LIBRARY_PATH` to the dedicated runtime library directory. The workstation runner independently checks the SHA of all four loaded ELF files.

```sh
ulimit -c 0
env LD_LIBRARY_PATH=/absolute/code/lib /absolute/code/x650_mpc_memory_benchmark --predictor /absolute/code/lib/libx650_benchmark_predictor.so --predictor-sha256 <actual-arm-library-sha256> --input-sha256 <exact-stdin-sha256>
```

Feed stdin through the workstation runner; do not create a remote input, result or log file. Restore, JSON parsing, hashes, diagnostic serialization and network output are outside the Core timers; allocations performed inside the original Core remain included. Wall time and thread CPU time are both returned. The original 50 ms production checks and Compact diagnostics remain unchanged.

The only implemented backend is CPU and the only mode is independently restored fixed snapshots. No CUDA, whole local stack, physical closed loop or worst-case timing qualification is implied. The first real NX measurement and reproducible workstation CLI are described in [the experiment protocol](../../../docs/ORIN_NX_MPC_BENCHMARK.md).

ABI 4 inputs bind `expected_core_abi=4` and `expected_optimization_abi=2`; a mismatch fails before measurement. The extended production profile has `nodes=15`, `node_dt=.04`, `max_dt=.1`, and `command_hold_s=.16`. Its nominal MPC period is **40 ms**; the independent continuous dispatcher period is **20 ms**, and the unchanged Core computation deadline is **50 ms**. The legacy sample field `exceeded_20ms_target` remains available as a dispatch-reference statistic and is explicitly labelled as such for this profile. Hold-guard and interval-repair diagnostics are copied after the Core timers stop.

Use `prepare_x650_abi4_benchmark.py` for the current recorded corpus, `build_x650_benchmark_in_ram.py` for the code-only deployment, and `compare_x650_paired_core_benchmark.py --timing-profile abi4_mpc25_dispatch50` for explicit ABI 4 verification. The default comparison profile remains `legacy20`, so an extended configuration cannot silently pass the old acceptance contract. `report_x650_abi4_benchmark.py` retains numerical failures and returns a nonzero exit status if the full acceptance policy fails, even when all timing measurements completed.
