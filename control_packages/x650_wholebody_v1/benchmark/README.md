# Standalone, RAM-input MPC CPU benchmark

This Linux C++ executable calls the original `x650::Core::step` without ROS or Python. It accepts one bounded, hash-bound JSON dataset from stdin, preloads all typed inputs and controller memories, and emits one JSON result only after all timed calls finish. There is no output-file option. Failures and late calls remain in the result.

Build with the existing Eigen3, JsonCpp and OpenSSL development packages:

```sh
cmake -S control_packages/x650_wholebody_v1/benchmark -B /absolute/ram/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build /absolute/ram/build -j2
```

The workstation source package intentionally includes only production sources and the benchmark, so its build must use `BUILD_TESTING=OFF`. Full repository development builds may enable the input and process contract tests. Do not run CTest in a persistent NX directory: it writes test logs.

Runtime consists of `x650_mpc_memory_benchmark` plus `libx650_mpc_core.so`, `libx650_optimization.so` and `libx650_benchmark_predictor.so`. Set `LD_LIBRARY_PATH` to the dedicated runtime library directory. The workstation runner independently checks the SHA of all four loaded ELF files.

```sh
ulimit -c 0
env LD_LIBRARY_PATH=/absolute/code/lib /absolute/code/x650_mpc_memory_benchmark --predictor /absolute/code/lib/libx650_benchmark_predictor.so --predictor-sha256 <actual-arm-library-sha256> --input-sha256 <exact-stdin-sha256>
```

Feed stdin through the workstation runner; do not create a remote input, result or log file. Restore, JSON parsing, hashes, diagnostic serialization and network output are outside the Core timers; allocations performed inside the original Core remain included. Wall time and thread CPU time are both returned. The original 50 ms production checks and Compact diagnostics remain unchanged.

The only implemented backend is CPU and the only mode is independently restored fixed snapshots. No CUDA, whole local stack, physical closed loop or worst-case timing qualification is implied. The first real NX measurement and reproducible workstation CLI are described in [the experiment protocol](../../../docs/ORIN_NX_MPC_BENCHMARK.md).
