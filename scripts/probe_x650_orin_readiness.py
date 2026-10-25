#!/usr/bin/env python3
"""Read-only Jetson inventory to stdout; no input, result or log files are created.

Run through SSH stdin with PYTHONDONTWRITEBYTECODE=1. This inventory does not
change clocks, power modes, affinity, services, packages or controller settings.
It is preparation, not a performance test or hardware acceptance certificate.
"""
from pathlib import Path
import glob
import json
import os
import platform
import shutil
import subprocess
import time


def read_text(path):
    try:
        return Path(path).read_text(errors="replace").replace("\0", "").strip()
    except OSError as error:
        return {"unavailable": type(error).__name__}


def command(arguments):
    executable = shutil.which(arguments[0])
    if executable is None:
        return {"available": False, "command": arguments}
    try:
        result = subprocess.run([executable, *arguments[1:]], capture_output=True,
                                text=True, timeout=8, check=False,
                                env=dict(os.environ, LC_ALL="C", PYTHONDONTWRITEBYTECODE="1"))
        return {"available": True, "command": arguments, "returncode": result.returncode,
                "stdout": result.stdout[:16384], "stderr": result.stderr[:4096]}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"available": True, "command": arguments, "error": type(error).__name__}


def probe():
    started = time.monotonic()
    cpu = {"online": read_text("/sys/devices/system/cpu/online"),
           "available_affinity": sorted(os.sched_getaffinity(0))}
    for folder in sorted(glob.glob("/sys/devices/system/cpu/cpufreq/policy*")):
        cpu[Path(folder).name] = {name: read_text(Path(folder) / name) for name in
            ("related_cpus", "scaling_governor", "scaling_cur_freq", "scaling_min_freq", "scaling_max_freq")}
    thermal = [{"zone": Path(p).name, "type": read_text(Path(p) / "type"),
                "temperature_millicelsius": read_text(Path(p) / "temp")}
               for p in sorted(glob.glob("/sys/class/thermal/thermal_zone*"))]
    devfreq = [{"device": Path(p).name, **{name: read_text(Path(p) / name)
                for name in ("governor", "cur_freq", "min_freq", "max_freq")}}
               for p in sorted(glob.glob("/sys/class/devfreq/*"))]
    return {"kind": "x650_orin_readiness_readonly_v1", "hardware_timing_measured": False,
            "architecture": platform.machine(), "hostname": platform.node(),
            "kernel": platform.release(), "model": read_text("/proc/device-tree/model"),
            "l4t": read_text("/etc/nv_tegra_release"), "os_release": read_text("/etc/os-release"),
            "meminfo": read_text("/proc/meminfo"), "cpu": cpu, "thermal": thermal,
            "devfreq": devfreq,
            "tools": {"compiler": command(["c++", "--version"]),
                      "cmake": command(["cmake", "--version"]),
                      "pkgconfig": command(["pkg-config", "--modversion", "eigen3", "jsoncpp", "openssl"]),
                      "cuda": command(["/usr/local/cuda/bin/nvcc", "--version"]),
                      "jetpack": command(["dpkg-query", "-W", "-f=${Package}\t${Version}\n",
                                          "nvidia-l4t-core", "nvidia-jetpack"]),
                      "power_query": command(["nvpmodel", "-q", "--verbose"]),
                      "clocks_query": command(["jetson_clocks", "--show"])},
            "application_writes_files": False,
            "scope": "Read-only inventory. Output is stdout for storage on the workstation. "
                     "No claim about OS SSH/journal activity or other installed applications.",
            "probe_wall_s": time.monotonic() - started}


if __name__ == "__main__":
    print(json.dumps(probe(), indent=2, allow_nan=False))
