# Native whole-body recorder

This directory contains the source for the Gazebo system plugin
`wbbench::WholeBodyRecorder`. It records physics-step measurements for the
two-link aerial-manipulator validation workflow. It is an observer, not a
controller, and does not constitute a stage-pass certificate.

The existing source was recovered within the simulation server without changing
its contents. Source SHA256:
`826bbe091507121e6f397f176cbd4288d707781f570e92efa5c1ac27d91e69ad`.

## Build

Use Linux with Python 3, a C++17 compiler, pkg-config, and the Gazebo Harmonic
development packages exposing `gz-sim8` and `gz-plugin2-register`. This script
does not download dependencies.

From the repository root:

```bash
python3 benchmark/whole_body_closed_loop_v1/build_recorder.py
```

The output is
`benchmark/whole_body_closed_loop_v1/build/libwholebody_recorder.so`.
The build directory also contains `compile.log` and `build_manifest.json`,
including source/binary SHA256 values, compiler version, package versions, and
the compiler command. Build products are excluded from Git.

For an isolated build, pass an explicit path:

```bash
python3 benchmark/whole_body_closed_loop_v1/build_recorder.py --build-dir /absolute/path/to/separate-build
```

The source directory itself is rejected as a build destination. The script does
not install, start Gazebo or PX4, or replace the legacy runtime library at
`benchmark/whole_body_closed_loop_v1/libwholebody_recorder.so`.
Do not rebuild over a library bound to an ongoing experiment. Select the new
absolute library path explicitly with the X650 scenario's
`native_recorder_library` configuration key before a new run; the scenario
records that binary's hash. Keep prior experiment manifests and their original
library hashes intact. Compiler/package changes can change the binary hash even
when the source hash is identical.

## Verify the built plugin

```bash
gz plugin --info --plugin benchmark/whole_body_closed_loop_v1/build/libwholebody_recorder.so
```

The plugin listing must include `wbbench::WholeBodyRecorder` and its Gazebo
system interfaces. Use the system Gazebo runtime for this check. Loading Gazebo
libraries from a Conda Python process can pick an older bundled `libstdc++`
and fail with a missing `GLIBCXX` version even when the system runtime works.
Do not replace system libraries to work around that environment mismatch.

## Runtime contract

The plugin name is `wbbench::WholeBodyRecorder`. Its SDF inner XML accepts a
`config_path` pointing to a text configuration file. The existing X650 adapter
uses these entries:

```text
model=your_model_instance
output=/absolute/path/to/native_stream.csv
duration=300
```

The existing observer assumes the two-link arm layout used by this project.
The independent tip is the world pose of `arm_link_2` applied to the local
offset `[0.25, 0, 0]`; this is not a configurable arbitrary-URDF end effector.
For a different arm or tip, change and revalidate the observer contract before
using its data.

The X650 adapter freezes complete original CSV rows and checks measured
base/joint/tip validity and time coverage. A successful build or plugin load
alone does not establish those runtime properties, controller performance,
GPS accuracy, or hardware performance.

The unrelated historical `build_disturbance.py` is not needed to build this
recorder. Private geometry, prebuilt libraries, and experiment data are not part
of this source directory's release.
