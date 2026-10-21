# URDF input and runtime model contracts

The platform accepts a local URDF or ZIP, resolves its local mesh resources,
checks rigid-body mass/inertia and joints, and generates a portable Gazebo model.
Importing geometry does not make a vehicle ready to fly: an explicit propulsion
profile, PX4 airframe ID, controlled-joint list and model hash must be bound first.
The original reviewed `x500_underfold` model remains supported.

Run these commands in the existing AMSIM environment:

```bash
scripts/amsim model-inspect /absolute/path/to/robot.urdf
scripts/amsim model-import /absolute/path/to/robot.urdf --name my_vehicle
scripts/amsim model-preview amsim_urdf_my_vehicle --static
scripts/amsim model-bind-flight amsim_urdf_my_vehicle /absolute/path/to/flight_profile.json
```

Use each command's `--help` for ZIP/package selection. Keep original CAD and
generated models local. The importer rejects resource paths escaping the input
tree and records hashes; conversion and preview preserve explicit source/model
identity. `model-preview --check` runs a bounded native physics check and does
not start PX4 or arm a vehicle.

The active runtime contract records the model hash, base link, PX4 airframe,
controlled joints and initial joint positions. Readiness, public API and recording
use that same contract. A vehicle with no controlled joints does not need an arm
topic; an active runtime with missing or conflicting metadata fails explicitly.
An imported model without its flight binding cannot start PX4.

Runtime startup cleanup handles errors and interruptions and preserves owned
process/runtime identity. This does not authorize terminating unrelated processes.
Platform files should be committed separately from experiment outputs. Local
`runs/`, `generated/`, `artifacts/` and credentials are excluded from version control.

## Reproducible checks without private CAD

The tests generate small synthetic URDFs and temporary model/runtime fixtures.
They require the normal Python project dependencies, but no private airframe,
running ROS/Gazebo, or server access:

```bash
PYTHONPATH=src python -m unittest discover -s tests/unit -p 'test_urdf_*.py' -q
PYTHONPATH=src python -m unittest discover -s tests/unit -p 'test_runtime_start_cleanup.py' -q
```

The 2026-10-21 local verification passed 63 URDF/model-contract tests and 12
runtime startup/cleanup tests. Physical flight performance is validated separately
against the actual generated model and its recorded parameters.
