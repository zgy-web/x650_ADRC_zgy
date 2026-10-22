# X650 TS20 conditional plant: preparation and native interface checks

This is the reproducible preparation workflow for the **new**
`amsim_urdf_x650_2link_ts20_v1_flight` model. It replaces the original two ideal
joint velocity controllers with one TS20 free-motion I/O surrogate. The X650,
arm URDF, rigid inertias, meshes, collision geometry, motor mapping and rotor
parameters are preserved. Old flight certificates are not transferred.

The subsequent damped candidate is a separate model,
`amsim_urdf_x650_2link_ts20_damped_v1_flight`, with airframe **4996**. Its frozen
profile is `configs/actuators/ts20_50_24v_damped_v1_candidate.json`: Kp remains 6,
Kd is 0.25, and all published mechanical/friction parameters, 5 ms bus period,
zero nominal torque feedforward and original acceptance limits remain fixed.
It does not replace model/airframe 4997 or relabel its failed high-speed probe.
The observed old joint-probe peak was approximately 0.307 rad/s, above the
predeclared 0.2 rad/s small-probe gate; that failure is retained.

The gain-selection criterion is
`max_negative_friction_slope=sqrt(2)*delta/vs*exp(-0.5)=0.171552777` in normalized
interface coordinates. With Kd 0.25, `b+Kd=0.254194908`, a ratio of 1.48173 that
exceeds the predeclared 1.4 margin. This is a sliding-branch incremental-damping
criterion, not a proof of overall sampled-loop or loaded-hardware stability.
Offline replay of the original consumed upper commands, integrated reference
and actual quantized MIT qref/dqref changed only Kd and gave joint speed peaks
0.113014/0.112727 rad/s; reference-tracking RMS increased slightly, about
0.09/0.13 mrad. New complete native flights remain separate acceptance steps.

The user confirmed two TS20-50 servos and 24 V supply. The frozen upstream model
was identified at approximately 20 V with a small bench output disk. It does
not establish physical shaft-torque scale or the response of this loaded aerial
arm at 24 V. Command-normalized inertia/friction are never used as URDF inertia.

## Source and outputs

| File | Purpose |
|---|---|
| `scripts/ts20_interface_system.cpp` | Gazebo system and numerical core self-test |
| `scripts/prepare_x650_ts20_plant.py` | Pinned external-model verification, native build, SDF derivation, complete import/preparation manifests and separate airframe installation |
| `scripts/prepare_x650_ts20_commissioning_inputs.py` | Optional exclusive creation of a copied world and unchanged 42-value PID profile rebound to the new SDF hash |
| `scripts/probe_x650_ts20_ground.py` | Unpowered low-amplitude native Gazebo direction/plumbing probe; no PX4 startup |

The original airframe is 4998. Airframe 4999 is already occupied by the original
unarmed X650 (`4999_gz_x650_mn4014_v1`), so this plant uses **4997**, after an
explicit conflict check. The preparer refuses to overwrite generated outputs or
an occupied PX4 airframe ID.

## Rebuild on the simulation server

Use an existing C++17 compiler, `pkg-config`, `gz-sim8`, `gz-plugin2-register`,
`gz-transport13` and `gz-msgs10`. No installation/download is performed.
Python requires NumPy, SciPy and PyYAML for the pinned upstream source checks.
On the 2026-10-22 container, `/usr/bin/python3` has the working system
NumPy 1.21.5/SciPy 1.8.0 pair. The existing `amsim-venv` had a NumPy/SciPy ABI
mismatch and must not be assumed suitable for these standalone source checks.

Run from the repository root with the existing original X650 assets and the
hash-pinned external TS20 snapshot available. This command is a **fresh output**
example: select unused output/build/preparation directories and a free airframe
ID on a new installation; do not rerun over the current installation.

```bash
/usr/bin/python3 scripts/prepare_x650_ts20_plant.py \
  --source-sdf generated/imported_models/amsim_urdf_x650_2link_v2_flight/model.sdf \
  --output-dir generated/imported_models/amsim_urdf_x650_2link_ts20_v1_flight \
  --model-name amsim_urdf_x650_2link_ts20_v1_flight \
  --source-root artifacts/ts20_integration_20261022/upstream_snapshot \
  --source-manifest artifacts/ts20_integration_20261022/upstream_snapshot/source_manifest.json \
  --motion-json artifacts/ts20_integration_20261022/upstream_snapshot/models/ts20_50/motion.json \
  --candidate-profile configs/actuators/ts20_50_24v_candidate.json \
  --log-dir artifacts/ts20_chain_hold_20261022/unassigned_log_fallback \
  --build --build-dir generated/ts20_interface_build_fresh \
  --complete-import \
  --source-preparation generated/x650_2link_v2_20261021/preparation_report.json \
  --preparation-dir generated/x650_2link_ts20_v1_20261022 \
  --source-airframe generated/x650_2link_v2_airframe_20261021/4998_gz_x650_2link_v2 \
  --airframe-dir generated/x650_2link_ts20_v1_airframe_20261022 \
  --airframe-id 4997 --px4-dir /root/autodl-tmp/PX4-Autopilot --install
```

The script first verifies the complete caller-owned TS20 numerical source tree.
It builds/tests the plugin in an isolated new directory, removes exactly the
two old `JointController` systems, retains one joint-state publisher and
validates the new SDF with `gz sdf -k`. `--complete-import` creates the complete
`import_manifest.json`, updated `flight_profile.yaml`, new geometry preparation
report and `installation_manifest.json`. It checks that rigid inertias and
collision geometry remain equal after normalizing the renamed model resource
prefix. The normal platform `manager.resolve` and `ArmModel.from_report` checks
are run before installation. `flight_ready` describes simulation configuration;
`flight_validated` remains false.

The preparer does not generate a world or tune PID. The current experiments use
the existing source world
`artifacts/x650_hover_diagnosis_20261021/default_expanded.sdf`. It is valid to
reference that exact file directly; a new world file is not required. The 42 PID
values come from
`artifacts/x650_precision_commissioning_20261021/selected_pid_parameters.json`.
They may be rebound to the new model with parent-source hashes and
`new_plant_tuned=false`; this does not imply a new PID tuning result.

For a fresh preparation directory without `world.sdf` or `pid_profile.json`,
the optional helper makes this operation explicit and refuses overwrites:

```bash
/usr/bin/python3 scripts/prepare_x650_ts20_commissioning_inputs.py \
  --source-world artifacts/x650_hover_diagnosis_20261021/default_expanded.sdf \
  --source-pid artifacts/x650_precision_commissioning_20261021/selected_pid_parameters.json \
  --model-manifest generated/imported_models/amsim_urdf_x650_2link_ts20_v1_flight/import_manifest.json \
  --preparation-dir generated/x650_2link_ts20_v1_20261022
```

This optional helper was added for reproducibility. The legacy TS20 inputs
predate it; the damped variant uses it to create its new world/PID files. It
does not issue a flight certificate.

To prepare the damped variant, use the same original ideal-joint source model,
the new frozen candidate profile and separate output/preparation/airframe/log
directories. Pass `--airframe-id 4996 --airframe-label
gz_x650_2link_ts20_damped_v1`. `--airframe-label` defaults to the original
`gz_x650_2link_ts20_v1` and only accepts a lowercase `gz_` identifier; unsafe
path characters are rejected before any model preparation. For this gain-only
change the parameterized, already validated `generated/ts20_interface_build_v2/
libts20_interface.so` is reused through `--library`, without overwriting it.
The two helper tools accept only the explicit legacy/4997 and damped/4996
model-airframe pairs.

The damped model outputs are:

```text
generated/imported_models/amsim_urdf_x650_2link_ts20_damped_v1_flight/
generated/x650_2link_ts20_damped_v1_20261022/
generated/x650_2link_ts20_damped_v1_airframe_20261022/
artifacts/ts20_chain_hold_damped_v1_20261022/
```

On 2026-10-22, the damped SDF hash is
`5a2dd9c7adaf929ad8a60abe2046eeaff9fe56b3a01fa13fa20129bdf32d53eb`;
the preparation report hash is
`0b6c7e79def5ad7b8cb12c799e9a4e47d4caf95a092cef80810def564d8a16e0`.
The separate `4996_gz_x650_2link_ts20_damped_v1` airframe was installed in the
PX4 source ROMFS and existing SITL build tree. The copied world retains source
hash `21701faad142ed507f15a05572eda2af813dea14ebb81b03c3bb8fd127f68248`;
all 42 PID values remain unchanged and the rebound profile explicitly records
`new_plant_tuned=false`. These identities describe prepared inputs, not passed
flight stages.

Runtime and audit predictors for each variant must obtain effective Kp, Kd and
bus period from its hash-bound plant parameters and check them against that
variant's frozen gain profile. They must not silently use the legacy default
Kd. The damped variant requires its own eight-case oracle declaration and
acceptance; the old-gain oracle cannot qualify changed MIT gains.

## Execution and observation contract

Before starting Gazebo, set `AMSIM_TS20_LOG_DIR` to a new, existing absolute run
directory. Gazebo inherits the environment through the normal supervisor.
The model SDF hash stays fixed across runs; the plugin rejects existing
`ts20_state.csv` or `ts20_packets.csv` rather than truncating old evidence.

The public upper interface remains the two `gz.msgs.Double` velocity topics:

```text
/model/<actual_instance>/joint/arm_joint_1/cmd_vel
/model/<actual_instance>/joint/arm_joint_2/cmd_vel
```

Upper velocity is integrated into reference position `r` from one initial
physical joint state. The reference is never reset to measured q at each MPC
step. On a fixed 5 ms grid the plugin quantizes position and velocity commands
and holds the MIT fields independently. It splits each 4 ms physics interval
at exact packet-send and delayed-application events; it does not approximate
the 5 ms bus with an 8 ms accumulator. The imported delay is a combined
interface delay, not an independently identified firmware current-loop delay.

The numerical core uses RK4 substeps no longer than 0.25 ms with a signed
sliding branch, bisection to a true zero-velocity event and static release/stick
logic. Internal displacement over a physics interval is applied as its mean
joint velocity via `JointVelocityCmd`. Consequently:

- Physical q is the actual Gazebo joint position and remains the feedback and
  native end-effector evaluation source.
- `dq_internal` is the conditional simulated servo endpoint-speed report.
  MPC's actuator state may use this report, explicitly identified as a
  simulated sensor, rather than asserting it is the native interval-average
  velocity or a validated real hardware encoder model.
- `dq_physical` is Gazebo's interval-mean joint velocity. Both quantities are
  logged independently. Their instantaneous difference is not a same-definition
  tracking-error measurement.
- The mean-velocity constraint leaves Gazebo's rigid bodies/contacts active, but
  does not implement available motor torque or load-dependent servo dynamics.
  Contact can prevent the physical q from following the free-motion state.

The new runner must reject `abs(q_internal-q_physical)>1e-4 rad`, stale/causally
future data, clock gaps and all original physical limits. It must not substitute
the internal position for native end-effector scoring. Sim-clock reset/gaps are
rejected; reset requires a fresh plugin instance and new evidence paths.

`ts20_state.csv` has `sim_s,step_dt_s,interval_begin_s`; for j=1,2 it records
`qj_internal,dqj_internal,qj_physical,dqj_physical,rj,uj,receiptj_s,upper_sequencej,zero_eventsj,release_eventsj`.
It also contains latest-send and currently-active MIT packet sequence/times and
reference fields. `ts20_packets.csv` records each complete send:

```text
send_s,apply_s,sequence,qref1,dqref1,tau1,kp1,kd1,qref2,dqref2,tau2,kp2,kd2
```

Both streams flush after every physics row or packet. Receipts mean the first
physics PreUpdate consuming the latest callback; callbacks within one physics
step can coalesce and their sequence count is retained. They are not separate
hardware bus timestamps.

## Native checks completed on 2026-10-22

The C++ mechanics core was checked against the pinned upstream LSODA event
solver on 501 samples, including 8 zero-speed events, 8 releases and 3 reversals.
Maximum differences were `1.013e-8 rad` in q and `1.199e-6 rad/s` in endpoint
velocity, within the declared `1e-5 rad`/`1e-3 rad/s` tolerances.

Three unpowered native Gazebo probes are retained under
`artifacts/ts20_chain_hold_20261022/native_ground_probe_v{1,2,3}`:

| Probe | Observation | Outcome |
|---|---|---|
| v1, ±0.08 rad/s | Joint 2 encountered the floor near 0.12 rad; internal/physical position difference accumulated to 0.0323 rad | Rejected; demonstrates the free-motion contact limit |
| v2, ±0.03 rad/s, short reversal | Position agreement was approximately 2e-9 rad, but joint 2 did not actually reverse before the short command ended | Not accepted as a two-direction check |
| v3, ±0.03 rad/s, longer joint-2 reversal | Both joints physically moved in both directions; 2,750 native 4 ms rows and 2,200 fixed-grid MIT packets | Interface/direction check passed |

For v3, maximum internal/physical q differences were `2.26e-9 rad` and
`2.80e-9 rad`; maximum bus-grid error was `9.95e-16 s`. Actual speed ranges were
`[-0.03608,0.03612]` and `[-0.03638,0.03619] rad/s`. Internal endpoint speed and
native average speed differed by up to 0.00842/0.01316 rad/s, as expected from
their distinct time definitions. These are interface checks, not hover,
flight-stage, 24 V or loaded hardware acceptance.

A reusable equivalent low-amplitude probe is now supplied as source:

```bash
/usr/bin/python3 scripts/probe_x650_ts20_ground.py \
  --model-manifest generated/imported_models/amsim_urdf_x650_2link_ts20_v1_flight/import_manifest.json \
  --output artifacts/ts20_chain_hold_20261022/native_ground_probe_fresh
```

It refuses existing PX4/Gazebo processes, starts only an isolated Gazebo world,
issues no rotor command and shuts down its own process group. The historical
v3 used an archived helper with the same schedule. The new reusable wrapper
adds manifest validation, process exclusion, source hashing and summary output;
it has not been substituted for the historical evidence.

The new damped model was checked independently using that reusable wrapper at
`artifacts/ts20_chain_hold_damped_v1_20261022/native_ground_probe_v1`.
`probe_analysis.json` reports 2,751 native 4 ms rows, 2,201 MIT packets and
5 ms packet-grid error at most `9.95e-16 s`. Both joints physically moved in
both directions. Maximum internal/physical q differences were `2.261e-9` and
`2.800e-9 rad`, and actual speed ranges were `[-0.030061,0.030085]` and
`[-0.030085,0.030085] rad/s`. The probe passed and its own Gazebo process exited
cleanly. No PX4 or rotor command was used. This accepts the conditional
interface and small-motion direction check only; fresh hover, airborne joint
probe and full end-effector hold remain separate gates.

## Native observer and disturbance source recovery

The required historical source files are now available locally under
`benchmark/whole_body_closed_loop_v1`, without downloading any geometry or
prebuilt library:

| Source | Original Linux LF SHA256 |
|---|---|
| `wholebody_recorder.cpp` | `826bbe091507121e6f397f176cbd4288d707781f570e92efa5c1ac27d91e69ad` |
| `disturbance.cpp` | `c7752489cded1a218a671a02c0098c165c9b5a9d66682a922fe89b056e17a257` |
| `disturbance_loader.py` | `266c17f9b37dc2c4698ff0fe55c3472f9d2acedd8299342c6caeda6ff38bba31` |
| `build_disturbance.py` | `6c4d4e7887e3b6e49f4b3638e9b0844c3b64b16d9028b4e6b7f09b5194cfa2c4` |

The existing recorder source already matched after Windows-to-LF normalization;
it was not overwritten. The disturbance build helper is historical and writes
its default library beside the source, so do not run it over a library bound to
an active experiment. The old loader hardcodes `/world/default` and world entity
id 1; a new disturbance integration must verify the actual world/entity, load
result, full schedule, physics-step wrench enqueue log and independently
observed motion. The C++ log measures an ECS wrench increment, not a contact
force sensor or independent proof of physics consumption.

Gazebo msgs10 `Scene` has a world name and model entries but no world entity-ID
field. A successful `scene/info` response alone cannot establish that entity 1
is the world. Before a new dynamic disturbance integration, bind the actual
world/instance from runtime metadata, validate the unique live instance in the
scene, and identify the world through a read-only serialized ECS state response
(`World` and `Name` components, plus the model's parent-world relationship).
Record the verified world name/ID and model/link identities in the load
evidence; a loader Boolean is only an acknowledgement. This requirement does
not qualify an earlier experiment.

A new opt-in implementation is supplied by `scripts/gz_world_entity_probe.cpp`
and `benchmark/whole_body_closed_loop_v1/gz_world_entity_binding.py`. The C++
client requests `/world/<actual-world>/state`, copies only identity components
into its own ECM, and requires a unique `World+Name`, unique `Model+Name` whose
`ParentEntity` is that world, and a unique `base_link` owned by that model. It
never commands or changes the server. The Python wrapper binds the raw ECS
snapshot, registered component type IDs, executable hash, runtime/meta hashes,
PID start ticks and boot ID. It rejects stale/future evidence and verifies the
same runtime again immediately before loading.

`load_disturbance(..., binding=entity_binding_path)` then uses the verified
world and actual entity ID. Omitting the new keyword preserves the historical
loader's `default`/1 behavior, which is insufficient for new physical
acceptance. The new acknowledgement manifest explicitly records
`force_experiment_complete=false`; full native waveform/status and independent
motion checks remain required. Existing `disturbance.cpp` and force-event
semantics are unchanged. The source-recovery table above records the original
loader hash; the opt-in wrapper change has a new source identity.

At an idle server checkpoint on 2026-10-22, the helper was compiled in the
separate `generated/gz_world_entity_probe_build_v1_20261022` directory. All six
C++ `--self-test` checks and all seven synthetic Python guard tests passed.
The build used g++ 11.4.0, gz-sim 8.15.0, gz-transport 13.6.0 and gz-msgs 10.4.0.
The binary SHA256 is
`12704f6820ce57642ea81e2e626b0ad06a80fb97ae8351d88283717465f21c96`.
`build_manifest.json` records the exact commands, source hashes and test results;
the same directory retains compilation and test logs. No live ECS query or
disturbance load has used this opt-in path at this checkpoint. On a new build,
use a **new** directory, not any binary bound to an experiment:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic scripts/gz_world_entity_probe.cpp \
  -o /absolute/new-build/gz_world_entity_probe \
  $(pkg-config --cflags --libs gz-sim8 gz-transport13 gz-msgs10)
/absolute/new-build/gz_world_entity_probe --self-test
```

The live wrapper can then be called from the owning runner with its existing
`pump` callback, or from its read-only CLI. It inherits the caller's GZ
transport environment. The actual executable, native self-test, raw ECS
response, runtime identity, task source archive and library hashes must be
reviewed before using a verified binding to load the 1 N / 0.3 s schedule.
The six C++ tests exercise synthetic ECS construction and identity rejection;
the Python tests use mocked transport. Neither establishes live service
compatibility, successful wrench application or disturbance recovery.

## Exact provenance and future builds

For this server installation the plant source SHA256 is
`310485f2d2eba8c611008b9b8b3e1e60d94759b465497086f5e64e73eb3477af` and the
v2 binary SHA256 is
`c021a1f6ceb5e8216db1d455eb6f7b89e837e0d0f92d980c49d23f3706fb8f1e`.
The exact deployed source bytes include a trailing CRLF added during the
original transfer; the local working files were restored to those exact bytes
without changing the server or its bound manifests. Git LF normalization,
another compiler or changed paths can produce different raw source/binary
hashes. On a new checkout, rebuild, regenerate the complete plant manifests and
rerun predictor oracle/native checks from the actual source; do not reuse old
manifests as if the hashes were unchanged. Launch source ZIPs must retain the
actual deployed source bytes used for each experiment.
