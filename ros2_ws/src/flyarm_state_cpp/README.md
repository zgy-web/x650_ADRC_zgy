# C++ raw state source (simulation adapter)

`state_source` receives the real PX4 estimated local position, quaternion,
angular velocity and estimator flags, together with ROS `/clock`. It tails the
actual TS20 plugin state and MIT packet CSV files. It does not read Python state
vectors, optimizer matrices, Gazebo base truth, planned packets, or the proposed
joint command as a measurement. The independent C++ physical-consumption reader
and actual-send admission gate remain in place.

The package is a migration candidate. A successful offline conversion or software
probe does not qualify a new full hold, figure-eight flight, or real hardware.

## Launch and one-time original binding

Start the process on the ground, with no active command output. Required ROS
parameters are `simulation_live=true`, positive `expected_session_id` and
`expected_epoch`, `configuration_sha256`, existing absolute `state_csv` and
`packet_csv`, a new `ready_file` and new `evidence_directory`, exact frozen
`position_gains` (three doubles), `bus_period_s`, `delay_s`, and
`trajectory_kind=hold|figure8`. A figure-eight additionally uses
`amplitude_x`, `amplitude_z`, `period_s`, `cycles`, `start_s`, `ramp_s`; its
center is the actual `LiveStart.target_position_enu`. The current C++ worker may
support only hold; enabling source figure-eight alone does not enable a flight.

The ground marker means **UNBOUND**: streams, actual causal joint records and DDS
connections are ready. The parent retains legacy output ownership and sends one
`SourceBinding` on `/flyarm/isolated/v1/source_binding` after the original
reference and reset guards have been bound. It copies the original reference
document; the source never substitutes its own newest estimator origin. Require
the exact positive `SourceBindingAck` (session, epoch, hash, sequence and original
issued monotonic time) before sending `LiveStart`. Both are reliable depth 1.
The binding sequence is exactly 1, and binding/LiveStart issue ages are at most
25 ms. Stale, duplicate, malformed or mismatched bindings fail closed. The
source checks all original frame/EV/reset/age guards before acknowledging.

No `ControlSnapshot` is published before binding plus actual `LiveStart`. Once
active, each newly observed increasing `/clock` can supply a single snapshot on
`/flyarm/isolated/v1/source` (best effort depth 1). `issued_monotonic_ns` is taken
before the CSV selection. The placeholder `dt_s=.02` is overwritten by the
existing admission node with the actual source-time difference. The angular
callback's held `/clock` receipt is `state_time_s`; it is not advanced by a
position or clock callback. PX4 acceleration is an earth-fixed NED velocity
derivative mapped to ENU without gravity addition or truth substitution.

TS20 position is the actual physical encoder and velocity is the same-stamp
conditional servo endpoint speed. The exact selected q/dq/r and already-read
histories form one owned snapshot. Only physical consumed counters/receipt times
and actual MIT sends enter the history; command availability is also guarded by
the observation row time. Future rows cannot supply a control observation.
Contexts exceeding 64 entries fail instead of silently truncating. The original
25 ms joint-age bound, 0.0001 rad physical/internal consistency, bus cadence and
delay checks remain unchanged.

## Evidence

No JSON encoding, diagnostic publication or evidence-file writing occurs in the
ACTIVE loop. The raw-wire buffer has a 128 MiB bound and selection metadata a
100,000-entry bound; exhaustion is a fault, not a lossy recording policy. CSV
reads remain input I/O from the actual simulator, with a 1 MiB per-file poll cap
and no use of partial final rows. On stop/exit the node writes:

* `source_wire.cdr`: magic `FSCdr001`, followed by native little-endian records
  `uint32 kind, uint64 monotonic_ns, uint64 sequence, double receipt_clock_s,
  uint32 cdr_bytes, byte[cdr_bytes]`. Kinds 1–5 are original local position,
  attitude, angular velocity, flags, clock; 6 LiveStart; 7 ControlSnapshot;
  8 RuntimeStop; 9 SourceBinding; 10 SourceBindingAck. Input timestamps precede
  copying/serialization; snapshot record time is selection completion, **not DDS
  subscriber arrival**. CDR includes raw PX4 timestamp fields.
* `source_selections.csv`: original issue, selection and publish-return clocks,
  all callback sequences and selected source/history identities.
* `source_final.json`: lifecycle, identity, counts, input paths and error.

A source exception publishes the matching one-way `RuntimeStop` and exits with
failure. It cannot emit actuator setpoints. The parent owns safe mode handoff,
landing, parameter restoration and process cleanup. A crash or `SIGKILL` can
prevent final evidence persistence; missing final evidence cannot pass audit.

## Tests and limits

`test_source_core` exercises coordinate/acceleration mapping, held angular time,
origin/reset/flags/receipt guards, no commit after a rejected estimator sample,
causal and immutable joint selection, actual MIT packet identity and cadence,
consumed-command availability, CSV partial rows, and C2 trajectory derivatives.

`test/compare_recorded_source.py` invokes the C++ `replay_source_core` executable
against original run artifacts. It preserves each original selected state/MIT
visibility prefix and compares all state and complete history values against
the original admitted snapshot CDR. It independently checks position/attitude
raw fields against timestamp-matched original ROS bag messages. In hold25 the
bag did not record raw angular-velocity or estimator-flags CDR: those two rely
on preserved original raw-field/receipt snapshots, explicitly reported. No
retiming, response fitting or online latency qualification is performed.

The first hold25 replay checked 1,056 original snapshots, with zero numerical
differences in all compared fields and 1,056 exact bag matches for each of
position and attitude. This validates conversion and selection on those inputs;
it does not prove the new ROS process meets live scheduling deadlines.
