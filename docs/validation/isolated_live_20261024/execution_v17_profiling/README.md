# Execution v17 preparation: offline timing and representation evidence

These files are **offline scratch evidence**, not a physical endpoint-hold pass,
not formal numerical qualification, and not a real DDS timing guarantee. No
controller model, constraint, weight, 20 ms dispatch period, or 50 ms expiry was
changed by these probes. The original Hold23 failure remains recorded separately.

The input is the complete ordered set of 26 actual snapshots from
`runs/x650_isolated_live_v1_20261024_121409_13665`, with the original LiveStart and
initialization. Each replay constructs the real hash-checked live engine first.
Only the issue wall timestamp is rebased to permit offline execution. Scratch
variants replace three numerical classes in memory **after** the original
constructor checks. Their results must not be represented as source-bound
qualification of those replacement classes.

## Evidence groups

- `hold_v23_worker_profile_v1/phases/report.json`: phase-only measurements of all
  26 original inputs. The separate `profile/report.json` contains cProfile data
  for rounds 25 and 26; its times are instrumented and must not be mixed with
  phase-only or actual-flight timings.
- `hold_v23_packed_pair_v1`: three sequential fresh-process pairs, production
  then packed scratch, using the same 26 snapshots. Commands, selected full
  predicted-state/endpoint fields, candidate envelopes, and quality results
  match exactly after excluding the recorded `elapsed_ms` observation. Joint
  preparation and candidate phases improve, but whole-engine timing does not
  improve consistently in all three pairs; all raw results are retained.
- `hold_v23_deferred_pair_v1`: three fresh-process pairs, packed scratch then
  packed plus deferred diagnostic preparation. Every control field and the
  **complete diagnostic record**, including nested allocation, measured alpha,
  FF, prediction and candidate-envelope data, matches exactly after excluding
  only the timing keys listed in its summary.

All processes use the production CPU affinity `87,89,91,97` and one BLAS thread.
There is one preflight generation-2 collection, then a 0.2 s idle, followed by an
absolute 20 ms replay schedule. An overrun does not reset that schedule. Each
pair uses a new process; no best run was selected. CPU active frequency was not
sampled, so differences in unchanged work must not be assigned a specific cause.

## Interpretation

Phase-only rounds 25/26 place arm FF at 0.851/0.861 ms, owned diagnostic cloning
at 0.598/0.602 ms, engine decode at 0.715/0.709 ms, and command construction at
0.079/0.077 ms. FF and cloning explain approximately 1.45 ms of the outer path,
not its entire live 3.56/3.76 ms overhead.

For the deferred pairs, engine median times change from 9.855 to 8.237 ms,
9.477 to 8.222 ms, and 9.409 to 8.137 ms. Full callback proxy medians remain
similar: 10.321 to 10.194 ms, 9.904 to 10.036 ms, and 9.840 to 9.883 ms. This is
expected: the same diagnostic work moves **after proposal publication** and
finishes before synchronous serialization and the next solve. It is not removed.

The callback proxy executes the real `process_snapshot`, ROS CDR serialization,
pickle detachment, and Unix datagram diagnostic send. Its publisher is a no-op,
so the numbers exclude real DDS publication/delivery. Deferred completion is
hooked at the first serializer entry, after publication returns and before any
CDR or pickle operation. The later production integration uses an explicit
completion hook and additionally copies measured omega; that production change
is not silently substituted into these preserved scratch files.

## Provenance and limits

`retrieval_manifest.json` lists each original server path, size, and SHA256. The
listed files were copied byte-for-byte. Each report additionally binds original
task, initialization, LiveStart, diagnostic input, and source hashes. Reproduction
requires the corresponding qualified source revision and input files; merely
running these probes against a later task identity is not equivalent.

Large full numerical payloads, original CDR inputs, and binary `.prof` files are
left in the server artifact directories named by the reports. No archive is
needed here. The summaries retain the exact-comparison results and canonical
hashes. All numerical thresholds remain unchanged; real source-bound numerical
qualification and a new complete physical endpoint-hold run are still required.
