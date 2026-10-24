# Execution revision 18: source timing observation

This directory preserves eight original, finalized server artifacts from deployment v31. It was collected while the next hold experiment was running. No running flight log, raw flight data or current mutable status was copied. `collection_receipt.json` binds each original file by its server path, byte count and SHA-256; all eight copied files were verified locally.

The change adds bounded, preallocated source-pump phase and natural-GC observations. A pump that raises during its first supervision poll is retained; observation freezes before recovery can replace the fault evidence. The existing source issue time, model, C++ numerical backend, controller parameters and timing thresholds remain unchanged. This is an observation improvement, **not evidence that the source stall is fixed or that a flight has passed**.

The original logs show 39 executed, passing targeted tests:

| Test group | Executed tests | Original log |
| --- | ---: | --- |
| Source activity trace | 11 | `execution_v31_tests.log` |
| Supervision lifecycle | 7 | `execution_v31_tests.log` |
| Failed recovery handoff | 11 | `execution_v31_tests.log` |
| Fault latch | 3 | `execution_v31_tests.log` |
| Ground preparation | 7 | `execution_v31_ground_tests.log` |
| Total | 39 | Both logs, both exit codes 0 |

The first log also retains an initial `Ran 0 tests` result caused by a misspelled discovery pattern. That invocation contributes **zero** tests to the total. Ground preparation was then executed with the correct pattern, producing seven passing tests in the separate ground log. The original unsuccessful discovery is not removed or counted as coverage. Collection did not rerun any tests.

`prepare_hold_v25.py` and `verify_source_activity_v1.py` are immutable copies of the preparation and observation-verification scripts, not results. A new complete hold must pass its original physical/runtime audit and the source-activity audit before ordinary figure-eight progression is permitted. Fast figure-eight remains later. This directory contains no claim that hold25, ordinary figure-eight, hardware at 24 V, Orin deployment or hard real-time behavior has been accepted.

The earlier v17 software qualification and failed hold24 evidence remain in sibling directories. This small deployment archive is not a portable replacement for their full external evidence dependencies.
