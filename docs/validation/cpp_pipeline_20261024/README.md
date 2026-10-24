# C++ pipeline software evidence, 2026-10-24

`summary.json` gives the final version's measured intervals and source report
identities. `inputs_matrix_and_ros_records.zip` has SHA-256
`20e4f1647980219008369275dee21994e44f0a67b9f98ea646e74c3176439ca3`
and contains:

- all 30 frozen raw input/oracle cases and full native mathematical results;
- all 1,056 historical raw input cases and the complete per-case comparison;
- all raw CDR, compact native outputs, phase timings and lifecycle evidence for
  the real C++ ROS worker positive replay and latched bad-hash negative case;
- a member-by-member SHA-256 manifest.

The full historical native matrix output is additionally retained on the server
in the 151,170,596-byte archive identified by `summary.json.server_full_archive`
(SHA-256 `15b9d4cf09e6bf5c15609ed66c333cbd86c09fca7ec3ac0f8f7503dcfb3a726c`).
Its large repeated matrices are reproducible from the included input cases and
bound core; this Git bundle includes the complete comparison report instead of
duplicating those matrices. The full archive is not claimed to be in Git.

These are software tests using preserved actual inputs. They do not change the
old hold25 failure outcome and do not certify a new 30-second physical hold.
ROS worker timing excludes the real source, admission, dispatcher and physical
actuator path. See [the architecture and acceptance limits](../../X650_CPP_PIPELINE.md).
