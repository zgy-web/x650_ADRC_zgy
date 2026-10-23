# Offline control recorder, phase one

This independent ROS 2 process records the fixed `flyarm_control_interfaces`
messages. It has no PX4, joint-command, Gazebo or hardware publisher. Reliable
DDS delivery is not a disk acknowledgement; `RecordingReceipt` is published
only after the corresponding file write, flush and `fsync` return.

```bash
ros2 run flyarm_control_recorder recorder_node --ros-args \
  -p output_dir:=/absolute/new/recording \
  -p expected_session_id:=123 -p expected_epoch:=1 \
  -p prefix:=/flyarm/offline/v1 -p capacity:=8192
```

The output directory must not exist. A private `ROS_DOMAIN_ID` must be chosen
by the offline test launcher. Capacity is a finite total across the three
streams, between 1 and 8192 rows. CDR payloads also have a fixed 256 KiB bound.
Session and epoch identifiers must be positive.

Topic suffixes are `input/snapshot`, `output/proposal`, `output/gate_event`,
`recorder/input/drain` and `recorder/output/receipt`. Stream identifiers are
0, 1 and 2 respectively. Each stream starts at sequence 1 and is independently
contiguous; arrival order across different topics is unrestricted. Wrong
schema/session/epoch, `replay_only=false`, missing/reordered/duplicate sequences,
changed configuration digests and overflow fail closed. The recorder archives
the original serialized CDR payload and its SHA256, not reconstructed floats.

`DrainRequest.final_sequences` declares the final sequence in each stream.
At least one snapshot is required; an empty drain cannot certify a session.
An early drain queues a bounded expectation; it never waits inside a callback.
Further matching messages complete it, or the wall-clock `drain_grace_s`
(default 2 s, at most 30 s) expires. Only a complete drain closes the JSONL,
checks the file hash and source identity, writes and fsyncs its final manifest,
then emits a successful final receipt. Its stream is 255, sequence is the total
row count, and SHA256 binds the exact `recording_manifest.json` bytes. This
receipt certifies persistence only. It is not control acceptance or execution.

The directory contains `records.jsonl`, `recording_manifest.json`,
`session.json`, `sources.json` and `sources.zip`. Package Python files and the
actual installed typed-interface definitions are captured before subscriptions
start. The configuration digest comes from the first persisted snapshot and
must match every proposal, including one that arrives before the snapshot.
Per-row receipts hash the exact JSONL line including its newline. Each row also
records the previous row hash. Failed recording cannot emit a successful final
receipt; a hard-killed recorder may have no final manifest at all.

The pure byte/metadata unit checks do not require ROS:

```bash
python3 -m unittest discover -s test -p 'test_*.py' -v
```

The orchestration task separately checks real typed CDR roundtrips, receipt
hashes and worker/recorder process deaths in an isolated ROS domain. No live
flight integration or real-time MPC qualification is claimed here.
