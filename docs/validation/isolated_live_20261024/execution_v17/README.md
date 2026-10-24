# Execution revision 17 / deployment v30: formal software evidence

The formal qualification completed with exit 0: **379 regression tests,
30 original matrix cases and 388 historical calls passed**, with zero skipped
tests. The formal native proof records successful verification of all 418
external full row-evidence files and both complete natural-GC traces.

This archive is a point-in-time software evidence snapshot. The included
`quiet_task_cpp_v10.json`, `execution_protocol_v17.json` and prepare status
belong to that snapshot. Ground preflight stopped with exit 1 before flight;
the shared recovery hook changed source identity and existing hover/probe
stage certificates were no longer current. The old certificates were not
waived. No complete hold acceptance or flight start is claimed here. New
prerequisite runs and a later task/protocol snapshot will be archived separately.

Full per-row evidence remains in the original server directories recorded in
`archive_manifest.json` and bound by path, size and SHA-256 in the main reports.
This compact archive includes the main reports, GC traces, code archive,
source hashes, build/install/test records and preflight failure snapshot. It
**does not contain all full rows and cannot independently revalidate the entire
proof without the bound external files and native/oracle dependencies**.

No successful numerical proof was modified during this collection.
