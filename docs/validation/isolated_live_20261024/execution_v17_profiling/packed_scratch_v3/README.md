# Packed native scratch qualification v3

This directory contains the compact results of one scratch-only qualification
run: **25/25 tests, 30/30 frozen matrix cases, 388/388 historical calls passed**.
This is not a production certificate and not a flight acceptance. The previous
failed v1/v2 runs remain unchanged on the server.

Full evidence for all 418 matrix/replay rows remains under the original server
paths in `qualification_summary.json` and the two main reports. Every row is
bound by path, size and SHA-256. The qualifier reopened and verified all rows
and both complete GC traces. This compact directory omits those full rows and
**cannot independently reproduce or revalidate the complete numerical proof**.

Natural GC stayed enabled with thresholds `[700, 10, 10]` and zero frozen
objects; no forced collection, disabled collection or relaxed deadline was
used. GC trace buffers did not overflow. Reported timing includes observation
overhead and describes an offline test on this server, not flight timing.

The next gate is a fresh qualification of the integrated production sources,
followed by a new full end-effector-hold acceptance.
