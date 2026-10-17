# Frozen regression reference

These six Python modules and `selected_parameters.json` are byte-for-byte copies
of the validated `benchmark/whole_body_closed_loop_v1` implementation. They are
used only as the independent reference for the package regression tests.

Their original paths and SHA-256 values are recorded in
[`flyarm_pid_ff/data/provenance.json`](../../../flyarm_pid_ff/data/provenance.json).
The test loader verifies every Python source hash before importing this reference;
the PID comparison verifies the parameter JSON hash. Git attributes preserve these
files' exact bytes on Windows and Linux.

Do not update this snapshot to make a failing controller test pass. A new validated
controller version needs an explicit baseline and provenance update. The runtime
package does not import these files, and they are excluded from the installed wheel.
