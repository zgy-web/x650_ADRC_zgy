# Exact binding validation; not a mode or actuator handoff.
uint64 session_id
uint64 epoch
uint8[32] configuration_sha256
uint64 binding_sequence
uint64 binding_issued_monotonic_ns
uint64 observed_monotonic_ns
bool accepted
# 0 accepted; 1 malformed/stale/duplicate; 2 raw-state/reference mismatch.
uint16 rejection_code
