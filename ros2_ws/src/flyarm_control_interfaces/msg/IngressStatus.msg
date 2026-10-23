# Same-host monotonic receipt; not a synchronized distributed clock.
uint64 session_id
uint64 epoch
uint64 source_sequence
uint64 received_monotonic_ns
uint64 source_issued_monotonic_ns
float64 control_time_s
bool faulted
