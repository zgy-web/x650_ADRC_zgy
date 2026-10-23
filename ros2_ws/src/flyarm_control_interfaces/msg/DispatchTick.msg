# Runtime contract test ONLY. This message is not an actuator command.
uint64 session_id
uint64 epoch
uint64 sequence
uint64 scheduled_monotonic_ns
uint64 actual_monotonic_ns
uint64 source_sequence
uint64 source_received_monotonic_ns
uint64 command_sequence
uint64 command_issued_monotonic_ns
uint64 command_age_ns
# 0 waiting; 1 valid new; 2 valid repeated; 3 latched fault; 4 stopped.
uint8 status
uint16 reason
bool output_valid
bool repeated
ControlProposal command
