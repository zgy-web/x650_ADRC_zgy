uint16 schema_version 1
uint64 session_id
uint64 epoch
# Final snapshot, proposal and gate-event sequence numbers, in that order.
uint64[3] final_sequences
bool replay_only true
