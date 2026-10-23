# Explicit one-shot simulation handoff. Not a hardware authorization.
uint64 session_id
uint64 epoch
uint8[32] configuration_sha256
uint64 issued_monotonic_ns
float64 origin_s
float64[3] anchor_position_enu
float64[9] anchor_rotation_row_major
float64[2] anchor_q
float64[3] target_position_enu
uint32 history_count
float64[64] history_time_s
float64[320] history_command
float64 ref_alt
float64 yaw_ned
