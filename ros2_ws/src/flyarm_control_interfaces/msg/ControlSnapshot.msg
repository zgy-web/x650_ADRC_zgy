# Phase 1 offline shadow contract. Context history times are task-relative.
uint16 schema_version 1
uint64 session_id
uint64 epoch
uint64 sequence
bool replay_only true
uint8[32] configuration_sha256
uint64 issued_monotonic_ns
float64 control_time_s
float64 task_time_s
float64 dt_s
float64 state_time_s
float64 joint_time_s
float64[3] position_enu
float64[3] velocity_enu
float64[3] omega_body_flu
float64[3] acceleration_enu
float64[9] rotation_row_major
float64[2] q
float64[2] dq
float64[3] target_position_enu
float64[3] target_velocity_enu
float64[3] target_acceleration_enu
float64[3] position_gains
float64 reference_time_s
float64 bus_origin_s
float64[2] reference_r
uint32 command_count
uint32 packet_count
float64[64] command_time_s
float64[64] packet_send_time_s
# Histories below are flattened [count, 2]; unused entries must be zero.
float64[128] command_velocity
float64[128] packet_q_ref
float64[128] packet_dq_ref
float64[128] packet_tau_ff
float64[128] packet_kp
float64[128] packet_kd
