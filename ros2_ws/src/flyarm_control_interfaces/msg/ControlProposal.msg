# This is an offline proposal, never a hardware or PX4 output message.
uint16 schema_version 1
uint64 session_id
uint64 epoch
uint64 sequence
bool replay_only true
uint8[32] configuration_sha256
uint64 snapshot_sequence
uint64 compute_start_monotonic_ns
uint64 compute_end_monotonic_ns
# 0 success, 1 input rejection, 2 solve rejection, 3 deadline rejection.
uint8 status
float64 control_time_s
float64 task_time_s
float64[3] position_reference_enu
float64[3] velocity_feedforward_enu
float64[3] acceleration_feedforward_enu
float64[3] base_velocity_target
float64[2] joint_velocity
bool accepted_optimum
bool nonlinear_check_passed
float64 primal_violation
float64 kkt_residual
uint32 subproblems
