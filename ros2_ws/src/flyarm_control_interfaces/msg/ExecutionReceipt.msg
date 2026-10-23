uint64 session_id
uint64 epoch
uint64 command_sequence
uint64 dispatch_sequence
uint64 observed_monotonic_ns
float64 observation_sim_s
bool joint_consumed
bool pva_send_returned
bool failed
uint64[2] consumed_counter
float64[2] consumed_sim_s
# PX4 SET_POSITION_TARGET_LOCAL_NED has no per-message ACK. Its reception
# requires independent ULog audit and is not claimed by this receipt.
