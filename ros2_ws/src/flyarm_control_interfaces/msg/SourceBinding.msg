# Original one-shot reference handoff, before LiveStart, while legacy owns output.
uint16 schema_version 1
uint64 session_id
uint64 epoch
uint8[32] configuration_sha256
uint64 binding_sequence
uint64 issued_monotonic_ns
string<=40 px4_source_contract_commit
string<=32 frame_mode
bool target_frozen
bool fit_estimator_translation
float64 ref_lat
float64 ref_lon
float64 ref_alt
uint64 ref_timestamp
bool xy_global
bool z_global
uint8 xy_reset_counter
uint8 z_reset_counter
uint8 heading_reset_counter
uint8 vxy_reset_counter
uint8 vz_reset_counter
uint8 quat_reset_counter
float64[3] position_gains
float64 bus_period_s
float64 delay_s
