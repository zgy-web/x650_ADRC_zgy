uint16 schema_version 1
uint64 session_id
uint64 epoch
# 0 snapshot, 1 proposal, 2 gate event, 255 final.
uint8 stream
uint64 sequence
bool persisted
bool complete
bool failed
# Per-row hashes may be zero; final receipt binds the manifest hash.
uint8[32] sha256
