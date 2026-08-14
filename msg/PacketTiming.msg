# Optional companion metadata for one EventPacket published on events_raw.
# These are driver-observation timestamps, not decoded sensor event timestamps.

uint32 SCHEMA_VERSION=1
uint32 FLAG_NONE=0

uint32 schema_version
std_msgs/Header packet_header
uint64 sequence
uint32 stream_epoch

int64 first_receive_steady_ns
int64 first_receive_system_ns
int64 last_receive_steady_ns
int64 last_receive_system_ns
int64 event_publish_steady_ns
int64 event_publish_system_ns

uint32 callback_count
uint64 packet_bytes
uint32 flags
