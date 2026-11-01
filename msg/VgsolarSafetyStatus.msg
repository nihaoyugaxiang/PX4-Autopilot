# VGSolar edge stop request consumed by mode and differential output gates; hardware stopping still needs acceptance.
uint64 timestamp
uint64 left_timestamp
uint64 right_timestamp

float32 left_distance_m
float32 right_distance_m
float32 max_distance_cm
uint32 debounce_ms
uint32 unsafe_duration_ms
uint32 clear_count
uint32 rejected_clear_count

uint8 state
uint8 STATE_WAITING = 0
uint8 STATE_SAFE = 1
uint8 STATE_PENDING = 2
uint8 STATE_LATCHED = 3
uint8 STATE_CONFIG_INVALID = 4
uint8 STATE_STOPPED = 5

bool config_valid
bool left_valid
bool right_valid
bool left_unsafe
bool right_unsafe
bool clear_eligible
bool latched
bool stop_request
bool enabled
