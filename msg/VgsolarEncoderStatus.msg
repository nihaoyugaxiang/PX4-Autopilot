uint64 timestamp					# publication time since system start (microseconds)
uint64 timestamp_sample				# latest accepted quadrature edge

int64 right_count				# signed raw transitions; not wheel revolutions
int64 left_count
int32 right_delta				# count change since previous publication
int32 left_delta
uint32 right_transitions			# accepted phase transitions
uint32 left_transitions
uint32 right_invalid_transitions		# two-bit phase jumps rejected by decoder
uint32 left_invalid_transitions
int8 right_direction				# -1, 0 (no movement in sample), +1 phase order
int8 left_direction
bool capture_active				# GPIO edge callbacks are installed
