# Current VGSolar mode state exported to the NCU status bridge.
uint64 timestamp
uint64 navigation_event_timestamp # distinct requests/terminal events may have the same state
uint8 state
uint8 STATE_STANDBY = 0
uint8 STATE_YAW = 1
uint8 STATE_YAWRATE = 2
uint8 STATE_TURN = 3
uint8 STATE_NAVIGATION = 4
uint8 STATE_ESTOP = 5
float32 velocity_m_s
bool active
uint8 turn_phase
uint8 TURN_IDLE = 0
uint8 TURN_STOPPING = 1
uint8 TURN_WAIT_STOPPED = 2
uint8 TURN_LOWER_SUCTION = 3
uint8 TURNING = 4
uint8 TURN_RAISE_SUCTION = 5
uint8 navigation_state
uint8 NAV_IDLE = 0
uint8 NAV_ACTIVE = 1
uint8 NAV_ARRIVED = 2
uint8 NAV_FAILED = 3
uint8 NAV_CANCELLED = 4
uint8 nav_mode
uint32 nav_distance_cm
int16 nav_heading_error_cd
uint16 fault_code
bool turning_motion

uint8 mode_id
bool motion_allowed
uint8 suction_state
bool suction_frozen

bool vacuum_warning

bool cleaner_output_permitted
bool release_pending
