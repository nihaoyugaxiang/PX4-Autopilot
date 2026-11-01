uint64 timestamp
uint8 kind # 1 command result, 2 mode/phase edge, 3 watchdog, 4 vacuum warning/recovery, 5 turn result
uint8 command
uint8 state
uint8 turn_phase
bool accepted
uint8 reason # 0 accepted,1 disarmed,2 estop,3 suction,4 hold,5 navigation,6 invalid/other,8 timeout,9 heading/feedback
int16 velocity_cms
int16 yaw_data
uint16 turn_angle_cd
uint16 turn_rate_cds
uint8 turn_direction
uint8 turn_mode
uint8 previous_state
uint8 previous_phase
uint8 ORB_QUEUE_LENGTH = 16
