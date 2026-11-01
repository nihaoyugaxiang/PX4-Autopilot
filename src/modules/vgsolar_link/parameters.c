/**
 * Wheel radius for NCU track velocity feedback in m
 *
 * Zero marks uncalibrated feedback. Set only after wheel radius and encoder counts are verified.
 *
 * @min 0
 * @max 1
 * @unit m
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_WHL_RAD, 0.f);

/**
 * Use calibrated VGSolar wheel speed for differential speed feedback
 *
 * Requires active capture, both valid encoder CPR/directions and VGS_WHL_RAD.
 * Applies to the native differential speed controller as well as VGSolar motion
 * validation. Missing/stale feedback stops output; no automatic estimator fallback.
 * Configure while disarmed. Does not fuse wheel odometry into the EKF.
 *
 * @boolean
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_WHL_CTRL, 0);

/**
 * Battery percentage: 0 native, 1 project voltage mapping.
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_BAT_SRC, 1);
/**
 * Pack voltage at zero percent in V.
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_BAT_MIN, 19.8f);
/**
 * Pack voltage at full charge in V.
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_BAT_MAX, 25.2f);

/**
 * Communication event logging: 0 off, 1 changed/5Hz speed, 2 every command.
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_CC_LOG, 1);
