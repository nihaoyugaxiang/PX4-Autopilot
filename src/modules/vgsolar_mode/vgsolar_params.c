
/**
 * Enable the VGSolar robot mode.
 * @boolean
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_ENABLE, 0);
/**
 * Enable cleaner servo and valve/pump outputs after wiring validation.
 * @boolean
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_CLN_EN, 0);
/**
 * Default navigation speed in m/s.
 * @min 0.1
 * @max 5
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_CRUISE_SPD, 1.0f);
/**
 * Turn timeout in seconds.
 * @min 5
 * @max 60
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_TURN_TO, 15.0f);
/**
 * Moving turn speed in m/s.
 * @min 0.1
 * @max 1
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_TURN_SPD, 0.3f);
/**
 * Navigation arrival radius in m.
 * @min 0.05
 * @max 20
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_NAV_RAD, 2.0f);
/**
 * Use the debounced lift IR input.
 * @boolean
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SC_IR, 1);
/**
 * Require calibrated pressure feedback.
 * @boolean
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SC_PRESS, 1);
/**
 * Raised lift pulse width in us.
 * @min 1000
 * @max 2000
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_SC_PWM_R, 1900.f);
/**
 * Lowered lift pulse width in us.
 * @min 1000
 * @max 2000
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_SC_PWM_L, 1100.f);
/**
 * Lift slew rate in us/s.
 * @min 50
 * @max 5000
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_SC_RATE, 400.f);
/**
 * Lift settle delay in ms.
 * @min 0
 * @max 5000
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SC_LIFT_DLY, 2000);
/**
 * Vacuum timeout or fallback delay in ms.
 * @min 100
 * @max 10000
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SC_VAC_DLY, 10000);
/**
 * Vent delay in ms.
 * @min 100
 * @max 5000
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SC_VENT_DLY, 2000);
/**
 * Whole suction action timeout in ms.
 * @min 1000
 * @max 60000
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SC_ACT_TO, 30000);
/**
 * Lift arrival timeout in ms.
 * @min 500
 * @max 15000
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SC_LIFT_TO, 5000);
/**
 * Vacuum threshold in kPa.
 * @min -100
 * @max -1
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_SC_VAC_KPA, -70.f);
/**
 * Vacuum debounce in ms.
 * @min 0
 * @max 5000
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SC_VAC_DEB, 300);

/**
 * Vacuum loss hysteresis in kPa.
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_SC_VAC_HYST, 5.f);
/**
 * Navigation braking deceleration in m/s^2.
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_NAV_DEC, .5f);
