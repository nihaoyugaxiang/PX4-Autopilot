// SPDX-License-Identifier: BSD-3-Clause
// 本步只定义压力转换和红外输入参数，不包含泵、阀或舵机控制参数。

/**
 * Start VGSolar pressure and infrared input module at boot
 *
 * @boolean
 * @reboot_required true
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_IN_EN, 0);

/**
 * Suction pressure lower voltage calibration endpoint
 *
 * @unit V
 * @min 0.02
 * @max 3.28
 * @decimal 3
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_P_VMIN, 0.1f);

/**
 * Suction pressure upper voltage calibration endpoint
 *
 * @unit V
 * @min 0.02
 * @max 3.28
 * @decimal 3
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_P_VMAX, 3.1f);

/**
 * Gauge pressure at lower voltage endpoint in kPa
 *
 * @min -150
 * @max 50
 * @decimal 2
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_P_PMIN, 0.f);

/**
 * Gauge pressure at upper voltage endpoint in kPa
 *
 * @min -150
 * @max 50
 * @decimal 2
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_P_PMAX, -100.f);

/**
 * Suction pressure low pass cutoff, zero bypasses filter
 *
 * @unit Hz
 * @min 0
 * @max 20
 * @decimal 1
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_P_FILT, 5.f);

/**
 * Invert lift infrared interpretation
 *
 * Zero means high=blade present, low=no blade. Neither level alone proves
 * mechanical lowering or sensor connection.
 *
 * @boolean
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_IR_INV, 0);

/**
 * Lift infrared stable level debounce duration
 *
 * @unit ms
 * @min 0
 * @max 500
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_IR_DEB, 30);
