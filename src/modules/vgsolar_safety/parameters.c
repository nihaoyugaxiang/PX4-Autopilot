// SPDX-License-Identifier: BSD-3-Clause
// 本步仅定义边缘判断参数，不定义执行器或整车急停控制参数。

/**
 * Start VGSolar edge monitor at boot
 *
 * Request/diagnostic only; this module does not stop actuators.
 *
 * @boolean
 * @reboot_required true
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_SAFE_EN, 0);

/**
 * Maximum safe outer rangefinder distance in centimeters
 *
 * Either invalid sensor or distance above this limit requests a stop
 * after VGS_RF_DBNC. Distance equal to the limit is safe.
 *
 * @min 1
 * @max 450
 * @decimal 1
 * @group VGSolar
 */
PARAM_DEFINE_FLOAT(VGS_RF_MAX, 15.f);

/**
 * Continuous rangefinder unsafe debounce time
 *
 * @unit ms
 * @min 10
 * @max 2000
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_RF_DBNC, 100);
