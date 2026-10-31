// SPDX-License-Identifier: BSD-3-Clause
// 轮速换算保持关闭，直到按实测配置输出轴每圈解码计数和前进方向。

/**
 * Right encoder transitions per driven-wheel revolution
 *
 * Count all valid A/B transitions for one revolution of the driven wheel,
 * after gearbox reduction. Zero disables right wheel angle and speed.
 *
 * @min 0
 * @max 1000000
 * @group VGSolar Encoder
 */
PARAM_DEFINE_INT32(VGS_ENC_R_CPR, 0);

/**
 * Left encoder transitions per driven-wheel revolution
 *
 * Count all valid A/B transitions for one revolution of the driven wheel,
 * after gearbox reduction. Zero disables left wheel angle and speed.
 *
 * @min 0
 * @max 1000000
 * @group VGSolar Encoder
 */
PARAM_DEFINE_INT32(VGS_ENC_L_CPR, 0);

/**
 * Reverse right encoder direction
 *
 * Set to one when a physically forward right wheel rotation decrements raw
 * counts. This must be checked on the vehicle.
 *
 * @boolean
 * @group VGSolar Encoder
 */
PARAM_DEFINE_INT32(VGS_ENC_R_REV, 0);

/**
 * Reverse left encoder direction
 *
 * Set to one when a physically forward left wheel rotation decrements raw
 * counts. This must be checked on the vehicle.
 *
 * @boolean
 * @group VGSolar Encoder
 */
PARAM_DEFINE_INT32(VGS_ENC_L_REV, 0);
