// SPDX-License-Identifier: BSD-3-Clause

/**
 * Enable VGSolar left and right DYP-A02 inputs at boot
 *
 * UART8 / ttyS7 is left (orientation 7), USART2 / ttyS1 is right
 * (orientation 1). Read-only inputs; no actuator or edge-stop control.
 *
 * @boolean
 * @reboot_required true
 * @group VGSolar
 */
PARAM_DEFINE_INT32(VGS_RNG_EN, 0);
