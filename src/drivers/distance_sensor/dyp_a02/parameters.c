// SPDX-License-Identifier: BSD-3-Clause

// 本板两路超声波开机启动开关：0关闭，1启用；保存后重启生效。
// 默认关闭，便于先用NSH手动验证左右串口；不控制执行器，不改校准。
// 下方结构化参数说明供PX4参数工具生成元数据，保持其原有标签与默认值。
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
