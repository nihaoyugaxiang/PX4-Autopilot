/****************************************************************************
 *
 *   Copyright (C) 2020, 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/** VGSolar FMU V3.1: bench-only PX4 Rover bring-up. */
#pragma once
#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

#define GPIO_nLED_RED  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define GPIO_nLED_BLUE (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED LED_RED
#define BOARD_ARMED_STATE_LED LED_BLUE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE
#define ADC1_CH(n) (n)
#define PX4_ADC_GPIO GPIO_ADC12_INP4, GPIO_ADC12_INP18, GPIO_ADC123_INP10, GPIO_ADC123_INP11
#define ADC_BATTERY_VOLTAGE_CHANNEL ADC1_CH(4)   /* PC4 */
#define ADC_BATTERY_CURRENT_CHANNEL ADC1_CH(18)  /* PA4 */
#define VGSOLAR_PRESSURE_ADC_CHANNEL ADC1_CH(10) /* PC0，复用board_adc发布的数据。 */
#define ADC_CHANNELS ((1 << 4) | (1 << 18) | (1 << 10) | (1 << 11))

/* 仅将五个已核对的引脚交给原生PWM驱动；四个编码器引脚继续保持输入。
 * 开机输出功能默认禁用，手动测试也只允许在机构动力断开的台架上进行。
 */
#define DIRECT_PWM_OUTPUT_CHANNELS 5
#define BOARD_NUM_IO_TIMERS 3
#define HRT_TIMER 8
#define HRT_TIMER_CHANNEL 3
#define BOARD_DMA_ALLOC_POOL_SIZE 5120
#define BOARD_HAS_ON_RESET 1
#define BOARD_ENABLE_CONSOLE_BUFFER
#define SDIO_SLOTNO 0
#define SDIO_MINOR 0
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_SERVO_VALID 0
#define RC_SERIAL_PORT "/dev/ttyS6"

#define VG_INPUT(port,pin) (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORT##port|GPIO_PIN##pin)
#define GPIO_VG_IR_INPUT VG_INPUT(B,5) /* 原AP红外GPIO98，只读下拉输入。 */
#define VG_OUTPUT_LOW(port,pin) (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORT##port|GPIO_PIN##pin)
#define VG_OUTPUT_HIGH(port,pin) (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORT##port|GPIO_PIN##pin)
#define GPIO_VG_VALVE_OFF VG_OUTPUT_HIGH(E,5) /* Active-low valve: OFF is HIGH. */
#define GPIO_VG_PUMP_OFF VG_OUTPUT_LOW(E,6)
#define GPIO_VG_MCU_K VG_OUTPUT_LOW(E,2) /* Match existing AP hardware definition. */
#define GPIO_VG_IMU_CS VG_OUTPUT_HIGH(E,12)
#define GPIO_VG_FRAM_CS VG_OUTPUT_HIGH(D,4)
#define GPIO_VG_ENCODER_RIGHT_A VG_INPUT(A,8)  /* 右轮编码器A相。 */
#define GPIO_VG_ENCODER_RIGHT_B VG_INPUT(E,11) /* 右轮编码器B相。 */
#define GPIO_VG_ENCODER_LEFT_A VG_INPUT(A,0)   /* 左轮编码器A相。 */
#define GPIO_VG_ENCODER_LEFT_B VG_INPUT(A,1)   /* 左轮编码器B相。 */
#define GPIO_VG_ACTUATOR_INPUTS \
    VG_INPUT(E,13), VG_INPUT(E,14), VG_INPUT(D,14), VG_INPUT(D,15), VG_INPUT(C,8), \
    GPIO_VG_ENCODER_RIGHT_A, GPIO_VG_ENCODER_RIGHT_B, GPIO_VG_ENCODER_LEFT_A, GPIO_VG_ENCODER_LEFT_B
#define PX4_GPIO_INIT_LIST { \
    PX4_ADC_GPIO, GPIO_VG_IMU_CS, GPIO_VG_FRAM_CS, GPIO_VG_ACTUATOR_INPUTS, \
    GPIO_VG_VALVE_OFF, GPIO_VG_PUMP_OFF, GPIO_VG_MCU_K, \
    VG_OUTPUT_LOW(A,10), VG_OUTPUT_LOW(B,12), VG_OUTPUT_LOW(B,13), VG_OUTPUT_LOW(E,15), \
    VG_INPUT(C,13), GPIO_VG_IR_INPUT, VG_INPUT(E,3), VG_INPUT(B,1) \
}

__BEGIN_DECLS
#ifndef __ASSEMBLY__
int stm32_sdio_initialize(void);
void stm32_spiinitialize(void);
void stm32_usbinitialize(void);
void board_peripheral_reset(int ms);
#include <px4_platform_common/board_common.h>
#endif
__END_DECLS
