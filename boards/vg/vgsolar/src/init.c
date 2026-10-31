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

#include "board_config.h"
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <chip.h>
#include <stm32_uart.h>
#include <arch/board/board.h>
#include "arm_internal.h"
#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_dma_alloc.h>

__BEGIN_DECLS
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

__EXPORT void board_on_reset(int status)
{
    const uint32_t actuator_inputs[] = {GPIO_VG_ACTUATOR_INPUTS};
    px4_gpio_init(actuator_inputs, arraySize(actuator_inputs));
    px4_arch_configgpio(GPIO_VG_VALVE_OFF);
    px4_arch_configgpio(GPIO_VG_PUMP_OFF);
    if (status >= 0) { up_mdelay(100); }
}

__EXPORT void board_peripheral_reset(int ms)
{
    /* Sensor rails are fixed LDOs. PB2 is IMU DRDY, not a rail enable. */
#if !defined(BOOTLOADER)
    board_spi_reset(ms, 0xffff);
#else
    (void)ms;
#endif
}

__EXPORT void stm32_boardinitialize(void)
{
    board_on_reset(-1);
    board_autoled_initialize();
    const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
    px4_gpio_init(gpio, arraySize(gpio));
#if !defined(BOOTLOADER)
    stm32_spiinitialize();
#endif
    stm32_usbinitialize();
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
    (void)arg;
#if !defined(BOOTLOADER)
    px4_platform_init();
    if (board_dma_alloc_init() < 0) { syslog(LOG_ERR, "[vgsolar] DMA allocation failed\n"); }
#if defined(SERIAL_HAVE_RXDMA)
    static struct hrt_call serial_dma_call;
    hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)stm32_serial_dma_poll, NULL);
#endif
    drv_led_start();
    led_off(LED_RED);
    led_off(LED_BLUE);
    if (board_hardfault_init(2, true) != 0) { led_on(LED_RED); }
#ifdef CONFIG_MMCSD
    if (stm32_sdio_initialize() != OK) {
        syslog(LOG_ERR, "[vgsolar] SD initialization failed\n");
        led_on(LED_RED);
    }
#endif
    px4_platform_configure();
    syslog(LOG_INFO, "[vgsolar] PX4 bench: PWM functions disabled by default; disconnect actuator power for waveform tests\n");
#endif /* !BOOTLOADER */
    return OK;
}
