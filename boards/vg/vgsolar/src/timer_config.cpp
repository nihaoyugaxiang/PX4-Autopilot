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

#include <px4_arch/io_timer_hw_description.h>
/* 五路PWM资源表，交由PX4原生pwm_out驱动使用，通道序号按此数组从1计数。
 * TIM1的两路共用频率，TIM4的两路也共用频率；TIM3单独控制第五路。
 * PA8/PE11/PA0/PA1仍为编码器输入，不能因AP原有PWM编号将其误用为输出。
 * PC8使用TIM3；TIM8保留给HRT，不能用于前刷PWM。
 */
constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
    // PWM_MAIN_TIM0、TIM1、TIM2对应数组索引，不是硬件TIM0/TIM1/TIM2。
    initIOTimer(Timer::Timer1, DMA{DMA::Index1}),
    initIOTimer(Timer::Timer4, DMA{DMA::Index1}),
    initIOTimer(Timer::Timer3),
};
constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
    // PWM1：PE13，原AP PWM3，左履带候选输出。
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),
    // PWM2：PE14，原AP PWM4，右履带候选输出。
    initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {GPIO::PortE, GPIO::Pin14}),
    // PWM3：PD14，原AP PWM5，升降候选输出；与PWM4共享TIM4频率。
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
    // PWM4：PD15，原AP PWM6，后刷候选输出。
    initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),
    // PWM5：PC8，原AP PWM9，前刷候选输出；不占用TIM8。
    initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {GPIO::PortC, GPIO::Pin8}),
};
constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
    initIOTimerChannelMapping(io_timers, timer_io_channels);
