#!/bin/sh
# 仅供动力断开的示波器/逻辑分析仪台架。此脚本不会解锁，也不会自动发测试脉冲。
# 先停止原生PWM再配置功能，避免运行中的驱动自动修改通道参数。
if pwm_out status
then
    if ! pwm_out stop
    then
        echo "PWM stop failed; setup aborted"
        exit 1
    fi
fi

# 清空旧功能，避免沿用此前电机/舵机映射。
param set PWM_MAIN_FUNC1 0
param set PWM_MAIN_FUNC2 0
param set PWM_MAIN_FUNC3 0
param set PWM_MAIN_FUNC4 0
param set PWM_MAIN_FUNC5 0
param set PWM_MAIN_TIM0 50
param set PWM_MAIN_TIM1 50
param set PWM_MAIN_TIM2 50
param set PWM_MAIN_REV 0

# 301～305为原生通用外设功能，供actuator_test -f指定。
# 避免套用电机的推力曲线或舵机的trim，确保波形测试直接对应归一化值。
param set PWM_MAIN_DIS1 0
param set PWM_MAIN_FAIL1 0
param set PWM_MAIN_MIN1 1000
param set PWM_MAIN_MAX1 2000
param set PWM_MAIN_CENT1 1500
param set PWM_MAIN_FUNC1 301
param set PWM_MAIN_DIS2 0
param set PWM_MAIN_FAIL2 0
param set PWM_MAIN_MIN2 1000
param set PWM_MAIN_MAX2 2000
param set PWM_MAIN_CENT2 1500
param set PWM_MAIN_FUNC2 302
param set PWM_MAIN_DIS3 0
param set PWM_MAIN_FAIL3 0
param set PWM_MAIN_MIN3 1000
param set PWM_MAIN_MAX3 2000
param set PWM_MAIN_CENT3 1500
param set PWM_MAIN_FUNC3 303
param set PWM_MAIN_DIS4 0
param set PWM_MAIN_FAIL4 0
param set PWM_MAIN_MIN4 1000
param set PWM_MAIN_MAX4 2000
param set PWM_MAIN_CENT4 1500
param set PWM_MAIN_FUNC4 304
param set PWM_MAIN_DIS5 0
param set PWM_MAIN_FAIL5 0
param set PWM_MAIN_MIN5 1000
param set PWM_MAIN_MAX5 2000
param set PWM_MAIN_CENT5 1500
param set PWM_MAIN_FUNC5 305

# 两类操作分开：本脚本只准备通道，测试脉冲必须另行手动限时发送。
pwm_out start
pwm_out status
echo "Bench config ready: 50 Hz, functions 301..305; actuator power must remain disconnected"
