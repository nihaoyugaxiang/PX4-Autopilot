#!/bin/sh
# 测试收尾：先停止PWM驱动，再将五路功能和上锁/失效值恢复为禁用状态。
# 不假定重启会清空测试参数；PX4可能自动保存参数修改。
if pwm_out status
then
    if ! pwm_out stop
    then
        echo "PWM stop failed; disconnect board power and inspect"
        exit 1
    fi
fi
param set PWM_MAIN_FUNC1 0
param set PWM_MAIN_FUNC2 0
param set PWM_MAIN_FUNC3 0
param set PWM_MAIN_FUNC4 0
param set PWM_MAIN_FUNC5 0
param set PWM_MAIN_DIS1 0
param set PWM_MAIN_DIS2 0
param set PWM_MAIN_DIS3 0
param set PWM_MAIN_DIS4 0
param set PWM_MAIN_DIS5 0
param set PWM_MAIN_FAIL1 0
param set PWM_MAIN_FAIL2 0
param set PWM_MAIN_FAIL3 0
param set PWM_MAIN_FAIL4 0
param set PWM_MAIN_FAIL5 0
param save
echo "PWM stopped; five functions disabled and saved"
