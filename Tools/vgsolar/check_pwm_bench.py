#!/usr/bin/env python3
"""Check real board resources and exercise NSH scripts with mock commands, not hardware."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / 'boards/vg/vgsolar'

def params(text):
    return dict(re.findall(r'^param set(?:-default)? (PWM_MAIN_\w+) (-?\d+)$', text, re.M))

table = (BOARD / 'src/timer_config.cpp').read_text()
channels = re.findall(r'initIOTimerChannel\(io_timers, \{Timer::Timer(\d+), Timer::Channel(\d+)\}, \{GPIO::Port([A-Z]), GPIO::Pin(\d+)\}\)', table)
assert channels == [('1', '3', 'E', '13'), ('1', '4', 'E', '14'), ('4', '3', 'D', '14'), ('4', '4', 'D', '15'), ('3', '3', 'C', '8')]
assert not ({p+n for _,_,p,n in channels} & {'A8', 'E11', 'A0', 'A1'})
assert all(timer != '8' for timer,_,_,_ in channels)
header = (BOARD / 'src/board_config.h').read_text()
assert re.search(r'#define HRT_TIMER 8\b', header)
assert re.search(r'#define DIRECT_PWM_OUTPUT_CHANNELS 5\b', header)
defaults = params((BOARD / 'init/rc.board_defaults').read_text())
for i in range(3): assert defaults[f'PWM_MAIN_TIM{i}'] == '50'
assert defaults['PWM_MAIN_REV'] == '0'
for i in range(1, 6):
    for key, value in [('FUNC','0'), ('DIS','0'), ('FAIL','0'), ('MIN','1000'), ('MAX','2000'), ('CENT','1500')]:
        assert defaults[f'PWM_MAIN_{key}{i}'] == value

setup = BOARD / 'extras/vgsolar_pwm_setup.nsh'
off = BOARD / 'extras/vgsolar_pwm_off.nsh'
for path in [setup, off]:
    subprocess.check_call(['bash', '-n', str(path)])
    assert 'actuator_test set' not in path.read_text()
    assert 'commander arm' not in path.read_text()

# Actual script execution on host with stand-in commands records ordering and values.
with tempfile.TemporaryDirectory(prefix='vgsolar_pwm_') as tmp:
    tmp = Path(tmp)
    log = tmp / 'calls.txt'
    for name in ['pwm_out', 'param']:
        executable = tmp / name
        executable.write_text('#!/bin/sh\necho "' + name + ' $*" >> "$PWM_CHECK_LOG"\n'
            'if [ "' + name + '" = pwm_out ] && [ "$1" = stop ] && [ "$PWM_CHECK_FAIL_STOP" = 1 ]; then exit 1; fi\nexit 0\n')
        executable.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp)+':'+os.environ['PATH'], PWM_CHECK_LOG=str(log), PWM_CHECK_FAIL_STOP='0')
    subprocess.check_call(['sh', str(setup)], env=env, stdout=subprocess.DEVNULL)
    calls = log.read_text().splitlines()
    assert calls[:2] == ['pwm_out status', 'pwm_out stop']
    assert calls[-2:] == ['pwm_out start', 'pwm_out status']
    settings = params('\n'.join(calls))
    for i in range(1, 6):
        assert settings[f'PWM_MAIN_FUNC{i}'] == str(300+i)
        assert settings[f'PWM_MAIN_DIS{i}'] == settings[f'PWM_MAIN_FAIL{i}'] == '0'
    log.write_text('')
    subprocess.check_call(['sh', str(off)], env=env, stdout=subprocess.DEVNULL)
    calls = log.read_text().splitlines()
    assert calls[:2] == ['pwm_out status', 'pwm_out stop']
    assert calls[-1] == 'param save' and 'pwm_out start' not in calls
    settings = params('\n'.join(calls))
    assert all(value == '0' for value in settings.values()) and len(settings) == 15
    for script in [setup, off]:
        log.write_text('')
        failed_env = dict(env, PWM_CHECK_FAIL_STOP='1')
        assert subprocess.call(['sh', str(script)], env=failed_env, stdout=subprocess.DEVNULL) != 0
        assert log.read_text().splitlines() == ['pwm_out status', 'pwm_out stop']
print('PASS: five pins/timer groups/encoder exclusion/HRT reservation/default-disabled/setup/off/stop-failure abort')
