#!/usr/bin/env python3
"""Exercise built ROMFS startup scripts with modeled parameters and module calls.

Usage: python3 boards/vg/vgsolar/tools/test_startup.py build/vg_vgsolar_default
Runs Bash with an NSH-style `set` shim, not NuttX or actual hardware. Tests
vehicle selection, serial routing, saved parameter precedence and sensor fallback.
"""

import argparse
import base64
import json
from pathlib import Path
import re
import shlex
import subprocess
import zlib


SHIM = r'''
set -e
declare -A P SAVED
set() { printf -v "$1" '%s' "$2"; }
param() {
    local op=$1
    shift
    if [ "${1:-}" = -s ]; then shift; fi
    if [[ ! -v P[$1] ]]; then
        if [ "$op" = set-default ]; then
            echo "Unknown parameter: $1" >&2
            exit 1
        fi
        return 1
    fi
    case "$op" in
        set-default) if [[ ! -v SAVED[$1] ]]; then P[$1]=$2; fi ;;
        compare) [[ ${P[$1]} = "$2" ]] ;;
        greater) awk -v a="${P[$1]}" -v b="$2" 'BEGIN {exit !(a>b)}' ;;
        *) echo "Unexpected param command: $op" >&2; exit 1 ;;
    esac
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build_dir', type=Path)
    args = parser.parse_args()
    build = args.build_dir.resolve()
    init = build / 'etc/init.d'
    metadata = json.loads((build / 'parameters.json').read_text())['parameters']
    symbols = subprocess.check_output(
        ['arm-none-eabi-nm', '--defined-only', str(build / 'vg_vgsolar_default.elf')], text=True)
    modules = set(re.findall(r'\b([a-z][a-z0-9_]*)_main$', symbols, re.M))
    setup = SHIM
    for parameter in metadata:
        value = parameter['default']
        if isinstance(value, bool):
            value = int(value)
        setup += f'P[{parameter["name"]}]={shlex.quote(str(value))}\n'
    # Stub only commands which actually exist in the linked ELF.
    for module in sorted(modules - {'param'}):
        setup += f'{module}() {{ echo "CALL {module} $*"; }}\n'
    setup += f'R={shlex.quote(str(build) + "/")}\n'
    setup += 'VEHICLE_TYPE=none\n'
    # The generated autostart uses an absolute /etc path on the target.
    autostart = (init / 'rc.autostart').read_text().replace('. /etc/', '. ${R}etc/')

    def boot(saved):
        script = setup
        for key, value in saved.items():
            script += f'P[{key}]={value}\nSAVED[{key}]=1\n'
        script += f'. {shlex.quote(str(init / "rc.board_defaults"))}\n'
        script += autostart + '\n'
        for name in ['rc.vehicle_setup', 'rc.serial']:
            script += f'. {shlex.quote(str(init / name))}\n'
        script += 'echo "RC_ARGS $RC_INPUT_ARGS"\n'
        script += 'for key in "${!P[@]}"; do echo "PARAM $key ${P[$key]}"; done\n'
        output = subprocess.check_output(['bash', '-c', script], text=True)
        assert 'Conflicting config' not in output, output
        calls = [line[5:] for line in output.splitlines() if line.startswith('CALL ')]
        params = dict(line[6:].split(' ', 1) for line in output.splitlines() if line.startswith('PARAM '))
        assert 'RC_ARGS -d /dev/ttyS6' in output, output
        assert any(call.startswith('gps start -d /dev/ttyS5 ') for call in calls), calls
        assert any(call.startswith('mavlink start -d /dev/ttyS4 ') and ' -Z' in call for call in calls), calls
        return calls, params

    calls, params = boot({})
    assert 'rover_differential start' in calls and 'land_detector start rover' in calls, calls
    assert not any(call.startswith('mc_') for call in calls), calls
    assert params['SYS_AUTOSTART'] == '50000' and params['SENS_BOARD_ROT'] == '6'
    assert not any('/dev/ttyS2' in call for call in calls), calls
    for channel in range(1, 6):
        for field in ['FUNC', 'DIS', 'FAIL']:
            assert params[f'PWM_MAIN_{field}{channel}'] == '0'
    assert all(params[f'PWM_MAIN_TIM{i}'] == '50' for i in range(3))
    print('PASS: fresh Rover boot, serial routes, disabled outputs and optional TELEM2')

    saved = {'SYS_AUTOSTART': 4001, 'SENS_BOARD_ROT': 0, 'PWM_MAIN_FUNC1': 101,
             'PWM_MAIN_TIM0': 400, 'MAV_1_CONFIG': 102, 'MAV_1_FLOW_CTRL': 0}
    calls, params = boot(saved)
    for module in ['control_allocator', 'mc_rate_control', 'mc_att_control',
                   'mc_hover_thrust_estimator', 'flight_mode_manager', 'mc_pos_control']:
        assert f'{module} start' in calls, calls
    assert 'land_detector start multicopter' in calls and 'rover_differential start' not in calls
    assert 'mc_autotune_attitude_control start' not in calls
    assert any(call.startswith('mavlink start -d /dev/ttyS2 ') for call in calls), calls
    assert all(params[key] == str(value) for key, value in saved.items()), params
    calls, _ = boot({'SYS_AUTOSTART': 4001, 'MC_AT_EN': 1})
    assert 'mc_autotune_attitude_control start' in calls
    print('PASS: Copter boot, optional autotune/TELEM2 and saved parameter precedence')

    # Execute the board sensor script, including failure paths and diagnostics.
    sensors = (init / 'rc.board_sensors').read_text()
    for primary_ok in [True, False]:
        script = 'set -e\n'
        script += 'board_adc() { echo "ADC $*"; }; icm42688p() { echo "IMU $*"; };\n'
        script += f'bmp388() {{ echo "BARO $*"; {"return 0" if primary_ok else "[[ $* == *0x77* ]]"}; }}\n'
        script += f'qmc5883l() {{ echo "MAG_L $*"; return {0 if primary_ok else 1}; }}\n'
        script += 'qmc5883p() { echo "MAG_P $*"; };\n' + sensors
        output = subprocess.check_output(['bash', '-c', script], text=True)
        assert ('MAG_P' in output) != primary_ok, output
        assert ('0x77' in output) != primary_ok, output
        assert 'IMU -s -b 1 -f 8000 -R 2 start' in output, output
    script = '\n'.join(f'{name}() {{ return 1; }}' for name in
                       ['board_adc', 'icm42688p', 'bmp388', 'qmc5883l', 'qmc5883p']) + '\n' + sensors
    output = subprocess.check_output(['bash', '-c', script], text=True)
    assert output.count('ERROR [vgsolar]') == 4, output
    print('PASS: sensor address/model fallback and explicit startup error reporting')

    package = json.loads((build / 'vg_vgsolar_default.px4').read_text())
    image = zlib.decompress(base64.b64decode(package['image']))
    assert image == (build / 'vg_vgsolar_default.bin').read_bytes()
    assert package['board_id'] == 0 and len(image) <= 0x1C0000
    assert 'pwm_out' in modules and 'actuator_test' in modules
    assert not modules.intersection({'px4io', 'dshot', 'tone_alarm', 'heater'})
    print('PASS: firmware package, board ID, flash limit and linked output drivers')


if __name__ == '__main__':
    main()
