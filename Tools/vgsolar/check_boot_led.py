#!/usr/bin/env python3
# Run the production board LED backend with host GPIO/HRT stubs; no hardware.
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'boards/vg/vgsolar/src/led.c').read_text()
# Replace only the board pin header; all LED/HRT state transitions stay unchanged.
source = source.replace('#include "board_config.h"', '#include "test_board_config.h"')
with tempfile.TemporaryDirectory(prefix='vgsolar-led-') as folder:
    folder = Path(folder)
    headers = {
        'px4_platform_common/px4_config.h': '#include <stddef.h>\n#define __EXPORT\n#define __BEGIN_DECLS\n#define __END_DECLS\n',
        'chip.h': '', 'nuttx/config.h': '', 'nuttx/board.h': '', 'arch/board/board.h': '',
        'test_board_config.h': '#define GPIO_nLED_BLUE 11\n#define GPIO_nLED_RED 10\n',
        'stm32_gpio.h': '''#include <stdbool.h>
#include <stdint.h>
static bool levels[16];
static int stm32_configgpio(uint32_t pin) { (void)pin; return 0; }
static void stm32_gpiowrite(uint32_t pin, bool level) { levels[pin] = level; }
static bool stm32_gpioread(uint32_t pin) { return levels[pin]; }
''',
        'nuttx/irq.h': '''typedef unsigned irqstate_t;
static unsigned critical_depth;
static irqstate_t enter_critical_section(void) { return critical_depth++; }
static void leave_critical_section(irqstate_t flags) { critical_depth = flags; }
''',
        'drivers/drv_hrt.h': '''#include <stdint.h>
#include <stdbool.h>
struct hrt_call { void (*callback)(void *); void *arg; bool scheduled; uint64_t period; };
static void hrt_call_every(struct hrt_call *call, uint64_t delay, uint64_t period, void (*callback)(void *), void *arg) {
 (void)delay; call->callback=callback; call->arg=arg; call->scheduled=true; call->period=period;
}
static void hrt_cancel(struct hrt_call *call) { call->scheduled=false; }
''',
    }
    for path, text in headers.items():
        target = folder / path; target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text('#pragma once\n' + text)
    (folder / 'led_backend.c').write_text(source)
    (folder / 'check.c').write_text('''#include <assert.h>
#include "led_backend.c"
int main(void) {
 led_init();
 levels[10] = true;
 vgsolar_boot_led_start();
 assert(!vgsolar_boot_led_completed() && !levels[11]);
 assert(boot_led_call.scheduled && boot_led_call.period == 125000);
 boot_led_call.callback(boot_led_call.arg); assert(levels[11]);
 boot_led_call.callback(boot_led_call.arg); assert(!levels[11]);
 assert(levels[10]);
 vgsolar_boot_led_complete();
 assert(vgsolar_boot_led_completed() && !boot_led_call.scheduled && !levels[11]);
 assert(critical_depth == 0 && levels[10]);
 vgsolar_boot_led_complete(); assert(!levels[11] && critical_depth == 0);
 vgsolar_boot_led_start(); assert(!vgsolar_boot_led_completed() && boot_led_call.scheduled);
 return 0;
}
''')
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                    '-I'+str(folder),str(folder/'check.c'),'-o',str(folder/'check')],check=True)
    subprocess.run([str(folder/'check')],check=True)
assert 'rc.board_boot_complete' in (ROOT / 'ROMFS/CMakeLists.txt').read_text()
script = (ROOT / 'ROMFS/px4fmu_common/init.d/rcS').read_text()
assert script.index('set BOARD_RC_BOOT_COMPLETE ${R}') < script.index('unset R\n')
assert script.index('mavlink boot_complete') < script.index('. $BOARD_RC_BOOT_COMPLETE')
assert 'BOARD_ARMED_STATE_LED LED_BLUE' not in (ROOT/'boards/vg/vgsolar/src/board_config.h').read_text()
assert (ROOT/'boards/vg/vgsolar/init/rc.board_boot_complete').read_text().strip().endswith('led_control boot_complete')
print('PASS: production LED GPIO polarity, 4Hz blink, cancellation, steady completion, idempotence, red isolation and startup ordering')
