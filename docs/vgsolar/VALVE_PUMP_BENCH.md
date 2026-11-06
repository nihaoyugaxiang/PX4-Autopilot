# VGSolar valve/pump electrical bench tests

## Native GPIO shell commands

The board now enables PX4's existing `CONFIG_SYSTEMCMDS_GPIO=y` command. A fresh
firmware build/flash is required if the shell previously reported `gpio: command
not found`. No GPIO driver changes are needed.

With the robot supported, disarmed, and without suction-dependent load, ensure
any preceding timed test has finished, then stop the normal pin owner:

```sh
vgsolar_mode stop
gpio write E6 0
gpio write E5 1
```

Both outputs are now commanded OFF. Execute each ON/OFF pair separately:

```sh
gpio write E5 0
gpio write E5 1
```

PE5 is the valve, active LOW: zero energizes it, one de-energizes it.

```sh
gpio write E6 1
gpio write E6 0
```

PE6 is the pump, active HIGH: one enables it, zero disables it. These native
commands have no automatic timeout or arming gate and remain in effect until
another writer changes the pin or the board restarts. Do not use `--force` for
these two outputs: they are already in the board's GPIO initialization list.
The raw `gpio read` command changes pin mode to INPUT; it should not be used to
verify an output while a valve/pump is connected. Observe actual response or
measure the output without changing its configuration. Native writes bypass the
cleaner state machine and do not move the lift PWM.

After testing, command E6 LOW and E5 HIGH again. Only then restart the normal
mode task with `vgsolar_mode start` if needed; leave automatic cleaning disabled.

These commands require firmware containing the new test entry points. They test
the GPIO outputs separately; they do not run the suction/lift state machine.
Place the robot on a stable bench/ground with no load depending on suction.
Disconnect drive/brush power. Supply the valve and pump at their rated voltage.

Keep the vehicle disarmed. Read the existing parameter values before setting
`VGS_ENABLE` and `VGS_CLN_EN` to zero. Leave both disabled throughout these tests.

```sh
param show VGS_ENABLE
param show VGS_CLN_EN
param set VGS_ENABLE 0
param set VGS_CLN_EN 0
vgsolar_mode stop
vgsolar_mode status
```

The mode must report `not running`. The test command marks its session active
under the ModuleBase command mutex, preventing a concurrent mode start or another
test from owning the pins. It releases the global mutex while waiting, so other
PX4 modules can still accept lifecycle commands.

Execute one command at a time and observe whether it returns to OFF:

```sh
vgsolar_mode test-valve
```

PE5 becomes LOW for one second, then HIGH (OFF). PE6 stays LOW (pump OFF).
Observe valve sound/operation; coil energization alone does not prove the actual
air-path direction. Keep hands clear of the lift; an RC-controlled servo may
remain powered in prearmed mode.

```sh
vgsolar_mode test-pump
```

PE6 becomes HIGH for one second, then LOW (OFF). PE5 stays HIGH (valve OFF).
This independent pump test does not request a sealed suction cycle and does not
establish vacuum performance. No PWM output or servo mapping is changed.

The command checks fresh (< =1 second old) `vehicle_status` and `actuator_armed`
messages, disarmed state, kill/lockdown/termination/calibration flags, and both
enable parameters before switching ON and every 10 ms while running. A denied
initial request does not touch the GPIOs. Any ordinary return after activation
restores pump LOW and valve HIGH. A scheduling stall/hard fault is outside this
software timeout guarantee; keep the external supply disconnect accessible.

After both tests have returned and the outputs are confirmed OFF, restore the
mode task without enabling automatic cleaning:

```sh
vgsolar_mode start
```

Record physical valve/pump response and, if measuring pressure separately, raw
PC0 voltage and calibration. Successful commands and host tests are software
evidence only. Existing IR response and servo movement do not validate the valve
or pump wiring, polarity, or pneumatic behavior.

## RC8 valve-only test session

The firmware also provides `vgsolar_mode test-rc-valve 8`. This command runs a
60-second foreground test session while the normal mode task is stopped.
During the session RC8 controls PE5 only. PE6 remains LOW (pump OFF).

RC8 was previously assigned to the lift through `PWM_MAIN_FUNC3=407`. Read and
record its current function, then disable MAIN3 during this valve-only test:

```sh
param show PWM_MAIN_FUNC3
param set PWM_MAIN_FUNC3 0
param set VGS_ENABLE 0
param set VGS_CLN_EN 0
vgsolar_mode stop
vgsolar_mode test-rc-valve 8
```

First move RC8 to its low raw-PWM position (<=1300 us), which keeps the valve OFF.
After observing LOW, switching HIGH (>=1700 us) energizes the valve (PE5 LOW).
The middle position keeps it OFF. Raw receiver PWM is used, independently of
RC_MAP_AUX1 and RC8_REV. No changes are made to PWM endpoints or reversal masks.
Keep the robot supported; disabling MAIN3 removes lift PWM and may remove servo
holding torque. This is a valve electrical test, not an automatic lift cycle.

RC lost/failsafe flags, absent channels, values outside 900..2100 us, or reception
timestamps older than 300 ms turn the valve OFF and reset the LOW-first latch.
After reception recovers, HIGH cannot re-enable it until LOW has been observed
again. A receiver that continues to send apparently valid HIGH after transmitter
loss cannot be detected from these fields; verify receiver failsafe behavior
with valve power disconnected before relying on it. Arming, kill/lockdown,
stale vehicle state, or enabling automatic control aborts the session. At 60
seconds the command returns and restores both outputs OFF. The shell is occupied
by this command during the session; LOW turns the valve off at any time.

Restore MAIN3's recorded function only after the valve test has ended, RC8 is
LOW and lift movement is clear. Restoring `407` restores the earlier direct RC8
lift control and can immediately move it. Start `vgsolar_mode` again if needed;
leave automatic cleaning disabled until its separate validation.
