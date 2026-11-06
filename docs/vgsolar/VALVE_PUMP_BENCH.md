# VGSolar valve/pump electrical bench tests

## Native GPIO shell commands

The board now enables PX4's existing `CONFIG_SYSTEMCMDS_GPIO=y` command. A fresh
firmware build/flash is required if the shell previously reported `gpio: command
not found`. No GPIO driver changes are needed.

With the robot supported, disarmed, and without suction-dependent load, stop
the normal pin owner:

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
