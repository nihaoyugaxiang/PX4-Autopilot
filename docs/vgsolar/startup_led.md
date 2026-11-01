# 第22篇：核心板启动指示灯

VGSolar核心板蓝灯PD11为低有效，应用完成基本平台初始化后，每125ms翻转一次，完整闪烁频率4Hz。`rcS`执行末尾、通知MAVLink启动完成后，执行本板`rc.board_boot_complete`，取消闪烁定时并把蓝灯常亮。

常亮只表示启动脚本到达末尾，不表示传感器全部正常、校准成功、GPS就绪、允许解锁或机器人可自动运行。启动脚本仍可能记录了非致命错误；必须继续看QGC消息及NSH状态。若启动中途卡住，完成钩子未执行，蓝灯继续闪烁。应用尚未进入平台初始化、ROM DFU和PX4引导程序阶段不由此逻辑控制。

蓝灯从Commander的解锁状态闪灯映射中移出，避免两个模块同时改同一GPIO；解锁／预检状态继续看QGC。红灯PD10保持PX4原有错误／过载逻辑，未重新定义。这里实现的是请求的启动灯效，并非移植APM所有LED状态组合。

## 实现位置

- `boards/vg/vgsolar/src/led.c`：HRT定时闪烁、原子取消与常亮。
- `boards/vg/vgsolar/src/init.c`：平台和LED驱动启动后开启闪烁。
- `boards/vg/vgsolar/src/board_config.h`：引脚与灯所有权声明。
- `src/systemcmds/led_control/led_control.cpp`：仅本板增加boot_complete和boot_status命令。
- `ROMFS/px4fmu_common/init.d/rcS`：启动脚本末尾可选板级完成钩子；没有该钩子的其他板保持原行为。
- `boards/vg/vgsolar/init/rc.board_boot_complete`：本板完成标记。

```text
led_control boot_status
```

只读状态应显示startup complete / steady。`led_control boot_complete`用于启动脚本，不要手动提前执行以伪造启动完成。

## 验证与实板步骤

`check_boot_led.py`编译生产led.c，仅以主机GPIO/HRT头替换硬件接口，验证低有效、125ms翻转、完成后取消、重复完成、重新初始化、红灯隔离及脚本顺序。该项已接入`run_offline_tests.py --sanitize --all`。板型构建、完整离线回归和脚本语法检查通过；没有执行实板灯效测试。

本次需更新固件才能得到灯效。QGC升级使用px4；CubeProgrammer使用交付的含引导程序HEX，不将px4直接写入Flash。换板或刷写前隔离动力，保留现有参数。新镜像未主动重置校准或PWM参数；刷后应回读确认。

刷后正常上电观察蓝灯从闪烁转常亮，连接QGC执行`led_control boot_status`；确认VGS_ENABLE/VGS_CLN_EN和五路PWM仍按当前安全配置禁用。若不亮或一直闪，返回启动日志，不凭灯效直接判定传感器损坏。
