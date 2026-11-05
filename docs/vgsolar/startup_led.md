# 第22篇：核心板启动指示灯

VGSolar核心板蓝灯PD11为低有效，应用完成基本平台初始化后，每125ms翻转一次，完整闪烁频率4Hz。`rcS`执行末尾、通知MAVLink启动完成后，执行本板`rc.board_boot_complete`，取消闪烁定时并把蓝灯常亮。

常亮只表示启动脚本到达末尾，不表示传感器全部正常、校准成功、GPS就绪、允许解锁或机器人可自动运行。启动脚本仍可能记录了非致命错误；必须继续看QGC消息及NSH状态。若启动中途卡住，完成钩子未执行，蓝灯继续闪烁。应用尚未进入平台初始化、ROM DFU和PX4引导程序阶段不由此逻辑控制。

蓝灯从Commander的解锁状态闪灯映射中移出，避免两个模块同时改同一GPIO；解锁／预检状态继续看QGC。红灯PD10保持PX4原有错误／过载逻辑，未重新定义。这里实现的是请求的启动灯效，并非移植APM所有LED状态组合。

## 实现位置

- `boards/vg/vgsolar/src/led.c`：HRT定时闪烁、原子取消与常亮。
- `boards/vg/vgsolar/src/init.c`：平台和LED驱动启动后开启闪烁。
- `boards/vg/vgsolar/src/board_config.h`：引脚与灯所有权声明。
- `src/systemcmds/led_control/led_control.cpp`：仅本板增加boot_complete、boot_complete_wait和boot_status命令。
- `ROMFS/px4fmu_common/init.d/rcS`：启动脚本末尾可选板级完成钩子；没有该钩子的其他板保持原行为。
- `boards/vg/vgsolar/init/rc.board_boot_complete`：本板完成标记。

```text
led_control boot_status
```

只读状态应显示startup complete / steady。`led_control boot_complete`用于启动脚本，不要手动提前执行以伪造启动完成。

## 验证与实板步骤

`boards/vg/vgsolar/tools/test_boot_led.py`编译生产led.c，仅以主机GPIO/HRT头替换硬件接口，验证低有效、125ms翻转、完成后取消、重复完成、重新初始化、红灯隔离及脚本顺序。可直接使用 Python 运行该测试；没有执行实板灯效测试。

本次需更新固件才能得到灯效。QGC升级使用px4；CubeProgrammer使用交付的含引导程序HEX，不将px4直接写入Flash。换板或刷写前隔离动力，保留现有参数。新镜像未主动重置校准或PWM参数；刷后应回读确认。

刷后正常上电观察蓝灯从闪烁转常亮，连接QGC执行`led_control boot_status`；回读五路PWM的输出功能及上锁／失效值，确认符合当前台架安全配置。若不亮或一直闪，返回启动日志，不凭灯效直接判定传感器损坏。

## 无串口控制台时保持初始任务存活

2026-11-05 的实板诊断显示：四旋翼机架脚本后直至 startup_end，CA_ROTOR_COUNT=4、CA_ROTOR0_PX/PY=1；连接后连自定义默认值也回到0。源码中，无物理串口控制台的初始NSH可能在读取EOF后退出；NuttX按任务管理C++退出回调，全局参数表析构会释放仍在使用的缓冲区。该路径与诊断现象及修复前后结果吻合；析构触发及释放过程尚未直接跟踪。

首次脚本sleep循环的实板验证未保持初始NSH存活，参数仍回到0。现改为本板rc.board_boot_complete调用前台`led_control boot_complete_wait`：标记启动灯完成后在C/C++命令内部持续休眠，NSH通过waitpid等待该命令，避免从缺失控制台返回。启动时打印`VGS_STARTUP_GUARD active`及PID，便于用日志和ps独立核实。MAVLink已在此前启动，其NSH会话通过一次性初始化保护跳过启动脚本，仍可正常使用。没有修改机架参数值、保存参数或分配PWM。

刷写后直接回读机架及旋翼参数，不先手动应用机架脚本。检查多次冷启动后旋翼数量与几何参数、控制分配器、参数保存和日志；检查SD上的崩溃文件，保留并分析后再处理，不能仅清除告警。

`boot_complete_wait`仅供开机脚本调用，不要在QGC控制台手动运行；它会占用调用会话直到重启。主机测试直接编译生产等待函数，以替代sleep的主机桩确认进入休眠，但不能替代NuttX任务生命周期的实板验证。

## V2首次实板回读（2026-11-05）

用户提供的V2日志中，after_connect保持SYS_AUTOSTART current=4001/default=50000，CA_ROTOR_COUNT current/default=4，CA_ROTOR0_PX/PY current/default=1。控制分配器显示Configured actuators: 4且矩阵非零。ps显示PID 3 nsh_main存活并等待，PID 1170 led_control boot_complete_wait休眠；日志出现对应VGS_STARTUP_GUARD active。此次观测中未再出现连接后机架默认值回零，支持初始任务生命周期的修复方向。

该日志是第一次成功样本。随后按用户执行的冷启动流程，Copter连续三轮回读为4001/4/1/1，Rover一轮回读为50000/6/3，且rover_differential运行、mc_att_control停止。长期运行和整机验收仍待完成。仍有param 65535 invalid、Crash dumps present on SD和Emergency battery level告警；logger栈水位84.1%也需在后续日志负载测试中观察。原始输出存于Windows交付目录VGSolar_启动参数验证V2_20261105/V2首次实板验证日志.txt。本轮未执行动力或飞行测试。
