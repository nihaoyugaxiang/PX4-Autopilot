# VGSolar 硬件基线与 Rover / Copter 配置

2026-11-05，基于 `fix/vgsolar-qmc5883p-bringup` 的 `1501409c6c`。
同一固件包含差速 Rover 和 Copter 控制栈，开机按 `SYS_AUTOSTART` 选择一套控制模块。
本轮完成软件配置与离线验证，尚未连接实板验证这些改动。

## 构建与升级

```bash
source /home/yu/.venvs/px4-117/bin/activate
make -j4 vg_vgsolar_default
```

输出 `build/vg_vgsolar_default/vg_vgsolar_default.px4`，用于已有配套 Bootloader 的板子通过 QGC 更新应用。
本轮离线验证使用独立目录 `build/vg_vgsolar_rover_copter`，其中同名 `.px4` 是本轮生成的固件。
刷写前从 QGC 导出参数。已有板升级不需要重刷 Bootloader；首次 DFU 流程仍见 README。
旧的 B0/B1/B2/B3 HEX 或 `build` 中其他分支的产物不包含本轮改动。

新板首次写入需要合并 HEX 时，先构建 `make -j4 vg_vgsolar_bootloader`，再运行：

```bash
python3 boards/vg/vgsolar/tools/make_first_flash.py
```

当前输出名为 `build/vgsolar_first_flash/VGSolar_Rover_Copter_首次烧写.hex`。
独立构建目录可通过 `--app-dir build/vg_vgsolar_rover_copter` 指定；脚本验证板号、向量、必要模块和 HEX 地址往返一致性。

## 开机驱动与端口

| 功能 | 配置/端口 | 启动方式 |
|---|---|---|
| ICM42688P | SPI1，CS PE12，DRDY PB2，8 MHz，驱动旋转 2 | 板级脚本自动启动 |
| BMP388 | I2C4，0x76 → 0x77 回退 | 板级脚本自动探测 |
| QMC5883L/P | I2C4，0x0d → 0x2c 回退 | 板级脚本自动探测，P 包含 B3 复位修正 |
| 电池 ADC | PC4 电压、PA4 电流；PC0/PC1 也采样 | `board_adc` 自动启动 |
| FRAM / TF | SPI2/PD4，32 KiB 参数分区；SDMMC2 | 板级初始化、标准参数/日志流程 |
| USB MAVLink | `/dev/ttyACM0` | `SYS_USB_AUTO=2`，由 USB 管理模块启动 |
| GPS1 | USART6 PC6/PC7，`/dev/ttyS5` | `GPS_1_CONFIG=201`，GPS 驱动自动启动 |
| TELEM1 | UART5 PC12/PD2，`/dev/ttyS4` | `MAV_0_CONFIG=101`；默认 57600，无 RTS/CTS |
| RC | UART7 PE8/PE7，`/dev/ttyS6` | `RC_PORT_CONFIG=300`，接收机协议默认自动扫描 |
| TELEM2 | USART3 PB10/PB11，`/dev/ttyS2` | 已取消 NuttX 串口控制台；`MAV_1_CONFIG=0`，默认关闭 |
| PWM | 下表五路 | 标准启动脚本启动 `pwm_out`；输出功能待分配 |

可选 TELEM2 使用标准 MAVLink，不是原 APM 的 NCU 自定义协议。
需要时设置 `MAV_1_CONFIG=102`、`MAV_1_MODE=2`、`MAV_1_FLOW_CTRL=0`，按外设设置 `SER_TEL2_BAUD`（默认 921600），保存并重启。
USB QGC Console 继续可用，USART3 不再输出 NSH 启动日志。
USART1 `/dev/ttyS0`、USART2 `/dev/ttyS1`、UART4 `/dev/ttyS3`、UART8 `/dev/ttyS7` 保留端口，按实际设备单独适配协议。

所有 `param set-default` 都不会覆盖已保存参数。升级后须在 QGC 回读端口配置、波特率、输出功能和安装方向；不能把源码默认值当作实物当前值。
`BAT1_V_DIV=20.65`、`BAT1_A_PER_V=85`、6S 仍是项目参考值，须用实测电压/电流标定。

## PWM 引脚和限制

| PX4 输出 | 引脚 | 定时器组参数 | 原机器人接线 |
|---|---|---|---|
| MAIN1 | PE13 | `PWM_MAIN_TIM0`，TIM1 CH3 | 左行走电机 |
| MAIN2 | PE14 | `PWM_MAIN_TIM0`，TIM1 CH4 | 右行走电机 |
| MAIN3 | PD14 | `PWM_MAIN_TIM1`，TIM4 CH3 | 升降舵机 |
| MAIN4 | PD15 | `PWM_MAIN_TIM1`，TIM4 CH4 | 后滚刷 |
| MAIN5 | PC8 | `PWM_MAIN_TIM2`，TIM3 CH3 | 前滚刷 |

三组默认 50 Hz，同组通道必须同频率；QGC 中配置输出功能、速率及电调所需的最小/中位/最大/上锁/失效值。
默认 `PWM_MAIN_FUNC1..5=0`、`PWM_MAIN_DIS1..5=0`、`PWM_MAIN_FAIL1..5=0`，不产生有效的高电平脉冲。
0 脉冲是否让电调停止，仍须按实际电调的失信号行为测试；这些值不替代设备的停止标定。
PA8、PE11、PA0、PA1 继续保留为编码器输入，不能按原 AP 的 PWM 编号分配电机。
TIM8 专用于 HRT；PC8 使用 TIM3，避免破坏系统时基。

本固件只启用原生 PWM，不选择负值的 DShot 协议配置。PX4IO 未启用，本板 USART6 是 GPS 接口，没有证据表明有 PX4IO 协处理器。
PB1 蜂鸣器使用 TIM3 CH4，与 MAIN5 共用定时器；标准 `tone_alarm` 会改变定时器频率，因此本轮不启用它。传感器加热器也没有对应硬件配置。
阀 PE5 保持高电平关闭，泵 PE6 保持低电平关闭；不驱动机器人清洁机构。
只有五路硬件输出，Copter 可按实际接线配置四旋翼，不能直接驱动六/八个独立电机。

## 选择机型

在 QGC 的 Airframe 页面选择并应用：

- 差速车：Generic Rover Differential，`SYS_AUTOSTART=50000`。
- 四旋翼：Generic Quadcopter，`SYS_AUTOSTART=4001`。

应用机型会重新配置机型相关参数，先导出参数；切换后逐项复核传感器安装方向、串口、电池、输出和失效保护。
不要只修改 `SYS_AUTOSTART` 后沿用另一机型已保存的 `MAV_TYPE`、`CA_*`、`EKF2_*` 和输出参数。
也不要把另一机型的完整参数文件直接覆盖回来。通过 QGC 的标准应用/重启流程切换。

Rover 启动 `rover_differential` 和 Rover land detector。
在 Actuators 页面将已核实接线的 MAIN1/MAIN2 分别分配 Motor 1/2（功能号 101/102）；左右顺序由当前 Rover 源码定义。
左右电机支持正反转，须检查 `CA_R_REV=3`，按实际电调设置中立、上锁和失效脉宽；原 AP 左电机反向不能盲目照搬。
`RD_WHEEL_TRACK`、`RO_MAX_THR_SPEED` 等车辆参数必须按实车配置，位置/航向控制还需要有效估计。

Copter 启动 `control_allocator`、`mc_rate_control`、`mc_att_control`、`mc_hover_thrust_estimator`、`flight_mode_manager`、`mc_pos_control` 和 multicopter land detector。
姿态自动调参模块已编入，仅在 `MC_AT_EN` 启用时启动。
按四旋翼实际接线及 QGC 电机编号分配四个 Motor 功能，设置电调支持的 PWM 速率；MAIN3/4 共享频率，原机器人的舵机/滚刷不能仍接在这些输出上。
不能仅凭控制模块 running 判断可以起飞。

## 实板检查与通过条件

先断开执行器动力；Copter 拆桨。以下命令只读取状态，可在 QGC Console 执行：

```text
ver all
param show SYS_AUTOSTART
param show SENS_BOARD_ROT
param show PWM_MAIN_*
dmesg
icm42688p status
bmp388 status
qmc5883p status
sensors status
ekf2 status
gps status
rc_input status
mavlink status
pwm_out status
logger status
```

Rover 另查 `rover_differential status`。Copter 另查 `control_allocator status`、`mc_rate_control status`、`mc_att_control status`、`mc_pos_control status`、`flight_mode_manager status`。
默认未分配输出时，PWM 定时器可能尚未初始化，不能因此断言驱动失败。

1. 按 `ver all` 确认刷入本次构建；`dmesg` 没有 ADC/IMU/BMP/罗盘启动错误、HardFault 或反复重启。
2. STM32 面朝上、USB 朝前时检查 `SENS_BOARD_ROT=6`，抬高 USB 端应为正俯仰，右侧压低应为正横滚。其他安装方向独立设置；逐机完成传感器校准。
3. QMC5883P 应进入 READ，CNTL1=0xc5、CNTL2=0x08，数据持续发布，reset/bad transfer 不持续增长。B3 修改后重做罗盘校准，检查偏移和水平旋转/倾斜时的航向稳定性，不能放宽检查阈值掩盖异常。
4. GPS 接入后确认接收数据和定位；遥控确认通道、方向、失联；TELEM1/可选 TELEM2 确认双向 MAVLink，设备断开时只代表接口可用，不能算链路验证完成。
5. 示波器逐路检查 MAIN1..5 的引脚、周期、脉宽和同组频率。用 QGC Actuators 测试功能逐路短时测试，验证停止测试、上锁、失联、重启后的实际输出和电调停止行为。
6. 保存一个明确的参数，完全断电再上电回读；检查 SD 新日志可以正常记录。
7. 完成上述检查后再按对应机型进行架空低速/拆桨电机测试、失效保护验证和控制参数调试。Copter 的电机方向、重心、振动、供电冗余与飞行验证需独立完成。

上述项目本轮均未新增实板通过记录。既有历史证据见 `BRINGUP.md`；校准、波形和飞行结果只能由实际设备测试确认。

## 离线回归

```bash
python3 boards/vg/vgsolar/tools/test_startup.py build/vg_vgsolar_default
python3 src/drivers/magnetometer/qmc5883p/test/test_driver.py
```

启动测试使用构建生成的 ROMFS、参数元数据和 ELF 命令表，在 Bash 中模拟参数及模块调用。
覆盖新板默认 Rover、已保存的 Copter 选择、可选自动调参、串口路由、TELEM2 默认关闭/按需开启、已保存安装方向和 PWM 参数优先级、传感器回退和启动失败提示，以及固件包内容一致性。
这不是 NuttX 仿真，不验证调度、硬件总线、电气波形或控制性能。

本轮结果：独立目录构建与标准 `make -j4 vg_vgsolar_default` 均通过，应用映像 1,330,552 字节，占 1,792 KiB FLASH 的 72.51%；Bootloader 构建也通过。
上述启动回归和 QMC5883P 故障注入回归均通过；生成的 QGC 输出元数据分组为 `[1,2]`、`[3,4]`、`[5]`。
合并 HEX 已生成，Bootloader/应用板号均为 0，地址分别为 `0x08000000` / `0x08020000`，经 GNU objcopy 往返逐字节一致性检查通过。
