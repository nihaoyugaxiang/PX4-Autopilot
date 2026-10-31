# 第一步：两路 DYP-A02 输入

## 实现与边界

使用当前APM源码的自动输出协议：9600波特、8N1，4字节`FF high low sum`，距离毫米、大端、前三字节低8位累加和。没有发送指令，不支持触发式或其他协议版本。0.03～4.5m沿用APM板级量程，尚未作为实物型号规格重新认证。

两路在一个非阻塞工作项中读取，各有独立解析器、样本状态、设备ID和distance_sensor实例。每轮每路最多读取128字节，10ms调度；某一路无数据或噪声不会等待另一条串口。只读输入，不写电机、阀、泵或舵机。

- 支持分包、连续帧、错校验后的重同步；半帧间隔超过50ms重置解析状态。
- 有校验正确的新帧才刷新样本时间。0、低于30mm、高于4500mm的完整帧立即撤销有效值。
- 超过500ms未收到校验正确帧，发布current_distance=0且signal_quality=0；等待数据时也发布无效状态。0是低于量程的失效占位值，不能当成实际距离或安全判据。有效帧signal_quality=-1表示未知质量，不能解释成精度百分比。
- 停止时发布MODE_DISABLED/0/quality0，不保留有效旧距离；恢复有效帧后自动恢复发布。使用有限值避免当前MAVLink流将NaN直接转换为无符号厘米值。
- 500ms是此次输入新鲜度阈值，不是整车停车保证。边缘检测、15cm阈值、100ms消抖、ESTOP锁存和输出联锁尚未实现；驱动对150与160mm均视为量程内有效距离。

## 接口对照

| 角色 | 硬件 | PX4设备 | MCU接收脚 | orientation |
|---|---|---|---|---|
| 左外侧 | UART8 | /dev/ttyS7 | PE0 | 7，沿用APM左侧标签 |
| 右外侧 | USART2 | /dev/ttyS1 | PA3 | 1，沿用APM右侧标签 |

MCU脚名不是连接器针序。按已有IO图核对供电、GND、传感器TX到MCU RX及电平兼容，不依据照片猜接线。orientation沿用APM配置，实际装机朝向待实板核查。两口须专用于超声波，不能同时给MAVLink、GPS或其他驱动使用。USART3控制台保持不动。

## 编译与测试

```sh
cd /home/yu/PX4-Autopilot
source /home/yu/.venvs/px4-117/bin/activate
g++ -std=c++14 -Wall -Wextra -Werror -fsanitize=address,undefined -g src/drivers/distance_sensor/dyp_a02/test/protocol_test.cpp -o /tmp/vgsolar_dyp_protocol_test
/tmp/vgsolar_dyp_protocol_test
make -j4 vg_vgsolar_default
```

测试覆盖字面黄金帧1953mm、分包、连续包、错校验、重叠帧头、半帧超时、全部65536种负载、双通道隔离、量程边界、失效撤销、500ms新鲜度边界和恢复。它不模拟实物波束、接线或硬件UART。

## 实板第一轮：仅手动启动

先导出当前参数并保留B3回退镜像。使用本步生成的“仅应用”固件，按已有应用更新方式保留Bootloader；动力保持断开。无需调整IMU/罗盘校准。此版本VGS_RNG_EN默认0，刷入后不会自动占用这两条串口。

在NSH依次执行：

```sh
ver all
param show VGS_RNG_EN
dyp_a02 start -l /dev/ttyS7 -r /dev/ttyS1
dyp_a02 status
listener distance_sensor -i 0 -n 5
listener distance_sensor -i 1 -n 5
```

按status的设备ID和orientation识别左右，不能只靠uORB实例序号猜左右；若另有测距驱动占用实例，查询其实际实例。输入有效时current_distance单位m，signal_quality=-1；未接入、越量程或超时时距离0、quality0。status里的latest_mm=4294967295也表示当前无效/过期，frames只是校验通过帧数，不保证物理测量可信。

先逐侧移动标准平面到10、15、20、50cm，各采集至少30个样本，确认只有对应侧变化并记录型号、距离与接线。然后保持另一侧正常，停止其中一侧数据流，确认500ms阈值后下一个调度周期撤销旧值、另一侧继续更新；恢复数据后自动恢复。输入错误校验不得使坏距离生效。未验证电气连接前不带电插拔，断流测试优先使用合适的串口模拟器或可控传感器供电。

```sh
dyp_a02 status
logger status
dyp_a02 stop
```

确认SD的Full File Logging Running后再正式记录；logger未运行时使用`logger start -e -t`，仅Not logging时用`logger on`。本板默认distance_sensor约1Hz入日志，适合查读数/左右/长期失效，不能据此精确量测500ms或100ms级时序；软件测试与后续专门采样补充时序证据。

台架确认两侧正确后，如需开机启动才执行：

```sh
param set VGS_RNG_EN 1
param save
reboot
```

此时板级脚本按上表启动两路。设置为0并重启可禁用自动启动；临时停止使用`dyp_a02 stop`。它不更改任何校准参数。

## 当前验收状态

协议/新鲜度主机测试：有可复算测试；固件构建和哈希见Windows交付记录。真实传感器收数、实际电平/波特率、左右映射、距离精度、拔线恢复、重启自动启动和ULog均待用户实测。第一步实板通过后再进行压力/红外输入，不能把编译成功当作传感器验收完成。

源码依据：APM `AP_RangeFinder_DYP_A02.cpp/.h`、VGSolar `hwdef.dat/defaults.parm`；PX4 `hardware-map.json`、NuttX串口配置、`PX4Rangefinder`、`DistanceSensor.msg`。当前本地新增devtype=0xED，仅保证当前树中不重复，不代表取得上游全局编号。
