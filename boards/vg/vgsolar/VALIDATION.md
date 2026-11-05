# VGSolar硬件开发基线验证记录

更新：2026-11-05。对象：STM32H743VI VGSolar核心板，硬件分支`feat/vgsolar-v1.17_shu-hardware-baseline`。本分支已注册独立PX4板型和`vg_vgsolar_default`/`vg_vgsolar_bootloader`目标；未声称PX4官方收录或整车验收。

## 实板观察

| 项目 | 结果与边界 |
|---|---|
| 启动与USB/QGC | 本板启动、MAVLink通信、控制台和参数读取正常 |
| IMU/气压计/罗盘 | 发布数据，所贴状态为OK；QMC5883P为READ、CNTL1=0xc5、CNTL2=0x08，错误计数0。动态航向稳定性尚未验收 |
| 参数保持修复 | 原先startup_end参数正确，连接后自定义默认值归零；V2前台C++等待使nsh_main和等待任务存活，连接后不再归零。源码支持初始任务退出及全局对象生命周期的解释，未直接跟踪析构调用 |
| Copter | 按用户执行的三轮完全断电上电流程，SYS_AUTOSTART=4001、CA_ROTOR_COUNT=4、CA_ROTOR0_PX/PY=1；所贴控制分配器有4个执行器和非零矩阵 |
| Rover | 切换后冷启动回读SYS_AUTOSTART=50000、CA_AIRFRAME=6、CA_R_REV=3；rover_differential运行，mc_att_control不运行 |
| PWM功能分配 | 板上保存102/101/301/302/303，pwm_out回读一致，三组50 Hz。未解锁，输出值均0；未做波形和电机测试 |
| SD日志 | 日志文件成功打开；未完成长期日志完整性与掉电耐久验证 |

上述核心板结果来自用户提供的控制台日志/截图和直接串口参数回读。冷启动轮次按用户执行的测试流程记录；不是自动化电源测量。已测试V2镜像含临时诊断日志；正式源码移除临时诊断，保留相同等待机制，正式镜像仍应复核一次冷启动。

## 尚未关闭的问题

- 启动日志有`param 65535 invalid`，须定位调用方和缺失参数，未宣称修复。
- `Crash dumps present on SD`：须保留、导出并分析崩溃记录，核对时间/版本后再处理；不能认定都是旧故障。
- `/fs/mtd_caldata`不存在时启动输出BSON检查错误；需核对工厂校准分区的启动检查。参数主文件实际为`/fs/mtd_params`，已能读取/保存。
- USB单板测试出现Emergency battery level；电压/电流ADC标定、电池连接与供电测试尚未完成。不得仅放宽阈值消除告警。
- 曾有heading estimate not stable、roll attitude failure和strong magnetic interference；安装方向、校准、动态航向及同次日志仍需验收。
- 所贴logger栈水位84.1%，需在后续日志负载及长期运行中观察。

## 后续验收

1. 正式提交对应镜像的冷启动、参数保持、控制台和机型切换复核。
2. 示波器验证五路引脚/频率/脉宽和停止测试、上锁、失联、重启状态；再验证实物电调中位、方向、停止行为。
3. 连接GPS、遥控、TELEM1/2外设，分别做双向通信、定位/通道及断线测试。
4. 车辆轮距、速度、转向增益及闭环调试；Copter需独立完成四电机、传感器、控制响应和飞行前验收。
5. 本基线未集成APM NCU自定义协议、编码器轮速闭环、压力/红外业务和清洁自动流程；这些属于后续二次开发。泵阀保持关闭。

## 软件检查

按HARDWARE_SETUP.md构建，运行test_boot_led.py、test_startup.py和QMC5883P故障注入测试。启动回归使用Bash模块桩，等待函数测试使用主机睡眠桩；均不能替代真实NuttX调度/任务退出或电气测试。合并HEX生成脚本核对向量、板号、模块和objcopy往返一致性。
