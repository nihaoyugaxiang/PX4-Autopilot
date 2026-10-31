# VGSolar PX4 Rover B1：IMU 方向修正验证固件

> 2026-10-31开发更新：当前迁移分支增加两路DYP-A02输入和原生五路PWM台架测试，PWM输出功能默认禁用，DShot/PX4IO仍未启用。下面B0/B1内容保留为历史说明；最新PWM配置、编号及过程见`docs/vgsolar/STEP02_PWM_BENCH.md`。实际波形及整车输出尚未验收，校准与航向暂缓。

该板型已从第一阶段“注册骨架”推进到可编译的最小板级移植。基线为 PX4 v1.17.0，目标 `vg_vgsolar_default`。B0 的目的是验证 CPU/NuttX 启动、差速 Rover 运行、USB/MAVLink、传感器和存储；不用于行驶或清洁作业。

## 新板首次烧写（最新说明）

首次烧写前，用户确认这是一块从未成功运行程序的新板，只有 USB。新板首次使用 STM32 芯片自带 ROM DFU 和 STM32CubeProgrammer 写入“Bootloader + B0 主程序”的合并 HEX，先前只刷 `.px4` 的流程只适用于已经有兼容 Bootloader 的板。

已增加 `vg_vgsolar_bootloader` 目标，芯片明确选 STM32H743VI；USB Bootloader 与应用的本地板号均为 0。Bootloader 放在 0x08000000，应用放在 0x08020000。首次 HEX 自带两段地址，不需要手填二进制偏移。用户已通过 USB DFU 烧写 B0，并提供成功启动 PX4 Rover 的 NSH 输出；未读取或修改选项字节。

```bash
cd /home/yu/PX4-Autopilot
source /home/yu/.venvs/px4-117/bin/activate
make -j4 vg_vgsolar_bootloader
make -j4 vg_vgsolar_default
python3 boards/vg/vgsolar/tools/make_first_flash.py
```

合并文件：`/home/yu/PX4-Autopilot/build/vgsolar_first_flash/VGSolar_Rover_B0_首次烧写.hex`。生成程序同时核对两段地址、向量、Bootloader ELF 板号和主程序板号，使用 GNU objcopy 反向解析 HEX 并逐字节比较。

新板原先没有可导出的 AP 参数，不需要执行旧流程中的 AP 参数备份步骤；若以后操作已运行过 AP 的板，则仍须先备份实物参数。

## 已完成

- STM32H743VI、16 MHz，应用起始地址 0x08020000。链接限制 1792 KiB，与现有 AP 应用镜像上限一致；这不保证刷写过程保留应用区末端旧数据。
- ICM42688P：SPI1，CS PE12、DRDY PB2，8 MHz，B1 旋转值改为 2（Yaw 90°）。PX4 驱动先翻转原始 Y/Z，故不能直接照搬 AP 的 27；两者完整变换已核算等效。实机确认 STM32 面朝上时不再倒置；USB 朝车头还需设置整板安装参数 SENS_BOARD_ROT=6，该参数已经由用户在实物设置并通过前向俯仰测试，但尚未写入固件默认值。
- I2C4 内部、I2C1 外部，I2C1 SDA PB9；禁用与 USART3 冲突的 I2C2。
- BMP388 地址 0x76/0x77 回退探测；QMC5883L 0x0d / QMC5883P 0x2c 回退探测。
- FRAM SPI2/PD4，32 KiB 参数分区；TF 卡 SDMMC2。去掉未装的 V6C EEPROM 分区。
- ADC：电压 PC4、电流 PA4，同时保留 PC0/PC1 原始模拟量用于后续检查。
- 移除 V6C 电阻版本检测及不适用的供电/加热器 GPIO。MCU_K 暂沿用原 AP 的输出低初态；气阀 PE5 高为关闭，气泵 PE6 低为关闭。
- 串口按当前 AP 连接重映射。NuttX 禁止串口重新排序：USART1/2/3、UART4/5、USART6、UART7/8 分别对应 ttyS0~ttyS7。GPS=ttyS5，数传=ttyS4，遥控=ttyS6。
- 默认 SYS_AUTOSTART=50000，启动通用差速 Rover；USB 使用现有 AP 构建的 0x1209/0x5740 标识作为本地台架身份，不代表取得新的 USB 厂商分配。
- PWM、DShot、PX4IO、加热器、蜂鸣器硬件驱动及清洁业务均不编入 B0。电机、滚刷、舵机及编码器资源保持输入状态；气阀/气泵仅设置关闭电平。

## 构建

在 XTDrone2-24.04 中：

```bash
cd /home/yu/PX4-Autopilot
source /home/yu/.venvs/px4-117/bin/activate
make -j4 vg_vgsolar_default
```

ARM 编译工具链及必要库已安装。输出位于：

```text
/home/yu/PX4-Autopilot/build/vg_vgsolar_default/vg_vgsolar_default.px4
```

## 已做的离线验证

- ARM GCC 13.2.1 交叉编译、链接和 PX4 固件打包成功。
- 29 项 MCU 引脚复用逐项解析到 NuttX 真实定义，与 AP/原理图证据核对。
- ELF 符号确认 Rover、ICM42688P、BMP388、QMC5883L/P、MAVLink、CDC ACM 和 MTD 已进入镜像。
- ELF 与生成配置确认 pwm_out、dshot、px4io、heater、tone_alarm 等硬件驱动缺席。
- `.px4` 解压内容与 `.bin` 相同，映像长度、复位向量及 RAM 栈地址范围检查通过；官方 px_uploader 可以离线读取此固件包。
- 使用 board_id 0、128 KiB 启动偏移，依据当前 AP 定义及构建产物；新板首次烧写现已提供自行编译的配套 Bootloader，板号从其 ELF 数据核对为 0；随后已由用户完成 ROM DFU 烧写并启动。

**用户已验证 B0 真板启动、差速 Rover 运行、四类传感器输出、EKF 姿态有效和 SD 日志写入。** B1 实机验证倒置问题消失；搭配实物安装参数 SENS_BOARD_ROT=6 后，USB 端抬高产生正俯仰。侧向滚转、校准和稳定航向仍待验证。本版未编入执行器输出驱动，不能控制车辆行驶。详见 [实机验证记录](BRINGUP.md)。

## 已有 AP 程序的板：刷写前必须保留的数据

PX4 参数存储会使用原 AP 同一颗 FRAM，可能覆盖 AP 参数。先在 Mission Planner 中完整导出当前板子的参数，特别是 IMU/罗盘校准和项目标定；现有 defaults.parm 不含全部逐机校准，不能替代这个备份。

当前测试配置还包括 SYS_AUTOSTART=50000 和 SENS_BOARD_ROT=6（STM32 面朝上、USB 朝前）；安装参数 6 已保存在本次实物中，未固化为固件默认值。新板请按 [实机验证记录](BRINGUP.md) 设置并独立验证。

先断开电池及执行器，使用单板 USB 台架验证。用 QGroundControl 的高级设置选择“自定义固件文件”，选择本板 B0 `.px4`，不是官方 V6C 固件，也不是把文件复制进 TF 卡。出现板号或 Bootloader 不匹配时停止并保留提示，不使用强制刷写或整片擦除。

详细的刷写和验收步骤随交付包提供。参考 [QGC 固件安装](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html)。
