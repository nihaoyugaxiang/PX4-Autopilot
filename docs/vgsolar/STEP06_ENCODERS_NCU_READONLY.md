# 第06批：编码器台架验收准备与 NCU 只读链路

## 本批范围

将两项相关工作合并推进：保留第05步原始编码器采集并给出实板验收流程；新增 USART3 上的 NCU 只读通信。NCU 帧解析和状态打包采用独立纯 C++ 逻辑，PC 串口工具可用来验证拆包、粘包、校验和、帧长度及周期回传。

本固件不执行 NCU 速度、转弯、位置、急停/解除急停、参数写入或重启命令。协议长度正确的速度/参数读取只计入拒绝数并保持无响应（APM 的速度本就无 ACK，参数读取在当前阶段无可读值）；转弯、参数写入、系统控制和位置命令只回失败 ACK。NCU 状态帧中的左右轮速度、运动状态、控制模式和故障码暂时固定为 0，表示这些数据源尚未接入，不能解释为编码器测出的零速、PX4 控制状态、急停清除或无故障证明。

## 依据与协议

协议依据 APM `develop` 实际源码 `libraries/AP_CompanionComputer/AP_CompanionComputer_config.h`、`AP_CompanionComputer.cpp` 及 `docs/VGSolar/FCU_NCU通信协议.md`，而非仅依据下载目录中的 ROS2 bridge。确认的 FCU 状态数据体为 34 字节、整帧 41 字节、10 Hz；帧头 `A5 5A`，来源 `BB`，命令 `01`，校验为从来源到数据体末尾的 uint8 累加和，尾字节 `FF`。超声波左右外侧数据映射到 APM 既有字段顺序，内侧字段复制同侧值；无效距离填 `FFFF`。

接收帧最大数据体为 15 字节，单帧超时 200 ms。坏校验、错长度、错误来源和超时均不会更新状态或执行命令。协议模块的检查计数可由 `vgsolar_link status` 查看。

## 串口与控制台

NCU 使用 USART3，PB10=TX、PB11=RX，对应 `/dev/ttyS2`，115200 8N1。板级 NuttX 物理控制台从 USART3 移到空闲 USART1（PB6/PB7，对应 `/dev/ttyS0`），避免与 NCU 共用引脚。USB CDCACM/MAVLink/QGC MAVLink Shell 启动方式保持原样；若你原来通过 USB-UART 接 USART3 看启动日志，这个物理控制台会改到 USART1。`vgsolar_link` 不加入开机脚本，需人工启动。

接实板前先确认 NCU TTL 电平是 3.3 V，双方共地，并交叉连接 NCU TX→PB11、NCU RX←PB10。不可将 RS-232 电平或 5 V 推挽输出直接接入 STM32。首次只连控制板与串口测试仪，不连接电机、泵、阀或其他动力负载。

## 软件结构

1. `Protocol.hpp` 实现有界逐字节解析器、APM 34 字节状态帧和拒绝 ACK；不依赖 PX4/uORB，主机编译即可测试。
2. `VgsolarLink.cpp` 手动启动后打开 `/dev/ttyS2`，用非阻塞 I/O 读 NCU，周期订阅 uORB 状态并按 10 Hz 回传。只对有效且新鲜的数据填充姿态、定位、电量和 DYP-A02 距离；速度字段按 APM 的地速大小语义编码。
3. 电池无有效百分比时用 0；无 GPS fix 时经纬度为 0、状态为 0；测距数据过期/无效时对应字段为 `FFFF`。距离只认 DYP-A02 的设备类型、串口总线和方向，避免拿其他测距实例顶替。
4. `Tools/vgsolar/test_ncu_link.py` 可连接 USB-UART/NCU 并校验实际回传帧。`--inject` 会发分片、粘连、错误校验样本及只读探测命令；命令都不产生 PX4 动作。

## 软件验证

在 PX4 根目录运行：

```sh
g++ -std=c++17 -Wall -Wextra -Werror src/modules/vgsolar_link/test/protocol_test.cpp -o /tmp/vgsolar_link_protocol_test
/tmp/vgsolar_link_protocol_test
python3 -m py_compile Tools/vgsolar/test_ncu_link.py
source /home/yu/.venvs/px4-117/bin/activate
make -j4 vg_vgsolar_default
```

协议测试逐字节检查 34 字节数据体/41 字节状态帧、累加校验、小缓冲区拒绝、分片/粘连帧、错误来源/长度/校验后的恢复、200 ms 超时及失败 ACK。Python 单测检查串口帧生成和状态字段解码。测试不代替板上 UART 电气/实际数据验收。

## 首轮实板命令

确认固件版本和 USART3 控制台释放后，先检查 NCU 不发命令时状态回传：

```sh
ver all
vgsolar_link start
vgsolar_link status
```

准备好 3.3 V USB-UART 适配器且已确认 PB10/PB11 测试点后，在 PC 运行：

```sh
python Tools/vgsolar/test_ncu_link.py COMxx --seconds 15
```

Linux 下将 `COMxx` 换成 `/dev/ttyUSB0` 等实际设备名。正常应约每秒显示 10 条状态帧，`data_len=34`、完整帧长 41、校验通过。随后做异常帧测试：

```sh
python Tools/vgsolar/test_ncu_link.py COMxx --seconds 15 --inject
```

合法的只读探测帧应观察到失败 ACK；速度命令不单独应答；错误校验只增加错误计数，不产生任何动作。记录 `vgsolar_link status` 前后计数及串口抓包。

## 编码器实板验收

断开履带动力，确认编码器输出电平不超过 STM32 3.3 V 输入额定值，并按原理图确认连接器到 MCU 引脚映射。车轮静止时启动：

```sh
vgsolar_encoders start
vgsolar_encoders status
listener vgsolar_encoder_status -n 5
```

静止 3 秒，计数应稳定；手转右轮一小段，再手转左轮，确认只改变对应侧计数；同一侧反向手转，净增量符号应相反。停止采集时运行 `vgsolar_encoders stop`，检查回调撤销。`timestamp_sample=0` 表示尚未出现有效边沿，静止时正常；`capture_active=true` 只说明采集回调已安装。

GPIO 中断计数尚未按最高边沿率验证，不应用于高速计数、轮速闭环或里程估计。需要用已知 AB 频率信号和参考计数器确认漏计范围；实板边沿方向也须与车辆前进方向做独立对照。

## 验收边界

- 软件：纯协议测试、Python 语法检查、板型完整构建通过后，才生成可刷固件。
- NCU：实际 PB10/PB11 电平、共地、收发交叉、帧频率、错误计数及断连恢复需在实板补证。
- 编码器：当前用户输出只有 `capture_active=true` 且计数为 0；尚未有手轮边沿证据，方向/映射/频率没有验收。
- 不因本批完成而开放履带闭环、NCU 运动控制、急停执行联锁或清洁机构控制。
