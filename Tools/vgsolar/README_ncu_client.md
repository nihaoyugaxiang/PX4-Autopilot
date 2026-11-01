# 第20篇：RK3588 / PC 通信客户端与离线联调

日期：2026-11-01。上一版飞控固件仍为“软件差异补齐”版本，本轮只新增主机端工具和测试，没有改变固件运行源码或刷写设备。

## 本轮实现

`Tools/vgsolar/ncu_client.py` 是新的通用命令行客户端，可在 RK3588 Linux 和 PC 使用；已移除历史只读阶段的 `test_ncu_link.py` 及其重复测试，统一使用本客户端。

- 编码：速度、组合转弯、四个滚刷运行参数、急停／解除／上锁重启、GPS/NED/Body 导航及取消。
- 解码：34字节状态、ACK、参数反馈、8字节导航反馈。保留有符号速度／姿态、0xFFFF无效测距和协议单位。
- 串口：115200/8N1；默认不发令，打印帧字节。`monitor` 只接收；命令加 `--send` 才开串口并发送。
- 速度：每100ms重复发令，持续 `--seconds` 秒；正常结束或Ctrl+C退出时补发零速，接收异常也会尝试发送。串口断开时停车帧可能无法送达，飞控自己的500ms看门狗是独立兜底。
- 转弯／导航／系统／参数：一次发令后在指定窗口接收反馈，不自动重发，避免重复执行。窗口结束不代表动作完成，TURN/SYSTEM的成功ACK仅表示收到。
- 此工具不负责PX4模式选择或解锁，也不包含视觉、任务规划、清洁路线生成等RK3588应用业务。需要在QGC等已有入口选择动态VGSolar模式；不能使用APM固定17号。

## 文件在哪里

本目录含客户端、测试和依赖说明。可将 `ncu_client.py` 与 `requirements-rk3588.txt` 复制到RK3588工作目录使用；不依赖固件仓库。

## 先运行不接设备的检查

```sh
python3 ncu_client.py speed 30 0
python3 ncu_client.py turn 9000 3000
python3 ncu_client.py param 0x0104 20
python3 ncu_client.py nav body 100 0 --velocity 10 --radius 10
python3 ncu_client.py estop
```

以上只打印字节。速度单位cm/s，角度与角速度单位0.01度或0.01度/s；例中9000为90度。Body/NED的x/y为cm；GPS的x是经度×1e7、y是纬度×1e7。未给 `--arrival` 使用0xFFFF不要求到达对航向。

客户端速度 `yaw` 在默认角速度模式中正值表示左转；加 `--heading` 后是航向厘度，编码为有符号int16；180度以上可用等效负角度（如270度用-9000）。导航到达航向支持0..36000。所有接口遵守现有协议，不扩展私有单位。

## RK3588实机开始步骤

1. 安装 Python3 和 pyserial：`python3 -m pip install -r requirements-rk3588.txt`。此安装需RK3588上自行联网执行，本轮未远程安装。
2. 确认板上真正连接NCU的串口设备名，把下面 `/dev/ttySx` 换成实际设备。RK3588端设备名不等于飞控端 `/dev/ttyS2`。确认串口电平、共地及TX/RX接线与两端硬件相符。
3. 先隔离履带／泵／刷动力；飞控侧确认 `vgsolar_link status` 和 `vgsolar_mode status`，用只读监听验证状态帧。

```sh
python3 ncu_client.py --port /dev/ttySx --seconds 10 monitor
```

4. 检查 battery_pct、fault_code、gps_status、四路测距字段；左右DYP未接时0xFFFF是无效，不是0cm。需要修改电量来源时按19篇参数说明处理。
5. 在隔离动力状态下先验证只读参数和急停反馈：

```sh
python3 ncu_client.py --port /dev/ttySx --send param 0x0104
python3 ncu_client.py --port /dev/ttySx --send estop
```

6. 再按19篇完成输入／机构／输出许可和停止行为的台架验收。以下只是后续已具备条件时的命令格式，本轮未执行：

```sh
python3 ncu_client.py --port /dev/ttySx --send clear
python3 ncu_client.py --port /dev/ttySx --send --seconds 1 speed 10 0
python3 ncu_client.py --port /dev/ttySx --send cancel
```

Windows示例把端口换成COM号，并使用 `python`；全局参数必须放在命令名之前。`--seconds`只控制本工具接收窗口／持续速度时间，不是导航或转弯任务超时。

## 本轮实际验证

- `python3 Tools/vgsolar/ncu_client_test.py`：9/9通过。含独立十六进制golden向量、四种反馈、边界值、逐字节半包、噪声／坏校验／超时重同步、部分写入、异常退出补零速和默认命令不打开串口。
- Linux PTY测试：客户端经真实操作系统虚拟终端收发，持续速度至少两帧，反馈分包后解析，最后一帧为零速。
- PTY另一端是本测试的应答模拟器，不是飞控固件；没有据此声称 `vgsolar_link → uORB → VgsolarMode → UART` 完整链路已联调。
- 飞控现有生产模式主机测试与其他回归：12/12 ASan/UBSan通过。真实调度、Commander、UART/PWM和物理机构仍未验证；这些测试与客户端测试分别执行。
- 当前工具可用于后续RK3588实机联调，但RK3588已有应用源码和具体串口尚未核对。

日志见 `ncu_client_tests.log`、`firmware_offline_regression.log`；源码／日志校验见 `SHA256SUMS.txt`；固件及用户设置未改变的校验见 `verification.json`。

## 下一步的验收记录

实际连通后记录：RK3588串口名、飞控固件SHA、VGSolar动态模式号、双方状态输出、10Hz状态持续性、参数读应答、急停／解除状态、断开速度帧后的停车时间、30分钟通信和10次重连结果。没有这些现场证据前，不把客户端离线测试标为实机通信通过。
