# VGSolar mode

本模块在 VGSolar Differential Rover 板型注册名为 **VGSolar** 的 Commander 外部模式。编号动态分配，不使用 APM 的固定模式 17。`VGS_ENABLE=0` 为默认；`VGS_CLN_EN=0` 禁止清洁机构输出。

## 数据路径

- `vgsolar_link`：USART3 `/dev/ttyS2`、115200/8N1，解析 NCU 协议并发布 `vgsolar_command`，收集状态／ACK／参数反馈。
- `VgsolarMode.cpp`：50 Hz 模式调度、注册／解锁、命令优先级、看门狗、导航和机构组合动作。
- `VgsolarModeLogic.hpp`：状态切换、滚刷参数、坐标／路径引导、定向净转角。
- `VgsolarRobotLogic.hpp`：吸盘缓变、IR 边沿、建压、保持／冻结观察、放气和抬升。
- `rover_differential`：原生速度／航向／角速度控制，最终输出由安全门控撤销。`VGS_WHL_CTRL=1` 可选双轮速度直接反馈，不创建 EKF 导航位置。
- `vgsolar_inputs`、`vgsolar_encoders`、`dyp_a02`、`vgsolar_safety`：输入与锁存安全请求。`vgsolar_esc` 仅作 Modbus 只读遥测。

本模式拥有清洁 `actuator_servos`：Servo 1 升降、Servo 2 前刷、Servo 3 后刷；PE5 低有效密封阀，PE6 高有效泵。PWM 功能／端点和物理极性需要单独验收，五路 PWM 默认功能为 0。

## 行为与边界

非零速度命令 500 ms 未更新停车；200 ms 以前的积压指令拒绝；进模式和解锁边沿丢弃旧运动。命令批次按系统、转弯、导航、速度优先处理。TURN/SYSTEM ACK 仅表示收到，不能代替执行结果。

急停在输出许可有效时推进放气／抬升，禁止履带、刷和泵。上锁／kill／退出模式后不越过 PX4 输出许可，保留待释放状态；获得许可后先完成释放才允许行驶。掉压按阈值+回差告警，恢复清告警；建立负压超时仍为故障。传感器有效性不等于机构实测到位。

GPS/NED/Body 点到点导航使用投影、直线前视和到点减速；不宣称与 APM wp_nav 全部动态行为等价。项目电压百分比用于 NCU 和关刷判断，标准 `battery_status` 保留 PX4 原生算法。

## 阅读与验证

- [软件对应与参数说明](../../../docs/vgsolar/software_parity.md)
- [核心板启动验证与接下来步骤](../../../docs/vgsolar/hardware_bringup.md)
- [统一工具和测试入口](../../../Tools/vgsolar/README.md)
- [RK3588客户端](../../../Tools/vgsolar/README_ncu_client.md)

`test/` 的纯逻辑测试和生产模式主机回归由统一入口执行。`test/host/` 仅替代调度、uORB 运输、参数、GPIO 和重启，不参与固件模块构建；真实消息布局、PX4 geo/math/matrix 仍参与测试。没有把这些测试当成完整 SITL/HIL 或真实机构验收。
