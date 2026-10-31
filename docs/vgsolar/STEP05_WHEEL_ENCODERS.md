# 第05步：左右轮编码器原始计数

## 目标与边界

本步在 PX4 中读取左右轮编码器 A/B 两相，输出有符号原始跳变计数、每 20 ms 计数增量、相位方向、有效跳变数和无效跳变数。模块由用户手动启动，默认不开机运行；不会驱动电机、PWM、舵机、泵阀，也不接入 EKF、底盘速度闭环或安全停车。

PX4 的标准 `wheel_encoders` 消息需要弧度制轮角和轮速。由于编码器 PPR/CPR、是否四倍频、齿轮比及轮径尚未实测，本步不发布该消息，避免把原始计数伪装成物理速度或里程。

## 引脚与采集方式

| 车轮 | A 相 | B 相 | 板级来源 |
|---|---|---|---|
| 右轮 | PA8 | PE11 | 原 APM `TIM1_CH1/CH2` |
| 左轮 | PA0 | PA1 | 原 APM `TIM5_CH1/CH2` |

PA8/PE11 属于 TIM1，而 TIM1 已分配给现有 PWM1/2，因此本步不把 TIM1 交给 NuttX 正交计数器。两侧统一采用 PX4 GPIO 双边沿中断，工作队列每 20 ms 发布诊断。四个 MCU 引脚不是 FPC/排针针号；接线前按本板原理图/PCB 确认实际测试点。

输入沿用板级 GPIO 下拉配置。接入编码器前须确认传感器地与板地共地、输出逻辑电平适合 3.3 V MCU 输入；未知或 5 V 推挽输出不可直接连接。

四路使用 EXTI0/1/8/11，彼此不重号。GPIO中断没有硬件脉冲队列，高频或长中断屏蔽可能丢边沿，完整漏掉一个周期也不一定增加错误数；错误数为0不能证明没有丢计。当前只支持慢速台架验收，最高工作边沿率必须另用已知频率的AB信号与参考计数器对照验证。静止和断线都可能没有边沿，`capture_active=true`只说明软件已安装回调，不证明传感器连接正常。

## 软件结构

1. `board_config.h` 为四个编码器引脚分别命名，并继续作为输入初始化，不改变已有 PWM 定时器和输出引脚。
2. `QuadratureDecoder.hpp` 将 `(A << 1) | B` 状态按 `00 -> 01 -> 11 -> 10 -> 00` 记为正向；反序记为负向。两位同时变化属于方向不可判定的跳变，计数拒绝并累加错误数。
3. `vgsolar_encoders` 在 `start` 时读取初始 AB 电平、为四个引脚安装双边沿回调；`stop` 时撤销回调。计数和快照使用临界区保护。
4. `vgsolar_encoder_status` 保存原始整数计数；MAVLink `DEBUG_FLOAT_ARRAY` 使用名称 `VGS_ENC` 向 QGC 实时发送 data[0]=右轮累计计数、data[1]=左轮累计计数、data[2]=左右无效跳变总数、data[3]=采集活动标志。它使用独立于压力显示的 MAVLink 消息，避免覆盖压力模块的 `DEBUG_VECT`。浮点遥测仅供短时观察，大计数以 ULog/uORB 整数为准。
5. logger 以 50 Hz 记录状态消息。传感器方向、最大边沿频率和电气噪声仍须实板验证。

## 编译与纯逻辑测试

在 PX4 仓库根目录运行：

```sh
g++ -std=c++17 -Wall -Wextra -Werror src/modules/vgsolar_encoders/test/processing_test.cpp -o /tmp/vgsolar_encoder_test
/tmp/vgsolar_encoder_test
source /home/yu/.venvs/px4-117/bin/activate
make -j4 vg_vgsolar_default
```

纯逻辑测试覆盖正反向完整相位周期、停止状态、单步反转和两位跳变拒绝；它不模拟 EXTI、电路、电机或实物编码器。

同时覆盖全部16种初始/终止相位组合、无效跳变后恢复、两轮独立反向累积各100000边沿以及重新初始化。

## 第一轮实板测试

前置条件：车架固定、左右驱动动力断开，轮子可手动安全转动；先确认 MCU 管脚到连接器测试点的对应关系和编码器逻辑电平。若编码器输出超过 3.3 V、需要外部上拉或电平转换，先完成接口适配再连接。启动采集时让车轮静止，避免启动读入初始相位期间运动导致漏计。

```sh
ver all
vgsolar_encoders start
vgsolar_encoders status
listener vgsolar_encoder_status -n 5
```

停住车轮观察 3 秒：计数应保持不变，错误跳变不应持续增加。之后只手动转动右轮一小段，再只转左轮；检查对应轮计数改变、另一侧不变。反向转动同一轮，计数增量符号应反转。解码器定义的正负代表 A/B 相序，只有和实车前进方向对照后才可以命名为“前进/后退”。

QGC 可在 Analyze Tools -> MAVLink 检测中查找 `DEBUG_FLOAT_ARRAY VGS_ENC`：data[0] 为右轮累计原始计数，data[1] 为左轮累计原始计数，data[2] 为无效跳变总数，data[3] 为采集活动标志（1=运行，0=停止）。也可用 Analyze Tools -> Log Viewer 检查本轮 ULog 中的 `vgsolar_encoder_status`。原始 ULog/uORB 整数消息是精确计数依据。

```sh
vgsolar_encoders stop
listener vgsolar_encoder_status -n 1
```

停止后最后一条状态的 `capture_active` 应为 false。将 `ver all`、status/listener 输出、QGC 或示波器记录、轮子转动方向和本轮 ULog 一并保存。

QGC数组的data[3]为采集活动标志（1=运行，0=停止），停采后不会继续刷新。`timestamp_sample=0`表示还没有有效跳变；停止车轮后该时间保持不变属于正常现象。有效/无效跳变总数为uint32，极长时间运行可能回绕；本步用于短时台架。累计净计数使用int64，`direction`只表示这一发布周期内的净增量方向。

## 验收与后续

- 软件验收：解码器纯测试通过，`vg_vgsolar_default` 完整编译通过。
- 实板验收：四个通道逻辑电平/相位顺序符合接线；单轮转动不串扰另一轮；正反向计数符号相反；停止时计数稳定；无效跳变能暴露接触/噪声问题；停止命令撤销全部 EXTI 回调。
- 完成上述实板记录后，再测每轮一圈的计数、齿轮比与轮径；之后才转换为弧度轮角/轮速并评估 PX4 标准 `wheel_encoders`、EKF 或速度闭环。
