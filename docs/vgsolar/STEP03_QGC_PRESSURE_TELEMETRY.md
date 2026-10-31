# 第03步补充：QGC实时显示PC0电压与吸盘压力

日期：2026-10-31。分支：`feat/vgsolar-px4-migration`。该说明与QGC压力遥测实现一同纳入功能提交。

## 目标和范围

让USB连接的QGroundControl实时检查PC0 ADC电压、按当前标定换算并滤波后的吸盘表压，以及该压力当前是否有效。数据只用于显示和台架校验，不接入吸附控制；仍以`vgsolar_inputs`作为后续软件读取接口，消费者必须检查有效状态和时间戳。

这会出现在QGC Analyze Tools中的MAVLink Inspector，不会自动成为Fly View标准气压高度仪表。QGC Inspector可展示收到的MAVLink消息及字段，也可以绘制实时曲线。

## 实现过程

1. `vgsolar_inputs`继续复用板载`board_adc`发布的`adc_report`，定位`channel_id=10`后换算PC0电压；不重新初始化ADC，也不改变电池采样。
2. 原输入模块根据`VGS_P_VMIN/VMAX`与`VGS_P_PMIN/PMAX`进行端点线性换算、有效性检查及5Hz低通；当前默认1.6V约为−50kPa。
3. 每20ms发布一个PX4既有`debug_vect`，名称`VGS_PRESS`。PX4 MAVLink模块已有`DEBUG_VECT`消息流，USB配置速率为50Hz；没有新增或修改MAVLink协议定义。
4. DEBUG_VECT字段映射：`x`最近一次PC0电压(V)，`y`滤波压力(kPa)，`z`有效标志(1=有效、0=无效)。无效时`y`为NaN；ADC停止更新时`x`可能保留旧样本，所以判读时必须一起看`z`。停止输入模块时也会发出`z=0`。

数据路径：

```text
PC0 -> board_adc/adc_report(channel 10) -> vgsolar_inputs换算/滤波
    -> uORB debug_vect(name=VGS_PRESS) -> MAVLink DEBUG_VECT -> QGC Inspector
```

## QGC查看步骤

1. USB连接飞控，确认QGC已有正常车辆连接。
2. Analyze Tools中打开“MAVLink检测 / MAVLink Inspector”。
3. 展开`DEBUG_VECT`，找到名称为`VGS_PRESS`的向量。
4. 查看`x`、`y`、`z`实时字段；勾选`x`和`y`曲线，必要时也勾选`z`。
5. 测试中以`z=1`作为压力数据软件有效的必要条件。`z=0`时不要把y当作有效压力。Inspector没有`VGS_PRESS`时，先确认NSH中模块运行，再检查USB MAVLink链路/流。

## 台架测试

断开MCP-H10-P传感器的输出线后再用可调电压源模拟，避免两个输出源互相驱动。飞控保持断电接线；信号源地连接飞控地，限流后信号接PC0。只施加0.1～3.1V，绝不超过3.3V。上电后启动输入模块：

```sh
vgsolar_inputs start
vgsolar_inputs status
mavlink status
```

用万用表直接测PC0对板地确认实际输入电压，然后在Inspector观察`VGS_PRESS`。每个稳定电压保持至少1秒，让100ms有效预热和低通滤波稳定：

| PC0电压 | 默认压力换算 | 期望z |
|---:|---:|---:|
| 0.1V | 约0kPa | 1 |
| 1.6V | 约−50kPa | 1 |
| 3.1V | 约−100kPa | 1 |

接近3.1V的量测值可能受ADC参考误差影响并越过允许的3.2V有效边界；以PC0万用表读数和当前换算公式判断，不要把3.3V轨附近读数当作正常标定点。`x`应跟随实测输入，`y`应与线性端点换算相符，`z`为1。发生无效/过期输入或停止模块时，`z`应为0。

结束时先将信号源输出调到0V并关闭，再拆线并恢复传感器信号连接。不要用导线短接PC0，不要在传感器仍连着时并接信号源。

## 验证与边界

- 通过：`vg_vgsolar_default`交叉编译成功；MAVLink既有DEBUG_VECT发送端订阅debug_vect，USB默认流包含DEBUG_VECT；模块的映射字段与消息结构一致。
- 上板后仍需确认：QGC Inspector确实收到`VGS_PRESS`；0.1/1.6/3.1V多点换算与PC0万用表值相符；压力真实精度需再用压力表对比。
- 当前遥测是调试向量，并未提供自定义QGC仪表布局；QGC不显示中文工程标签，使用固定消息名和x/y/z定义。
- 该遥测不表示气压高度，不改`SCALED_PRESSURE`，也不改变EKF、罗盘校准或任何执行器输出。
