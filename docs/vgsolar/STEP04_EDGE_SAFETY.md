# 迁移第04步：DYP-A02边缘检测与安全请求

## 本步范围

PX4新增vgsolar_safety模块，订阅两路DYP-A02的distance_sensor，发布vgsolar_safety_status并写入ULog。本步复现APM外侧测距判断，不接入PWM、电机、泵阀、Commander急停或其他执行输出。

APM行为核对结果：仅使用左外侧方向7（YAW_315）和右外侧方向1（YAW_45）；任一路缺失、状态无效或距离严格大于VGS_RF_MAX即为异常。默认门限15cm，15cm本身安全；连续异常默认100ms后锁存。测距恢复后不自动解除，必须显式清除。本步采用更保守的清除条件：两路当下都有效且距离不超限；APM原逻辑的显式清除没有这个前置检查。

## 代码过程

1. 复用DYP-A02驱动已有的多实例distance_sensor消息，不另开串口或重复读取传感器。
2. 按device_id中的DYP-A02设备类型和串口总线号筛选左ttyS7与右ttyS1，再核对方向7/1；不依赖uORB实例编号。
3. 新建纯逻辑SafetyProcessing.hpp：要求消息已启用、超声波类型、方向正确、质量非零或未知、量程内、时间戳不在未来且小于500ms。无效或过期与大于门限同样进入连续消抖。安全模块每20ms检查一次；从首次观测到异常开始，默认满100ms锁存。驱动彻底停止发布时先等待500ms新鲜期，再开始消抖。这些为软件周期预算，实际传感器和队列延迟需由实板日志测量。
4. 连续异常达到门限后发布stop_request=1并保持锁存；恢复不会自动清除。vgsolar_safety clear只提交一次清除请求，若当时任一路无效或超限则拒绝且不留待执行请求。
5. 用独立VgsolarSafetyStatus.msg发布左右距离、有效性、异常侧、状态、持续时间、锁存位和清除计数；默认ULog配置按20ms间隔（最高约50Hz）记录。自定义logger_topics.txt需另外加入vgsolar_safety_status。没有覆盖压力模块正在使用的DEBUG_VECT [VGS_PRESS]；本步状态通过QGC的MAVLink控制台观察，还未增加Inspector消息。
6. 增加VGS_SAFE_EN开机门控（默认0）、VGS_RF_MAX（默认15cm）和VGS_RF_DBNC（默认100ms），保持传感器与业务逻辑独立启停。
7. 主机侧测试覆盖等于/大于阈值、100ms边界、短毛刺恢复、左右缺失/陈旧/未来时间戳、错误方向/质量、锁存保持、危险时拒绝清除、正常时手动清除和非法参数。

非法配置（门限不是1至450cm或消抖不是10至2000ms）会立即生成CONFIG_INVALID锁存请求；改正参数后仍需确认两路恢复正常并手动清除。

## 使用与观察

默认不自动运行，也不需要为本次台架测试修改开机参数。动力断开后，在NSH逐条执行：

    dyp_a02 status
    dyp_a02 start -l /dev/ttyS7 -r /dev/ttyS1
    dyp_a02 status
    vgsolar_safety start
    vgsolar_safety status
    listener vgsolar_safety_status

若第一条status显示DYP-A02已运行，不要重复执行start。预期健康且距离不超过15cm时为STATE_SAFE，stop_request=0。保持传感器和板子固定，用平整目标面分别改变每侧距离至大于15cm并维持至少1秒，状态应变为STATE_LATCHED、该侧unsafe=1。不要让机器人驶向真实边缘。断开一侧串口后，待数据陈旧再消抖，应锁存请求；记录从最后有效帧到锁存的实际延迟。普通手动操作不能验证50ms/150ms边界，精确时间需要后续串口模拟或日志测量。

验证另一侧前先让两路恢复有效安全距离，再执行clear并确认状态回到SAFE；不要在上一轮锁存期间把旧状态当成新的触发。清除命令如下：

    vgsolar_safety clear
    vgsolar_safety status

status中的clear_count应增加，状态回到SAFE。危险条件仍存在时会拒绝并增加rejected_clear_count。可以用listener distance_sensor分别核对两路原始消息，方向7对应左外侧，方向1对应右外侧。参数VGS_SAFE_EN仍为0，手动运行不需要保存新参数。

若需开机自动启动，在参数中设VGS_SAFE_EN=1并重启；同时须启用VGS_RNG_EN=1。只编入模块不等于开机运行。

## 已完成与待实板

已实现和待验证的项目：

- 软件逻辑与本步参数默认值：由源代码和主机侧测试验证。
- VGSolar固件构建：构建通过后随交付记录。
- 实板接线、遮挡距离、断线超时、消抖边界和ULog状态：待用户按T21台架流程验证。
- 任何物理停机效果：本步没有消费者，未实现、未测试、不可宣称通过。

## 安全边界

此模块只是记录并发布停止请求。stop_request=1不是电机/机构已停止的证据；整车安全联锁、解除条件、执行器停机波形及机械停车距离需要在后续模块与独立测试阶段实现。台架时保持电机、电调和清洁机构动力断开。

锁存仅存在于本次模块运行期间；stop/start或重启会重新建立状态机，不是持久化整车ESTOP。stop时消息enabled=false，未来执行器消费者必须对监视器停用、状态陈旧和请求锁存分别采取安全措施，本步没有这样的消费者。

构建与测试命令：

    source /home/yu/.venvs/px4-117/bin/activate
    make -j4 vg_vgsolar_default
    g++ -std=c++14 -fsanitize=address,undefined -fno-omit-frame-pointer -Wall -Wextra -Werror src/modules/vgsolar_safety/test/processing_test.cpp -o /tmp/vgsolar_safety_test
    /tmp/vgsolar_safety_test
