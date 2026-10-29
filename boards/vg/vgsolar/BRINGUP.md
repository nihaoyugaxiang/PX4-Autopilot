# VGSolar 实机启动记录（2026-10-29）

证据来自用户在 QGroundControl MAVLink Console 中提交的 NSH 输出，未由主机工具直接读取硬件。

- `ver all`：VG_VGSOLAR，PX4 1.17.0，Git d6f12ad1c4f70ad3230afd7d86e971421e02fef4，STM32H7 rev V。
- 用户在首次启动后手动设置 `SYS_AUTOSTART=50000` 并重启，`rover_differential status` 报 running。若已有参数把此值保存为 0，set-default 不会覆盖已保存值。
- gyro/accel ID 2490378、MAG ID 994337、BARO ID 6780449 均报告 validator OK。EKF healthy，attitude=1，IMU missed=0；没有本地/全球定位。
- MTD 枚举 `/fs/mtd_params`，32768 bytes。尚未证明彻底断电后的参数保存与恢复。
- logger 写入 `/fs/microsd/log/2026-10-29/08_35_54.ulg`，2.03 MiB，报告 0 dropouts。
- USB 标识沿用 1209:5740，Windows 旧 ArduPilot composite INF 错绑导致伪 COM65/66，其中 COM66 code 10。将无 COM 号的父设备手动改为 Microsoft usbser.inf 后，主机识别正常的 COM67。此改动是用户在设备管理器完成的。

## B0 IMU 方向问题与 B1 修复

用户确认 STM32 主芯片所在面朝上，USB 接口所在边朝机器人前方。B0 在这个摆放下报告 Roll=-178.2°，Pitch=5.1°，accel=(-0.04222,-0.00636,+10.39567) m/s²；SENS_BOARD_ROT 与三个微调角均为 0。

AP Invensensev3 对 FIFO 原始 XYZ 直接应用 hwdef 的旋转 27，即 (x,y,z)→(y,x,-z)。PX4 ICM42688P 驱动先对原始数据做 (x,y,z)→(x,-y,-z)，随后 PX4Accelerometer/PX4Gyroscope 才应用 -R。因此应将本板驱动参数从 27 改为 2（Yaw 90°）：R2·diag(1,-1,-1)=R27。不能因枚举名相同就复用编号。

B1 修正本板传感器启动脚本的旋转，不改通用传感器驱动。上式已对三个基向量核对；它证明与 AP 配置等效，并不单独证明 USB 朝前的实物安装方向正确。后续实机读数确定还需设置整板安装旋转 SENS_BOARD_ROT=6，见下节。

更改驱动旋转会改变发布给校准层的数据坐标。B0 中已经存在的 CAL_ACC0/CAL_GYRO0 校准值不能直接当作 B1 的有效校准，应先导出参数，再对这些传感器单独重做校准，不重置所有参数。

## B1 实物方向验证与安装参数

- B1 驱动使用 `-R 2`。用户刷入后，STM32 面朝上静止时 `sensor_accel=(-0.10199,0.11564,-9.78774) m/s²`，Roll=-0.3°、Pitch=-0.8°；倒置问题消失，error_count=0、clip_counter=[0,0,0]。
- `SENS_BOARD_ROT=0` 时抬高 USB 所在边，Roll=-25.6°、Pitch=-0.6°，表明驱动参考坐标的 X 轴还没有对应 USB 朝前的车体方向。
- 用户设置并保存 `SENS_BOARD_ROT=6`（Yaw 270°）后重启，输出 `x + SENS_BOARD_ROT : 6`；USB 端抬高时 Pitch=+30.2°、Roll=-1.1°，前向俯仰轴测试符合预期。此时平放读数为 Roll=-4.6°、Pitch=+1.4°，仍待静置、六面校准与安装水平校准。
- 当前已测试组合是 **驱动 `-R 2` + 安装参数 `SENS_BOARD_ROT=6`**，参考姿态为 **STM32 面朝上、USB 所在边朝车头**。`sensor_accel` 话题处于驱动坐标系，`sensor_combined`/`vehicle_attitude` 已应用整板安装旋转。参数 6 是用户写入实物并保存的值，**尚未写入 rc.board_defaults**；把此分支编译后刷入另一块全新板，不会自动获得该安装参数。
- 倾斜时 Yaw 从 2.9° 跳到 144.9°、quat_reset_counter 从 0 变为 1；罗盘轴向、校准、磁干扰和航向稳定性未验证，不能声称航向测试通过。

新板采用同一安装方向时，在 QGC MAVLink Console 单独设置以下参数，再做方向测试和传感器校准。安装方向不同须另行确认。

```text
param set SYS_AUTOSTART 50000
param set SENS_BOARD_ROT 6
param save
reboot
```

## 接下来

- [x] B1 STM32 面朝上静止时加速度 Z 接近 -9.81 m/s²，倒置已消除。
- [x] `-R 2` 与 `SENS_BOARD_ROT=6` 组合下，USB 端抬高时 Pitch 为正。
- [ ] 从 USB 朝前的车身视角，右侧压低时 Roll 为正；此项尚未收到用户读数。
- [ ] 重做陀螺仪和加速度计六面校准，安装后做水平校准。
- [ ] 内置磁力计与 IMU 轴的一致性需独立核对，不能用一次水平静止读数证明。
- [ ] 验证完全断电再上电的参数保存/加载。
- [ ] 校准后检查预检和日志，再处理定位、编码器与执行器；B1 仍不包含 PWM/DShot/PX4IO 输出。
