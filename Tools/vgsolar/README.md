# VGSolar 开发、测试与通信工具

## 构建与完整离线验证（WSL/Linux）

在PX4仓库根目录执行：

```sh
make -j4 vg_vgsolar_default
python3 -B Tools/vgsolar/run_offline_tests.py --sanitize --all
```

`--all` 包含12组C++回归（含生产模式主机测试）、9项NCU客户端测试和PWM、启动LED板级检查。模式测试需要先构建以生成uORB消息头；仅跑11组纯逻辑／协议测试可省略`--all`。主机需要g++、Python3、bash；离线测试本身不需要pyserial、串口或硬件。临时测试二进制在系统临时目录退出后清理，不写入固件源码树。

## 文件用途

| 文件 | 用途 |
|---|---|
| ncu_client.py | RK3588／PC NCU客户端，默认只打印命令，monitor只读，--send明确发送 |
| requirements-rk3588.txt | 实际串口使用所需pyserial依赖 |
| ncu_client_test.py | 协议布局、边界、解析、写入、退出停车、Linux虚拟串口验证 |
| check_boot_led.py | 生产LED后端定时、极性、完成取消及启动顺序检查 |
| check_pwm_bench.py | 五路引脚／定时器、HRT隔离、禁用默认值、NSH设置／停止脚本回归 |
| run_offline_tests.py | 固件纯逻辑／生产模式主机测试及完整测试统一入口 |

[客户端命令说明](README_ncu_client.md)；[软件行为与参数](../../docs/vgsolar/software_parity.md)；[核心板启动与后续验收](../../docs/vgsolar/hardware_bringup.md)。

## 清理原则

旧只读NCU脚本和重复测试已被新客户端覆盖，因此移除。吸盘／导航／安全门控／协议测试及host替身有实际回归用途，保留且不链接进固件。Python缓存和临时输出不提交；构建和固件交付文件保存在既有build目录／外部outputs，不加入Git。

虚拟串口另一端是测试应答器，生产模式主机测试替换了调度与物理接口。两者都不等于实际RK3588到飞控的完整链路、SITL/HIL或整车测试。
