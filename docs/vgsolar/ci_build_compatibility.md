# VGSolar 可选安全模块与标准测试构建兼容修复

日期：2026-11-01；修复前提交：`15c38b6dc4`。

## 原因

`DifferentialActControl` 是通用 Rover 模块，但直接使用了 `ParamInt<px4::params::VGS_SAFE_EN>`。该参数只在启用 `CONFIG_MODULES_VGSOLAR_SAFETY` 时由 `vgsolar_safety/parameters.c` 生成。VGSolar 板启用了这个模块，所以板级构建能通过；标准 `px4_sitl_test` 没有启用，导致编译报错：

```text
DifferentialActControl.hpp: error: 'VGS_SAFE_EN' is not a member of 'px4::params'
```

GitHub 的 `EKF Update Change Indicator` 在 `make tests TESTFILTER=EKF` 的编译阶段失败，尚未开始运行 EKF 测试。LED 提交和其父提交都受影响，问题来自之前共享控制器对可选模块参数的依赖。

## 修复

按 `CONFIG_MODULES_VGSOLAR_SAFETY` 条件声明和读取安全使能参数。标准构建不引用不存在的参数；VGSolar 板继续读取 `VGS_SAFE_EN`，超声波联锁、数据时效检查、锁存与恢复规则不变。没有修改硬件引脚、输出映射、校准参数、LED 或 EKF 算法，也没有通过删除测试或忽略失败让检查通过。

## 验证

在 WSL `XTDrone2-24.04:/home/yu/PX4-Autopilot` 中，先激活既有 Python 环境：

```sh
source /home/yu/.venvs/px4-117/bin/activate
make -j4 tests TESTFILTER=EKF
make -j4 vg_vgsolar_default
python3 -B Tools/vgsolar/run_offline_tests.py --sanitize --all
```

修复前复现了上述编译错误；修复后 EKF 测试 26/26 通过，`src/modules/ekf2/test/change_indication` 无差异。核心板固件构建通过；12 组 C++ 回归（ASan/UBSan）、9 项 NCU 客户端测试、PWM 与生产 LED 后端检查全部通过。板级生成配置仍启用 VGSolar 安全模块，参数表仍包含 `VGS_SAFE_EN`。

这是本地相同测试目标的验证，GitHub 云端结果应单独以新提交的 Actions 页面为准。修复前的提交红叉是历史记录，不会被新提交变绿而覆盖。

## 对核心板的影响

现有已刷写的 VGSolar 固件不需要为了这项兼容修复立即重刷，也不需要重新校准。此次没有进行真实执行器、超声波、遥控器、RK3588 或整车测试，已有硬件验收边界继续保留。

完整日志在 `E:/shuwei/px4/outputs/VGSolar_CI编译修复_20261101/`。
