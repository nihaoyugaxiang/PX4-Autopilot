# SPDX-License-Identifier: BSD-3-Clause
# 注册NSH命令dyp_a02；实际固件代码从DypA02.cpp编译。
# Protocol.hpp由驱动包含，parameters.c供PX4参数生成工具扫描。
# 两个依赖分别提供标准距离消息发布封装和周期工作队列。
px4_add_module(
	MODULE drivers__dyp_a02
	MAIN dyp_a02
	SRCS DypA02.cpp
	DEPENDS drivers_rangefinder px4_work_queue
)
