# 仅订阅测距并发布请求；不添加执行器依赖。
px4_add_module(
	MODULE modules__vgsolar_safety
	MAIN vgsolar_safety
	SRCS VgsolarSafety.cpp
	DEPENDS px4_work_queue
)
