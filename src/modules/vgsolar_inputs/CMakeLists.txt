# 复用现有ADC消息，只添加输入处理工作项，不申请或重新初始化ADC。
px4_add_module(
	MODULE modules__vgsolar_inputs
	MAIN vgsolar_inputs
	SRCS VgsolarInputs.cpp
	DEPENDS px4_work_queue
)
