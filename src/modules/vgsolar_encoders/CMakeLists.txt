# 只读采集AB相边沿，发布原始诊断与校准后的标准轮编码器消息。
px4_add_module(
	MODULE modules__vgsolar_encoders
	MAIN vgsolar_encoders
	SRCS VgsolarEncoders.cpp
	DEPENDS px4_work_queue
)
