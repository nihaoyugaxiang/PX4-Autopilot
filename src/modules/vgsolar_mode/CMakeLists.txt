# VGSolar custom vehicle mode registered with PX4 Commander.
px4_add_module(
	MODULE modules__vgsolar_mode
	MAIN vgsolar_mode
	SRCS VgsolarMode.cpp
	DEPENDS px4_work_queue geo
)
