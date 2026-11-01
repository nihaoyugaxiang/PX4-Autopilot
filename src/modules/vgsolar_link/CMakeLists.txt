# parameters.c supplies the calibrated NCU wheel-radius feedback parameter.
# APM-compatible NCU command and status bridge for the VGSolar bench target.
px4_add_module(
	MODULE modules__vgsolar_link
	MAIN vgsolar_link
	SRCS VgsolarLink.cpp
	DEPENDS px4_work_queue
)

# VGS_WHL_CTRL selects calibrated wheel feedback after disarmed configuration.
