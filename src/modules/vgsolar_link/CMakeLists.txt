# APM-compatible read-only NCU status bridge for the VGSolar bench target.
px4_add_module(
	MODULE modules__vgsolar_link
	MAIN vgsolar_link
	SRCS VgsolarLink.cpp
	DEPENDS px4_work_queue
)
