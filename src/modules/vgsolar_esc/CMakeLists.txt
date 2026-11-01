px4_add_module(
 MODULE modules__vgsolar_esc
 MAIN vgsolar_esc
 SRCS VgsolarEsc.cpp
 DEPENDS px4_work_queue
)
