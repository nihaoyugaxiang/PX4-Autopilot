# SPDX-License-Identifier: BSD-3-Clause
px4_add_module(
	MODULE drivers__dyp_a02
	MAIN dyp_a02
	SRCS DypA02.cpp
	DEPENDS drivers_rangefinder px4_work_queue
)
