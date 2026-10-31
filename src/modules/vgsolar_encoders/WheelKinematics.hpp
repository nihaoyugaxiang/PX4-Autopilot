// SPDX-License-Identifier: BSD-3-Clause
// Convert calibrated quadrature counts to wheel-shaft angle and angular speed.
#pragma once

#include <math.h>
#include <stdint.h>

namespace vgsolar_encoders
{
static constexpr double TwoPi = 6.28318530717958647692;

struct WheelKinematics {
	float angle_rad;
	float speed_rad_s;
	bool configured;
};

inline WheelKinematics calculate_wheel_kinematics(int64_t count, int64_t delta, int32_t counts_per_revolution,
		uint64_t interval_us, bool reverse_direction)
{
	const float unavailable = NAN;

	if (counts_per_revolution <= 0) {
		return {unavailable, unavailable, false};
	}

	const double direction = reverse_direction ? -1.0 : 1.0;
	const double radians_per_count = TwoPi / static_cast<double>(counts_per_revolution);
	const float angle = static_cast<float>(static_cast<double>(count) * radians_per_count * direction);
	const float speed = interval_us > 0 ? static_cast<float>(static_cast<double>(delta) * radians_per_count * 1e6 /
			static_cast<double>(interval_us) * direction) : unavailable;
	return {angle, speed, true};
}
}
