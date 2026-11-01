// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <stdint.h>

namespace rover_control
{
struct WheelFeedback {
	float body_speed_m_s{NAN};
	float maximum_track_speed_m_s{NAN};
	bool valid{false};
};

inline WheelFeedback wheel_feedback(uint64_t now, uint64_t wheel_timestamp, float right_rad_s, float left_rad_s,
				    uint64_t capture_timestamp, bool capture_active, float radius_m)
{
	WheelFeedback result{};
	if (!capture_active || wheel_timestamp == 0 || wheel_timestamp > now || now - wheel_timestamp >= 500000
	    || capture_timestamp == 0 || capture_timestamp > now || now - capture_timestamp >= 500000
	    || !std::isfinite(radius_m) || radius_m <= 0.f || radius_m > 1.f
	    || !std::isfinite(right_rad_s) || !std::isfinite(left_rad_s)) { return result; }
	// Standard PX4 wheel_encoders uses wheel 0=right and wheel 1=left.
	const float right_m_s = right_rad_s * radius_m;
	const float left_m_s = left_rad_s * radius_m;
	result.body_speed_m_s = (right_m_s + left_m_s) * .5f;
	// A spot turn has zero body speed but is not stationary for lowering suction.
	result.maximum_track_speed_m_s = fmaxf(fabsf(right_m_s), fabsf(left_m_s));
	result.valid = std::isfinite(result.body_speed_m_s) && std::isfinite(result.maximum_track_speed_m_s);
	return result;
}
}
