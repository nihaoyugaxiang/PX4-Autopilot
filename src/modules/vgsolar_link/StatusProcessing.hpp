// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <stdint.h>
namespace vgsolar_link
{
// Terminal reports are events: identical failures from separate requests each
// need a frame, while the same persistent status must not be repeated at 10 Hz.
class NavigationReportTracker
{
public:
	bool needed(uint8_t state, uint8_t mode, uint64_t event_timestamp) const {
		return state != 0 && (state == 1 || state != _state || mode != _mode || event_timestamp != _event);
	}
	void queued(uint8_t state, uint8_t mode, uint64_t event_timestamp) {
		_state = state; _mode = mode; _event = event_timestamp;
	}
private:
	uint8_t _state{0}, _mode{0};
	uint64_t _event{0};
};
struct SensorDiagnostics {
	bool attitude_valid{false}, estimator_healthy{false}, gps_3d{false};
	bool left_enabled{false}, right_enabled{false}, left_healthy{false}, right_healthy{false};
	float roll_deg{NAN}, pitch_deg{NAN};
};
inline uint16_t sensor_faults(const SensorDiagnostics &input)
{
	uint16_t faults = 0;
	if (!input.attitude_valid || !input.estimator_healthy) { faults |= 1u << 5; }
	if (!input.gps_3d) { faults |= 1u << 6; }
	if (input.left_enabled && !input.left_healthy) { faults |= 1u << 0; }
	if (input.right_enabled && !input.right_healthy) { faults |= 1u << 1; }
	if (input.attitude_valid && (fabsf(input.roll_deg) > 30.f || fabsf(input.pitch_deg) > 30.f)) { faults |= 1u << 9; }
	return faults;
}
inline uint8_t motion_state(int16_t velocity_cms, bool estop, bool turning, uint16_t faults)
{
	constexpr uint16_t mask = (1u << 0) | (1u << 1) | (1u << 4) | (1u << 5) | (1u << 8) | (1u << 9) | (1u << 10);
	if ((faults & mask) != 0) { return 5; }
	if (estop) { return 4; }
	if (turning) { return 3; }
	return velocity_cms > 5 ? 1 : velocity_cms < -5 ? 2 : 0;
}
}
