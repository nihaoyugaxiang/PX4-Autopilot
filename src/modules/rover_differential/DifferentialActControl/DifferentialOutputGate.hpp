// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <stdint.h>

namespace rover_differential
{
static constexpr uint64_t InputTimeoutUs = 500000;
static constexpr uint8_t SafetyStateSafe = 1;

struct SafetySnapshot {
	uint64_t timestamp{0};
	uint8_t state{0};
	bool enabled{false};
	bool config_valid{false};
	bool left_valid{false};
	bool right_valid{false};
	bool stop_request{true};
	bool latched{true};
};

inline bool fresh(uint64_t timestamp, uint64_t now)
{
	return timestamp > 0 && timestamp <= now && now - timestamp < InputTimeoutUs;
}

inline bool mode_allows(bool selected, bool active, bool motion_allowed, uint64_t timestamp, uint64_t now)
{
	return !selected || (active && motion_allowed && timestamp > 0 && timestamp <= now && now - timestamp < 200000);
}

class OutputGate
{
public:
	// A stopped or stale path requires fresh inputs and one complete standby cycle before motion resumes.
	bool allow(bool safety_enabled, const SafetySnapshot &safety, uint64_t throttle_timestamp,
		   uint64_t steering_timestamp, uint64_t now, bool throttle_updated, bool steering_updated)
	{
		if (safety_enabled && !_safety_was_enabled) { _blocked = true; }
		_safety_was_enabled = safety_enabled;
		const bool safety_ok = !safety_enabled || (safety.enabled && safety.config_valid && safety.left_valid
							 && safety.right_valid && !safety.stop_request && !safety.latched
							 && safety.state == SafetyStateSafe && fresh(safety.timestamp, now));
		const bool controls_ok = fresh(throttle_timestamp, now) && fresh(steering_timestamp, now);

		if (!safety_ok || !controls_ok) {
			_blocked = true;
			return false;
		}

		if (_blocked) {
			if (throttle_updated && steering_updated) {
				_blocked = false;
			}

			return false;
		}

		return true;
	}

private:
	bool _blocked{false};
	bool _safety_was_enabled{false};
};
}
