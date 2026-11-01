// SPDX-License-Identifier: BSD-3-Clause
#include "../StatusProcessing.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace vgsolar_link;
int main()
{
	SensorDiagnostics sample{};
	assert(sensor_faults(sample) == ((1u << 5) | (1u << 6)));
	sample.attitude_valid = sample.estimator_healthy = sample.gps_3d = true;
	sample.roll_deg = 30.f; sample.pitch_deg = -30.f;
	assert(sensor_faults(sample) == 0);
	sample.roll_deg = 30.01f; assert(sensor_faults(sample) == (1u << 9));
	sample.roll_deg = 0.f; sample.pitch_deg = -30.01f; assert(sensor_faults(sample) == (1u << 9));
	sample.pitch_deg = 0.f;
	sample.left_enabled = true; assert(sensor_faults(sample) == (1u << 0));
	sample.left_healthy = true; sample.right_enabled = true; assert(sensor_faults(sample) == (1u << 1));
	sample.right_healthy = true; assert(sensor_faults(sample) == 0);
	assert(motion_state(5, false, false, 0) == 0 && motion_state(-5, false, false, 0) == 0);
	assert(motion_state(6, false, false, 0) == 1 && motion_state(-6, false, false, 0) == 2);
	assert(motion_state(0, false, true, 0) == 3 && motion_state(0, true, true, 0) == 4);
	for (unsigned bit : {0u, 1u, 4u, 5u, 8u, 9u, 10u}) { assert(motion_state(0, true, true, 1u << bit) == 5); }
	assert(motion_state(6, false, false, 1u << 6) == 1); // GPS loss alone is not motor FAULT.
	assert(motion_state(0, true, false, 1u << 11) == 4); // Range uses ESTOP, like APM.
	NavigationReportTracker reports;
	assert(!reports.needed(0, 0, 0));
	assert(reports.needed(1, 3, 100)); reports.queued(1, 3, 100);
	assert(reports.needed(1, 3, 100)); // Active always streams.
	assert(reports.needed(3, 3, 200)); reports.queued(3, 3, 200);
	assert(!reports.needed(3, 3, 200));
	assert(reports.needed(3, 3, 300)); // A new failed request has the same state.
	assert(reports.needed(3, 3, 300)); // A full TX queue must not consume the event.
	reports.queued(3, 3, 300); assert(!reports.needed(3, 3, 300));
	puts("PASS: APM IMU/GPS/encoder/tilt fault bits, inclusive tilt threshold and motion-state priority and distinct terminal navigation events");
}
