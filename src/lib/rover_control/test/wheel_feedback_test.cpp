// SPDX-License-Identifier: BSD-3-Clause
#include "../WheelSpeedFeedback.hpp"
#include <cassert>
#include <cstdio>
#include <cfloat>
using namespace rover_control;
int main()
{
	const uint64_t now = 1000000;
	auto feedback = wheel_feedback(now, now, 4.f, 6.f, now, true, .1f);
	assert(feedback.valid && fabsf(feedback.body_speed_m_s - .5f) < 1e-6f);
	assert(fabsf(feedback.maximum_track_speed_m_s - .6f) < 1e-6f);
	feedback = wheel_feedback(now, now, -4.f, -6.f, now, true, .1f);
	assert(feedback.valid && fabsf(feedback.body_speed_m_s + .5f) < 1e-6f);
	feedback = wheel_feedback(now, now, -4.f, 4.f, now, true, .1f);
	assert(feedback.valid && fabsf(feedback.body_speed_m_s) < 1e-6f && feedback.maximum_track_speed_m_s > .39f);
	assert(wheel_feedback(now, now, 0.f, 0.f, now, true, .1f).valid);
	assert(!wheel_feedback(now, now, 0.f, 0.f, now, false, .1f).valid);
	assert(!wheel_feedback(now, 0, 1.f, 1.f, now, true, .1f).valid);
	assert(!wheel_feedback(now, now + 1, 1.f, 1.f, now, true, .1f).valid);
	assert(wheel_feedback(now, now - 499999, 1.f, 1.f, now, true, .1f).valid);
	assert(!wheel_feedback(now, now - 500000, 1.f, 1.f, now, true, .1f).valid);
	assert(!wheel_feedback(now, now, 1.f, 1.f, now - 500000, true, .1f).valid);
	assert(!wheel_feedback(now, now, 1.f, 1.f, now + 1, true, .1f).valid);
	assert(!wheel_feedback(now, now, NAN, 1.f, now, true, .1f).valid);
	assert(!wheel_feedback(now, now, 1.f, INFINITY, now, true, .1f).valid);
	assert(!wheel_feedback(now, now, 1.f, 1.f, now, true, 0.f).valid);
	assert(!wheel_feedback(now, now, 1.f, 1.f, now, true, NAN).valid);
	assert(!wheel_feedback(now, now, FLT_MAX, FLT_MAX, now, true, 1.f).valid);
	puts("PASS: calibrated signed wheel/body speed, spot-turn standstill guard, active capture, radius, finite domain and freshness");
}
