// SPDX-License-Identifier: BSD-3-Clause
#include "../VgsolarModeLogic.hpp"
#include <assert.h>
#include <stdio.h>

using namespace vgsolar_mode;

int main()
{
 float n=0.f, e=2.f, heading=0.f;
 for(int i=0;i<3000;++i) {
  const auto g = vgsolar_mode::path_guidance(n,e,0.f,0.f,10.f,0.f,.7f);
  assert(g.valid);
  if(g.distance_m <= .201f) { break; }
  const float error=vgsolar_mode::wrap_pi(g.bearing_rad-heading);
  heading+=fminf(.03f,fmaxf(-.03f,error));
  const float v=fminf(1.f,sqrtf(fmaxf(0.f,g.distance_m-.2f))) * fmaxf(0.f,cosf(error));
  n+=v*cosf(heading)*.02f; e+=v*sinf(heading)*.02f;
 }
 assert(hypotf(10.f-n,e) < .22f);

	assert(speed_output(State::Yaw, 0.4f) == 0.4f);
	assert(speed_output(State::YawRate, -0.2f) == -0.2f);
	assert(speed_output(State::Standby, 0.4f) == 0.f);
	assert(speed_output(State::Estop, -0.2f) == 0.f);
	const NavigationSolution nav = solve_navigation(0.f, 0.f, 3.f, 4.f);
	assert(nav.valid && fabsf(nav.distance_m - 5.f) < 1e-5f);
	assert(fabsf(nav.bearing_rad - atan2f(4.f, 3.f)) < 1e-5f);
	assert(!solve_navigation(NAN, 0.f, 1.f, 1.f).valid);
	assert(fabsf(wrap_pi(3.5f) - (3.5f - 2.f * static_cast<float>(M_PI))) < 1e-5f);
	assert(valid_turn(1, 1, 9000, 3000) && !valid_turn(0, 1, 9000, 3000));
	assert(fabsf(turn_rate_rad_s(1, 3000) + static_cast<float>(M_PI) / 6.f) < 1e-5f);
	// A stationary +/-1 degree oscillation never completes a 90 degree turn.
	TurnProgress progress;
	assert(progress.start(0.f, 1.f));
	for (int i = 0; i < 10000; ++i) {
		assert(progress.update(.0174532925f));
		assert(progress.update(-.0174532925f));
		assert(!progress.reached(static_cast<float>(M_PI) / 2.f));
	}
	// Wrong-way travel is subtracted; retracing it does not count twice.
	assert(progress.start(0.f, 1.f));
	assert(progress.update(-1.f) && !progress.reached(.5f));
	assert(progress.update(0.f) && !progress.reached(.5f));
	assert(progress.update(.6f) && progress.reached(.5f));
	// Both directions cross the heading wrap without a discontinuity.
	assert(progress.start(3.f, 1.f));
	assert(progress.update(-3.f) && progress.reached(.28f) && !progress.reached(.3f));
	assert(progress.start(-3.f, -1.f));
	assert(progress.update(3.f) && progress.reached(.28f) && !progress.reached(.3f));
	assert(!progress.update(NAN) && !progress.reached(.1f));
	assert(!progress.start(0.f, 0.f));
	assert(navigation_distance_cm(3.5f) == 350);
	assert(navigation_distance_cm(1000000.f) == 100000000);
	assert(navigation_distance_cm(1e10f) == UINT32_MAX);
	assert(navigation_distance_cm(NAN) == 0 && navigation_distance_cm(-1.f) == 0);
	BrushParameters brush{};
	uint32_t param_value = 0;
	assert(read_brush_parameter(0x0104, brush, param_value) && param_value == 0);
	assert(write_brush_parameter(0x0101, 3, 1, false, brush, param_value) && param_value == 1);
	assert(write_brush_parameter(0x0103, 3, 1, false, brush, param_value) && param_value == 1);
	assert(read_brush_parameter(0x0102, brush, param_value) && param_value == 1);
	assert(!write_brush_parameter(0x0101, 3, 1, true, brush, param_value));
	assert(write_brush_parameter(0x0104, 3, 255, true, brush, param_value) && param_value == 100);
	assert(!write_brush_parameter(0x0104, 2, 50, false, brush, param_value));
	assert(!read_brush_parameter(0x0199, brush, param_value));
	Logic logic;
	assert(valid_speed(1, -500, 32767));
	assert(!valid_speed(1, 501, 0));
	assert(valid_speed(1, 0, -1));
	assert(protocol_yaw_rate(3000) < 0.f);
	assert(protocol_yaw_rate(-3000) > 0.f);
	assert(valid_speed(2, 0, -18000));
	assert(valid_speed(2, 500, 18000));
	assert(!valid_speed(2, 0, 18001));
	assert(!valid_speed(3, 0, 0));
	assert(logic.command(Command::SpeedYaw, false).accepted == false);
	assert(logic.command(Command::SpeedYaw, true).state == State::Yaw);
	assert(logic.timeout().state == State::Standby);
	assert(logic.command(Command::Turn, true).state == State::Turn);
	assert(!logic.command(Command::Turn, true).accepted);
	assert(!logic.command(Command::SpeedYawRate, true).accepted);
	assert(logic.command(Command::Estop, false).state == State::Estop);
	assert(!logic.command(Command::SpeedYawRate, true).accepted);
	assert(!logic.command(Command::Navigation, true).accepted);
	assert(logic.command(Command::ClearEstop, true).state == State::Standby);
	assert(logic.command(Command::Navigation, true).state == State::Navigation);
	assert(logic.command(Command::CancelNavigation, true).state == State::Standby);
	assert(logic.disarm().stop_motion);
	puts("PASS: armed gating, speed timeout, turn priority, estop latch/clear, nav cancel, disarm standby");
}
