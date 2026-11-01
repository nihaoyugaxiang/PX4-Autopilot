// SPDX-License-Identifier: BSD-3-Clause
#include "host/px4_stubs.hpp"
#include <cassert>
// Expose only the production class's private entry points to this test.
// All external headers are already included, avoiding changes to their access rules.
#define private public
#include "../VgsolarMode.cpp"
#undef private

using namespace vgsolar_mode;

static vehicle_local_position_s local{};
static uint8_t mode_id = 23;
static float right_speed = 0.f, left_speed = 0.f;
static bool wheel_active = true;
static float attitude_roll = 0.f;
static bool range_messages = false;
static vgsolar_safety_status_s safety{};
static void health(bool armed = true)
{
	vehicle_status_s vehicle{};
	vehicle.timestamp = host::now; vehicle.nav_state = mode_id;
	vehicle.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
	host::publish("vehicle_status", vehicle);
	actuator_armed_s arm{}; arm.timestamp = host::now; arm.armed = armed;
	host::publish("actuator_armed", arm);
	local.timestamp = host::now;
	host::publish("vehicle_local_position", local);
	vehicle_attitude_s attitude{}; attitude.timestamp = host::now;
	matrix::Quatf{matrix::Eulerf{attitude_roll, 0.f, local.heading}}.copyTo(attitude.q);
	host::publish("vehicle_attitude", attitude);
	wheel_encoders_s wheel{}; wheel.timestamp = host::now;
	wheel.wheel_speed[0] = right_speed; wheel.wheel_speed[1] = left_speed;
	host::publish("wheel_encoders", wheel);
	vgsolar_encoder_status_s capture{}; capture.timestamp = host::now; capture.capture_active = wheel_active;
	host::publish("vgsolar_encoder_status", capture);
	if (range_messages) { safety.timestamp = host::now; host::publish("vgsolar_safety_status", safety); }
}
static vgsolar_mode_status_s status() { return host::latest<vgsolar_mode_status_s>("vgsolar_mode_status"); }
static void step(VgsolarMode &mode, bool armed = true)
{
	host::now += 20000; health(armed); mode.Run();
}
static void send(vgsolar_command_s command)
{
	command.timestamp = host::now; host::publish("vgsolar_command", command);
}
static vgsolar_command_s speed()
{
	vgsolar_command_s command{}; command.command = vgsolar_command_s::CMD_SPEED;
	command.control_mode = 2; command.velocity_cms = 40; return command;
}
static vgsolar_command_s system(uint8_t value)
{
	vgsolar_command_s command{}; command.command = vgsolar_command_s::CMD_SYSTEM;
	command.system_command = value; return command;
}
static vgsolar_command_s nav(uint8_t kind)
{
	vgsolar_command_s command{}; command.command = vgsolar_command_s::CMD_POSITION;
	command.nav_mode = kind; command.target_x = 1000; command.arrival_radius_cm = 10;
	command.arrival_yaw_cd = 0xffff; command.cruise_speed_cms = 30; return command;
}
static vgsolar_command_s turn()
{
	vgsolar_command_s command{}; command.command = vgsolar_command_s::CMD_TURN;
	command.turn_mode = 1; command.turn_direction = 1;
	command.turn_angle_cd = 9000; command.turn_rate_cds = 3000; return command;
}
int main()
{
 assert(fabsf(vgsolar::battery_remaining(22.5f, .1f, true) - .5f) < .001f);
 assert(vgsolar::battery_remaining(30.f, .1f, true) == 1.f);
 assert(vgsolar::battery_remaining(10.f, .9f, true) == 0.f);
 assert(std::isnan(vgsolar::battery_remaining(22.f, .5f, false)));
 assert(std::isnan(vgsolar::battery_remaining(NAN, .5f, true)));

	host::parameter<int32_t>("VGS_ENABLE", 1);
	host::parameter<int32_t>("VGS_CLN_EN", 1);
	host::parameter<int32_t>("VGS_SC_IR", 0);
	host::parameter<int32_t>("VGS_SC_PRESS", 0);
	host::parameter<float>("VGS_SC_RATE", 5000.f);
	host::parameter<int32_t>("VGS_SC_LIFT_DLY", 0);
	host::parameter<int32_t>("VGS_SC_VAC_DLY", 100);
	host::parameter<int32_t>("VGS_SC_VENT_DLY", 100);
	local.xy_valid = local.v_xy_valid = local.heading_good_for_control = true;
	VgsolarMode mode;
	mode.Run(); // registration request
	auto request = host::latest<register_ext_component_request_s>("register_ext_component_request");
	assert(request.register_mode && std::strcmp(request.name, "VGSolar") == 0);
	register_ext_component_reply_s reply{};
	reply.request_id = request.request_id; reply.success = true; reply.mode_id = mode_id; reply.arming_check_id = 1;
	host::publish("register_ext_component_reply", reply);
	step(mode); assert(status().active && status().state == 0 && !status().motion_allowed);

	// An unimplemented system command must not turn off the speed watchdog.
	send(speed()); step(mode); assert(status().state == 2 && status().motion_allowed);
	for (int i = 0; i < 10; ++i) { step(mode); }
	send(system(4)); step(mode); assert(status().state == 2);
	for (int i = 0; i < 16; ++i) { step(mode); }
	assert(status().state == 0 && !status().motion_allowed);
	// A rejected reboot while armed has the same watchdog invariant.
	send(speed()); step(mode); send(system(3)); step(mode);
	for (int i = 0; i < 26; ++i) { step(mode); }
	assert(!host::reboot_requested && status().state == 0);

	// ESTOP wins, clearing it cannot execute a speed queued in the same batch.
	send(speed()); send(system(1)); step(mode); assert(status().state == 5 && !status().motion_allowed);
	send(system(2)); send(speed()); step(mode); assert(status().state == 0 && !status().motion_allowed);
	send(speed()); step(mode); assert(status().state == 2 && status().motion_allowed);
	step(mode, false); assert(status().state == 0 && !status().motion_allowed);
	// Pending commands are discarded on arm and mode-entry edges.
	send(speed()); step(mode); assert(status().state == 0);

	// Cancellation is standby even when sent during speed/heading control.
	auto yaw_speed = speed(); yaw_speed.control_mode = 1; yaw_speed.yaw_data = 9000;
	send(yaw_speed); step(mode); assert(status().state == 1);
	send(nav(4)); step(mode); assert(status().state == 0 && !status().motion_allowed);
	// System ESTOP cancels navigation instead of streaming NAV_ACTIVE forever.
	send(nav(3)); step(mode); assert(status().navigation_state == 1);
	send(system(1)); step(mode); assert(status().state == 5 && status().navigation_state == 4);
	send(system(2)); step(mode); assert(status().state == 0);

	// A changed estimator frame invalidates the mode-entry NED origin.
	send(nav(2)); step(mode); assert(status().state == 4);
	++local.ref_timestamp; step(mode); assert(status().navigation_state == 3 && !status().motion_allowed);
	send(nav(2)); step(mode); assert(status().state == 0 && status().navigation_state == 3);
	// Body goals can use the fresh frame; NED keeps its original-entry contract.
	send(nav(3)); step(mode); assert(status().state == 4);
	send(nav(4)); step(mode); assert(status().state == 0);
	mode_id = 0; step(mode); assert(!status().active);
	mode_id = 23; step(mode); assert(status().active && status().state == 0);
	send(nav(2)); step(mode); assert(status().state == 4);
	send(nav(4)); step(mode); assert(status().state == 0);

	// GPS conversion shares the estimator projection, including longitude wrap.
	local.xy_global = true; local.ref_lat = 0.; local.ref_lon = 179.9999;
	++local.ref_timestamp; step(mode);
	auto gps_goal = nav(1); gps_goal.target_x = -1799999000; gps_goal.target_y = 0;
	send(gps_goal); step(mode);
	assert(status().state == 4 && status().nav_distance_cm > 2200 && status().nav_distance_cm < 2250);
	assert(status().nav_heading_error_cd < -8990 && status().nav_heading_error_cd > -9010);
	send(nav(4)); step(mode);
	// Geographic coordinates retain double precision at centimetre offsets.
	local.ref_lat = 30.; local.ref_lon = 120.; ++local.ref_timestamp; step(mode);
	gps_goal.target_x = 1200000001; gps_goal.target_y = 300000001;
	gps_goal.arrival_radius_cm = 1;
	send(gps_goal); step(mode);
	assert(status().nav_distance_cm >= 1 && status().nav_distance_cm <= 2);
	send(nav(4)); step(mode);

	// Arrival-heading feedback is relative to the alignment target, not the old bearing.
	auto align = nav(3); align.target_x = align.target_y = 0; align.arrival_yaw_cd = 9000;
	send(align); step(mode); assert(status().state == 4 && status().nav_heading_error_cd == -9000);
	send(nav(4)); step(mode); assert(status().state == 0);

	// A rejected replacement cannot say FAILED while following the old target.
	local.xy_global = false;
	send(nav(3)); step(mode); assert(status().navigation_state == 1 && status().motion_allowed);
	send(nav(1)); step(mode); // no global reference
	assert(status().navigation_state == 3 && status().state == 0 && !status().motion_allowed);
	assert(status().nav_mode == 1);
	const uint64_t failed_event = status().navigation_event_timestamp;
	send(nav(1)); step(mode);
	assert(status().navigation_state == 3 && status().navigation_event_timestamp > failed_event);
	local.xy_global = true; local.ref_lat = 30.; local.ref_lon = 120.;
	send(nav(3)); step(mode); assert(status().state == 4);
	auto invalid = nav(1); invalid.target_y = 1000000000; send(invalid); step(mode);
	assert(status().navigation_state == 3 && status().state == 0 && !status().motion_allowed);
	send(nav(3)); step(mode); assert(status().state == 4);
	++local.xy_reset_counter; step(mode);
	assert(status().navigation_state == 3 && !status().motion_allowed);

	// Run production TURN through stop/dwell/lower/vacuum, then inject noise.
	send(nav(3)); step(mode); assert(status().state == 4);
	send(turn()); step(mode); assert(status().state == 3 && !status().motion_allowed && status().navigation_state == 4);
	for (int i = 0; i < 200 && status().turn_phase != 4; ++i) { step(mode); }
	assert(status().turn_phase == 4 && status().motion_allowed);
	for (int i = 0; i < 200; ++i) {
		local.heading = i % 2 ? -.02f : .02f; step(mode);
		assert(status().state == 3 && status().turn_phase == 4);
	}
	// Rotate left by 90 degrees: finish, vent, raise, return to standby.
	for (int i = 1; i <= 16 && status().turn_phase == 4; ++i) { local.heading = -.1f * i; step(mode); }
	assert(status().turn_phase == 5 && !status().motion_allowed);
	for (int i = 0; i < 100 && status().state == 3; ++i) { step(mode); }
	assert(status().state == 0 && !status().motion_allowed && !host::gpio[GPIO_VG_PUMP_OFF]);

	// EKF heading reset preempts turning, never counts it as completed motion.
	send(turn()); step(mode);
	for (int i = 0; i < 200 && status().turn_phase != 4; ++i) { step(mode); }
	assert(status().turn_phase == 4);
	++local.heading_reset_counter; local.heading = 1.f; step(mode);
	assert(status().turn_phase == 5 && !status().motion_allowed);
	for (int i = 0; i < 100 && status().state == 3; ++i) { step(mode); }
	assert(status().state == 0);
	// Excessive tilt freezes suction and clears brushes; recovery aborts the turn.
	send(turn()); step(mode);
	for (int i = 0; i < 200 && status().turn_phase != 4; ++i) { step(mode); }
	assert(status().turn_phase == 4 && host::gpio[GPIO_VG_PUMP_OFF]);
	attitude_roll = math::radians(40.f); step(mode);
	assert(status().suction_frozen && !status().motion_allowed && !host::gpio[GPIO_VG_PUMP_OFF]);
	assert(!host::gpio[GPIO_VG_VALVE_OFF]); // Hold sealed, like APM freeze.
	attitude_roll = 0.f; step(mode);
	assert(status().state == 0 && !status().suction_frozen && !status().motion_allowed);
	send(speed()); step(mode); assert(status().state == 0); // Still raising.
	for (int i = 0; i < 100 && status().suction_state != 0; ++i) { step(mode); }
	assert(status().suction_state == 0 && !host::gpio[GPIO_VG_PUMP_OFF]);

	// Drive production parameter dispatch through servo output and battery lockout.
	vgsolar_command_s parameter{}; parameter.command = vgsolar_command_s::CMD_PARAM_WRITE;
	parameter.param_type = 3; parameter.param_index = 0x0104; parameter.param_value = 40; send(parameter);
	parameter.param_index = 0x0101; parameter.param_value = 1; send(parameter); step(mode);
	auto servos = host::latest<actuator_servos_s>("actuator_servos");
	assert(fabsf(servos.control[1] + .2f) < 1e-6f && fabsf(servos.control[2] + 1.f) < 1e-6f);
	battery_status_s battery{}; battery.timestamp = host::now; battery.remaining = .20f; battery.connected = true; battery.voltage_v = 20.88f;
	host::publish("battery_status", battery); step(mode);
	servos = host::latest<actuator_servos_s>("actuator_servos");
	assert(fabsf(servos.control[1] + 1.f) < 1e-6f);
	send(parameter); step(mode);
	assert(!host::latest<vgsolar_link_ack_s>("vgsolar_link_ack").success);
	battery.timestamp = host::now; battery.remaining = .9f; battery.voltage_v = 24.66f; host::publish("battery_status", battery);
	step(mode); servos = host::latest<actuator_servos_s>("actuator_servos");
	assert(fabsf(servos.control[1] + 1.f) < 1e-6f); // Recovery does not restart old brushes.

	// The optional encoder source supplies body-speed validation without EKF velocity.
	host::parameter<int32_t>("VGS_WHL_CTRL", 1);
	host::parameter<float>("VGS_WHL_RAD", .1f);
	for (int i = 0; i < 55; ++i) { step(mode, false); }
	local.v_xy_valid = false; local.vx = local.vy = NAN;
	step(mode); send(speed()); step(mode);
	assert(status().state == 2 && status().motion_allowed);
	wheel_active = false; step(mode);
	assert(status().state == 0 && !status().motion_allowed);
	wheel_active = true; right_speed = NAN;
	send(speed()); step(mode); assert(status().state == 0);
	right_speed = 0.f;
	// Do not fall back to the estimator after changing the source while armed.
	host::parameter<int32_t>("VGS_WHL_CTRL", 0);
	for (int i = 0; i < 55; ++i) { step(mode); }
	send(speed()); step(mode); assert(status().state == 2);
	// Opposing tracks average to zero, but lowering is blocked until both stop.
	auto stop = speed(); stop.velocity_cms = 0; send(stop); step(mode);
	right_speed = -2.f; left_speed = 2.f;
	send(turn()); step(mode);
	for (int i = 0; i < 40; ++i) { step(mode); assert(status().turn_phase == 1); }
	right_speed = left_speed = 0.f;
	for (int i = 0; i < 200 && status().turn_phase != 4; ++i) { step(mode); }
	assert(status().turn_phase == 4);
	wheel_active = false; step(mode);
	assert(status().turn_phase == 5 && !status().motion_allowed);
	// Range clear is an explicit fresh handshake, never a delayed dangerous clear.
	wheel_active = true;
	local.v_xy_valid = true; local.vx = local.vy = 0.f;
	range_messages = true; safety.enabled = safety.config_valid = safety.left_valid = safety.right_valid = true;
	safety.state = 1;
	host::parameter<int32_t>("VGS_SAFE_EN", 1);
	for (int i = 0; i < 55; ++i) { step(mode, false); }
	// Disarm inhibits lift output: rearming must first complete the pending release.
	step(mode);
	for (int i=0;i<150 && !mode._suction.raised();++i) { step(mode); }
	send(speed()); step(mode); assert(status().motion_allowed);
	safety.stop_request = safety.latched = true; safety.state = 2; step(mode);
	assert(status().state == 5 && !status().motion_allowed);
	send(system(2)); step(mode);
	assert(status().state == 5 && host::latest<vgsolar_safety_request_s>("vgsolar_safety_request").timestamp == 0);
	safety.clear_eligible = true;
	send(system(2)); step(mode);
	assert(status().state == 5 && host::latest<vgsolar_safety_request_s>("vgsolar_safety_request").clear);
	safety.stop_request = safety.latched = false; safety.state = 1; step(mode);
	assert(status().state == 0 && !status().motion_allowed);
	send(speed()); step(mode); assert(status().motion_allowed);

 // Full production ESTOP during a lowered suction cycle preserves vent/ramp sequencing.
 mode._suction.clear_fault();
 assert(mode._suction.lower(host::now));
 for(int i=0;i<200 && !mode._suction.lowered();++i) { step(mode); }
 assert(mode._suction.lowered());
 const float lowered_pwm = mode._suction.pwm();
 send(system(1)); step(mode);
 assert(status().state == 5 && !status().motion_allowed && !mode._suction.raised());
 assert(mode._suction.pwm() == lowered_pwm);
 assert(!host::gpio[GPIO_VG_PUMP_OFF] && host::gpio[GPIO_VG_VALVE_OFF]);
 for(int i=0;i<150 && !mode._suction.raised();++i) { step(mode); }
 assert(mode._suction.raised());
 send(system(2)); step(mode);
	// Disarmed reboot revokes all cleaner outputs before requesting restart.
	step(mode, false); send(system(3)); step(mode, false);
	assert(host::reboot_requested && !host::gpio[GPIO_VG_PUMP_OFF] && host::gpio[GPIO_VG_VALVE_OFF]);
	servos = host::latest<actuator_servos_s>("actuator_servos");
	for (float value : servos.control) { assert(!std::isfinite(value)); }
	puts("PASS: production mode registration, queued command priority, watchdog, arm edge, nav rejection/reset, suction turn/noise/completion/heading reset, optional wheel feedback/source latch/capture failure/standstill, tilt recovery, brush/battery, range-clear handshake, safe reboot");
}
