// SPDX-License-Identifier: BSD-3-Clause
// Commander registration shell for the native VGSolar external mode.
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/vgsolar_command.h>
#include <uORB/topics/vgsolar_event.h>
#include <uORB/topics/vgsolar_link_ack.h>
#include <uORB/topics/vgsolar_mode_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vgsolar_param_feedback.h>
#include "VgsolarModeLogic.hpp"
#include "VgsolarRobotLogic.hpp"
#include <lib/rover_control/VgsolarBattery.hpp>
#include <lib/rover_control/WheelSpeedFeedback.hpp>
#include <lib/geo/geo.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/vgsolar_encoder_status.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform/gpio.h>
#include <board_config.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vgsolar_inputs.h>
#include <uORB/topics/vgsolar_safety_request.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vgsolar_safety_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <mathlib/mathlib.h>
#include <cstring>

class VgsolarMode : public ModuleBase<VgsolarMode>, public px4::ScheduledWorkItem
{
public:
	VgsolarMode() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}

	static int task_spawn(int, char *[])
	{
		auto *instance = new VgsolarMode();
		if (!instance) { return PX4_ERROR; }
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleOnInterval(20000);
		return PX4_OK;
	}

	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }
		PRINT_MODULE_USAGE_NAME("vgsolar_mode", "mode");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("registered=%u nav_state=%u rejected=%u", (unsigned)(_mode_id >= 0), (unsigned)_mode_id,
			 static_cast<unsigned>(_rejected_commands));
		return 0;
	}

private:
	static float number(const char *name, float fallback) {
		float value = fallback;
		const param_t handle = param_find(name);
		if (handle != PARAM_INVALID) { param_get(handle, &value); }
		return PX4_ISFINITE(value) ? value : fallback;
	}
	static int32_t integer(const char *name, int32_t fallback) {
		int32_t value = fallback;
		const param_t handle = param_find(name);
		if (handle != PARAM_INVALID) { param_get(handle, &value); }
		return value;
	}
	void load_config() {
		_enabled = integer("VGS_ENABLE", 0) == 1;
		_wheel_feedback_enabled = integer("VGS_WHL_CTRL", 0) == 1;
		_wheel_radius_m = number("VGS_WHL_RAD", 0.f);
		_range_enabled = integer("VGS_SAFE_EN", 0) == 1;
		_cleaner_enabled = integer("VGS_CLN_EN", 0) == 1;
		_cruise_speed = math::constrain(number("VGS_CRUISE_SPD", 1.f), 0.1f, 5.f);
		_turn_timeout_s = math::constrain(number("VGS_TURN_TO", 15.f), 5.f, 60.f);
		_turn_speed = math::constrain(number("VGS_TURN_SPD", .3f), .1f, 1.f);
		_nav_default_radius = math::constrain(number("VGS_NAV_RAD", 2.f), .05f, 20.f);
		vgsolar_robot::Config config{};
		config.raised_pwm = number("VGS_SC_PWM_R", 1900.f);
		config.lowered_pwm = number("VGS_SC_PWM_L", 1100.f);
		config.slew_us_s = number("VGS_SC_RATE", 400.f);
		config.vacuum_kpa = number("VGS_SC_VAC_KPA", -70.f);
		config.vacuum_hysteresis_kpa = number("VGS_SC_VAC_HYST", 5.f);
		config.use_ir = integer("VGS_SC_IR", 1) == 1;
		config.use_pressure = integer("VGS_SC_PRESS", 1) == 1;
		config.lift_delay_ms = integer("VGS_SC_LIFT_DLY", 2000);
		config.vacuum_delay_ms = integer("VGS_SC_VAC_DLY", 10000);
		config.vent_delay_ms = integer("VGS_SC_VENT_DLY", 2000);
		config.action_timeout_ms = integer("VGS_SC_ACT_TO", 30000);
		config.lift_timeout_ms = integer("VGS_SC_LIFT_TO", 5000);
		config.vacuum_debounce_ms = integer("VGS_SC_VAC_DEB", 300);
		_suction.configure(config);
	}
	void stop_motion() {
		_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
		_last_motion_command = 0;
	}
	void set_navigation_state(uint8_t state) {
		_nav_state = state;
		_nav_event_timestamp = hrt_absolute_time();
	}
	void abort_task() {
  if (_logic.state() == vgsolar_mode::State::Turn) { turn_result(false, 6); }
		if (_logic.state() == vgsolar_mode::State::Navigation) { set_navigation_state(4); }
		_logic.disarm();
		_turn_phase = vgsolar_mode::TurnPhase::Idle;
		stop_motion();
	}
	void publish_cleaner(bool armed) {
		const bool allowed = armed && _enabled && _cleaner_enabled && _mode_active
				     && !_suction.fault();
		const bool brush_allowed = allowed && _logic.state() != vgsolar_mode::State::Estop && !_safety_hold && !low_battery();
		// This module owns Servo 1=lift, Servo 2=front brush, Servo 3=rear brush.
		// PWM function mapping and endpoints must be configured separately.
		actuator_servos_s output{};
		output.timestamp = hrt_absolute_time();
		for (float &value : output.control) { value = NAN; }
		if (allowed) {
			output.control[0] = math::constrain((_suction.pwm() - 1500.f) / 500.f, -1.f, 1.f);
			const float power = _brush_params.power_percent * .02f - 1.f;
			output.control[1] = brush_allowed && _brush_params.front ? power : -1.f;
			output.control[2] = brush_allowed && _brush_params.rear ? power : -1.f;
		}
		_servo_pub.publish(output);
		px4_arch_gpiowrite(GPIO_VG_VALVE_OFF, !(allowed && _logic.state() != vgsolar_mode::State::Estop && _suction.sealed()));
		px4_arch_gpiowrite(GPIO_VG_PUMP_OFF, allowed && _logic.state() != vgsolar_mode::State::Estop && _suction.pump());
	}
	void register_mode()
	{
		register_ext_component_request_s request{};
		request.timestamp = hrt_absolute_time();
		request.request_id = REQUEST_ID;
		request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
		strncpy(request.name, "VGSolar", sizeof(request.name) - 1);
		request.register_mode = true;
		request.register_arming_check = true;
		_register_pub.publish(request);
		_request_sent = true;
	}

	void unregister_mode()
	{
		if (_mode_id < 0) { return; }
		unregister_ext_component_s request{};
		request.timestamp = hrt_absolute_time();
		strncpy(request.name, "VGSolar", sizeof(request.name) - 1);
		request.mode_id = _mode_id;
		request.arming_check_id = _arming_check_id;
		request.mode_executor_id = -1;
		_unregister_pub.publish(request);
	}

	void configure_mode()
	{
		vehicle_control_mode_s config{};
		config.timestamp = hrt_absolute_time();
		config.source_id = _mode_id;
		config.flag_control_auto_enabled = true;
		config.flag_control_velocity_enabled = true;
		config.flag_control_attitude_enabled = true;
		config.flag_control_rates_enabled = true;
		config.flag_control_allocation_enabled = true;
		_config_pub.publish(config);
	}

	void ack(uint8_t command, bool success)
	{
		vgsolar_link_ack_s response{};
		response.timestamp = hrt_absolute_time();
		response.command = command;
		response.success = success;
		_ack_pub.publish(response);
	}

	bool low_battery()
	{
		battery_status_s battery{};
		const uint64_t now = hrt_absolute_time();
		if (!_battery_sub.copy(&battery) || battery.timestamp == 0 || now < battery.timestamp
		    || now - battery.timestamp > 5000000) { return false; }
		const float remaining = vgsolar::battery_remaining(battery.voltage_v, battery.remaining, battery.connected);
		return PX4_ISFINITE(remaining) && remaining <= 0.20f;
	}

	bool process_parameter_command(const vgsolar_command_s &command)
	{
		using namespace vgsolar_mode;
		if (command.command != vgsolar_command_s::CMD_PARAM_READ && command.command != vgsolar_command_s::CMD_PARAM_WRITE) {
			return false;
		}
		uint32_t value = 0;
		const bool write = command.command == vgsolar_command_s::CMD_PARAM_WRITE;
		const bool can_write = !write || command.param_value == 0
				       || (_mode_active && _armed && _enabled && _cleaner_enabled
					   && _logic.state() != State::Estop && !_safety_hold && !_suction.fault());
		const bool success = can_write && (write
			? write_brush_parameter(command.param_index, command.param_type, command.param_value, low_battery(), _brush_params, value)
			: read_brush_parameter(command.param_index, _brush_params, value));
		if (write) { ack(command.command, success); }
		if (success) {
			vgsolar_param_feedback_s feedback{};
			feedback.timestamp = hrt_absolute_time();
			feedback.param_index = command.param_index;
			feedback.param_type = 3;

			feedback.param_value = value;
			_param_feedback_pub.publish(feedback);
		} else {
			_rejected_commands++;
		}
		return true;
	}

	bool velocity_feedback(uint64_t now, const vehicle_local_position_s &local, float &motion_speed)
	{
		if (_wheel_feedback_enabled) {
			wheel_encoders_s wheel{};
			vgsolar_encoder_status_s capture{};
			if (!_wheel_sub.copy(&wheel) || !_wheel_capture_sub.copy(&capture)) { return false; }
			const auto feedback = rover_control::wheel_feedback(now, wheel.timestamp, wheel.wheel_speed[0],
					wheel.wheel_speed[1], capture.timestamp, capture.capture_active, _wheel_radius_m);
			motion_speed = feedback.maximum_track_speed_m_s;
			return feedback.valid;
		}
		motion_speed = hypotf(local.vx, local.vy);
		return local.timestamp > 0 && local.timestamp <= now && now - local.timestamp < 500000
		       && local.v_xy_valid && PX4_ISFINITE(local.vx) && PX4_ISFINITE(local.vy) && PX4_ISFINITE(motion_speed);
	}

	void observe_navigation_frame(uint64_t now)
	{
		vehicle_local_position_s local{};
		if (!_local_position_sub.copy(&local) || local.timestamp == 0 || local.timestamp > now
		    || now - local.timestamp >= 500000 || !local.xy_valid || !PX4_ISFINITE(local.x) || !PX4_ISFINITE(local.y)) { return; }
		if (_nav_frame_seen && (local.xy_reset_counter != _nav_frame_xy_reset || local.ref_timestamp != _nav_frame_reference)) {
			// The mode-entry NED origin belongs to the previous estimator frame.
			// Keep it invalid until mode re-entry; silently re-capturing changes the command meaning.
			_nav_origin_valid = false;
			_nav_origin_invalidated = true;
			if (_logic.state() == vgsolar_mode::State::Navigation) {
				set_navigation_state(3);
				_logic.command(vgsolar_mode::Command::CancelNavigation, _armed);
				stop_motion();
				_nav_yaw_align = false;
			}
		}
		_nav_frame_seen = true;
		_nav_frame_xy_reset = local.xy_reset_counter;
		_nav_frame_reference = local.ref_timestamp;
	}

	void reject_navigation(uint8_t requested_mode)
	{
		_rejected_commands++;
		_nav_mode = requested_mode;
		set_navigation_state(3);
		if (_logic.state() == vgsolar_mode::State::Navigation) {
			_logic.command(vgsolar_mode::Command::CancelNavigation, _armed);
			stop_motion();
			_nav_yaw_align = false;
		}
		ack(vgsolar_command_s::CMD_POSITION, false);
	}

 void turn_result(bool success, uint8_t reason) {
  if (_turn_result_sent) { return; }
  _turn_result_sent = true; event(5, _turn_command, success, reason);
 }
 void event(uint8_t kind, const vgsolar_command_s &command, bool accepted, uint8_t reason) {
  const int level = integer("VGS_CC_LOG", 1);
  if (level <= 0) { return; }
  const uint64_t now = hrt_absolute_time();
  if (kind == 1 && command.command == vgsolar_command_s::CMD_SPEED && level == 1) {
   if (_last_speed_event > 0 && now - _last_speed_event < 200000 && command.control_mode == _logged_speed.control_mode
       && command.velocity_cms == _logged_speed.velocity_cms && command.yaw_data == _logged_speed.yaw_data
       && reason == _logged_reason) { return; }
   _last_speed_event = now; _logged_speed = command; _logged_reason = reason;
  }
  vgsolar_event_s e{}; e.timestamp = now; e.kind = kind; e.command = command.command;
  e.state = static_cast<uint8_t>(_logic.state()); e.turn_phase = static_cast<uint8_t>(_turn_phase);
  e.previous_state = _logged_state; e.previous_phase = _logged_phase;
  e.accepted = accepted; e.reason = reason; e.velocity_cms = command.velocity_cms; e.yaw_data = command.yaw_data;
  e.turn_angle_cd = command.turn_angle_cd; e.turn_rate_cds = command.turn_rate_cds;
  e.turn_direction = command.turn_direction; e.turn_mode = command.turn_mode;
  _event_pub.publish(e);
 }
 void process_command(const vgsolar_command_s &command, bool armed) {
  const uint32_t before = _rejected_commands;
  const auto state = _logic.state();
  process_command_impl(command, armed);
  const bool accepted = before == _rejected_commands;
  const uint8_t reason = accepted ? 0 : !armed ? 1 : state == vgsolar_mode::State::Estop ? 2
    : !_suction.raised() ? 3 : _safety_hold ? 4 : state == vgsolar_mode::State::Navigation ? 5 : 6;
  event(1, command, accepted, reason);
 }
 void process_command_impl(const vgsolar_command_s &command, bool armed)
	{
		using namespace vgsolar_mode;
		if (process_parameter_command(command)) { return; }
		const uint64_t now = hrt_absolute_time();
		if (command.command == vgsolar_command_s::CMD_SYSTEM) {
			if (command.system_command == SYSTEM_ESTOP) {
				_pending_range_clear = false;
				enter_estop(0);
			} else if (command.system_command == SYSTEM_ESTOP_CLEAR) {
				if (_logic.state() != State::Estop) { _rejected_commands++; return; }
				if (!range_safe(now, true)) { _rejected_commands++; return; }
				if (_range_enabled && !range_safe(now)) {
					vgsolar_safety_request_s request{};
					request.timestamp = now;
					request.clear = true;
					_safety_request_pub.publish(request);
					_pending_range_clear = true;
					_clear_requested_at = now;
				} else {
					_logic.command(Command::ClearEstop, armed);
					_fault_code &= static_cast<uint16_t>(~(1u << 11));
				}
				stop_motion();
			} else if (command.system_command == 3 && !armed) {
				_suction.emergency_release(hrt_absolute_time());
				_brush_params = {};
				publish_cleaner(false);
				px4_reboot_request(REBOOT_REQUEST, 200000);
			} else {
				_rejected_commands++;
			}
			return;
		}

		if (command.command == vgsolar_command_s::CMD_SPEED) {
			vehicle_local_position_s velocity{};
			_local_position_sub.copy(&velocity);
			float motion_speed = NAN;
			const bool velocity_ok = velocity_feedback(now, velocity, motion_speed);
			const bool stop_requested = command.velocity_cms == 0 && (command.control_mode == SPEED_YAW || command.yaw_data == 0);
			const bool valid = armed && _enabled && (velocity_ok || stop_requested) && !_safety_hold && _suction.raised() && valid_speed(command.control_mode, command.velocity_cms, command.yaw_data)
					   && _logic.state() != State::Turn && _logic.state() != State::Navigation
					   && _logic.state() != State::Estop;
			if (!valid) { _rejected_commands++; return; }
			if (command.velocity_cms == 0 && (command.control_mode == SPEED_YAW || command.yaw_data == 0)) {
				_logic.disarm();
				_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
				_last_motion_command = 0;
				return;
			}
			const Transition transition = _logic.command(command.control_mode == SPEED_YAW ? Command::SpeedYaw : Command::SpeedYawRate, true);
			if (!transition.accepted) { _rejected_commands++; return; }
			_velocity_m_s = command.velocity_cms * 0.01f;
			if (command.control_mode == SPEED_YAW) {
				_yaw_rad = math::radians(command.yaw_data * 0.01f);
				_yaw_rate_rad_s = 0.f;
			} else {
				_yaw_rate_rad_s = protocol_yaw_rate(command.yaw_data);
				_yaw_rad = 0.f;
			}
			_last_motion_command = now;
			return;
		}

		if (command.command == vgsolar_command_s::CMD_POSITION) {
			if (_logic.state() == State::Turn) { _rejected_commands++; ack(command.command, false); return; }
			if (command.nav_mode == NAV_CANCEL) {
				if (_logic.state() == State::Turn) { ack(command.command, false); return; }
				_logic.disarm();
				_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
				_last_motion_command = 0;
				set_navigation_state(4);
				_nav_yaw_align = false;
				ack(command.command, true);
				return;
			}

			vehicle_local_position_s local{};
			float motion_speed = NAN;
			const bool local_ok = _local_position_sub.copy(&local) && local.timestamp != 0 && now >= local.timestamp
					       && now - local.timestamp <= 500000 && local.xy_valid && velocity_feedback(now, local, motion_speed)
						       && local.heading_good_for_control
					       && PX4_ISFINITE(local.heading) && PX4_ISFINITE(local.x) && PX4_ISFINITE(local.y);
			if (!armed || !_enabled || _safety_hold || !_suction.raised() || _logic.state() == State::Turn || !local_ok || command.nav_mode < 1 || command.nav_mode > 3
			    || command.arrival_radius_cm > 50000 || command.cruise_speed_cms < -500
			    || command.cruise_speed_cms > 500 || (command.arrival_yaw_cd != 0xffff && command.arrival_yaw_cd > 36000)) {
				reject_navigation(command.nav_mode);
				return;
			}

			if (command.nav_mode == 2 && _nav_origin_invalidated) { reject_navigation(command.nav_mode); return; }
			if (!_nav_origin_valid && !_nav_origin_invalidated) {
				_nav_origin_x = local.x;
				_nav_origin_y = local.y;
				_nav_origin_valid = true;
			}

			float target_north = 0.f;
			float target_east = 0.f;
			if (command.nav_mode == 1) {
				if (!local.xy_global || !PX4_ISFINITE(local.ref_lat) || !PX4_ISFINITE(local.ref_lon)
				    || fabs(local.ref_lat) > 90. || fabs(local.ref_lon) > 180.) {
					reject_navigation(command.nav_mode);
					return;
				}
				if (command.target_y < -900000000 || command.target_y > 900000000
				    || command.target_x < -1800000000 || command.target_x > 1800000000) {
					reject_navigation(command.nav_mode);
					return;
				}
				const double target_lon = static_cast<double>(command.target_x) * 1e-7;
				const double target_lat = static_cast<double>(command.target_y) * 1e-7;
				const MapProjection projection{local.ref_lat, local.ref_lon, local.ref_timestamp};
				projection.project(target_lat, target_lon, target_north, target_east);
			} else if (command.nav_mode == 2) {
				target_north = _nav_origin_x + command.target_x * 0.01f;
				target_east = _nav_origin_y + command.target_y * 0.01f;
			} else {
				const float forward = command.target_x * 0.01f;
				const float right = command.target_y * 0.01f;
				const float yaw = local.heading;
				target_north = local.x + forward * cosf(yaw) - right * sinf(yaw);
				target_east = local.y + forward * sinf(yaw) + right * cosf(yaw);
			}

			const Transition transition = _logic.command(Command::Navigation, true);
			if (!transition.accepted) {
				reject_navigation(command.nav_mode);
				return;
			}
			_nav_xy_reset = local.xy_reset_counter;
			_nav_heading_reset = local.heading_reset_counter;
			_nav_reference = local.ref_timestamp;
			_nav_mode = command.nav_mode;
			_nav_start_x = local.x; _nav_start_y = local.y;
			_nav_target_x = target_north;
			_nav_target_y = target_east;
			_nav_radius_m = command.arrival_radius_cm > 0 ? command.arrival_radius_cm * 0.01f : _nav_default_radius;
			_nav_speed_m_s = command.cruise_speed_cms != 0 ? command.cruise_speed_cms * 0.01f : _cruise_speed;
			_nav_arrival_yaw_valid = command.arrival_yaw_cd != 0xFFFF;
			_nav_arrival_yaw_cd = command.arrival_yaw_cd;
			_nav_yaw_align = false;
			set_navigation_state(1);
			_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
			_last_motion_command = 0;
			ack(command.command, true);
			return;
		}

		if (command.command == vgsolar_command_s::CMD_TURN) {
			const bool replaces_navigation = _logic.state() == State::Navigation;
			vehicle_local_position_s local{};
			if (!armed || !_enabled || !_cleaner_enabled || _safety_hold || !_suction.raised()
			    || !valid_turn(command.turn_mode, command.turn_direction, command.turn_angle_cd, command.turn_rate_cds)
			    || !_local_position_sub.copy(&local) || local.timestamp == 0 || now < local.timestamp
			    || now - local.timestamp > 500000 || !local.heading_good_for_control || !PX4_ISFINITE(local.heading)
			    || (_suction.config().use_ir && !fresh_inputs(now, true))
			    || (_suction.config().use_pressure && !fresh_inputs(now, false))
			    || !_logic.command(Command::Turn, true).accepted) {
				_rejected_commands++;
				return;
			}
			stop_motion();
			if (replaces_navigation) { set_navigation_state(4); _nav_yaw_align = false; }
			_turn_result_sent = false; _turn_command = command;
			_turn_phase = TurnPhase::Stopping;
			_turn_start = _turn_phase_start = now;
			_turn_target = math::radians(command.turn_angle_cd * .01f);
			_turn_rate = turn_rate_rad_s(command.turn_direction, command.turn_rate_cds);
			_turn_moving = command.turn_mode == 2;
			_turn_heading_reset = local.heading_reset_counter;
			return;
		}
		if (command.command == vgsolar_command_s::CMD_POSITION || command.command == vgsolar_command_s::CMD_PARAM_READ
		    || command.command == vgsolar_command_s::CMD_PARAM_WRITE) {
			_rejected_commands++;
			ack(command.command, false);
		}
	}

	void update_navigation(uint64_t now)
	{
		if (_logic.state() != vgsolar_mode::State::Navigation) { return; }
		vehicle_local_position_s local{};
		float motion_speed = NAN;
		if (!_local_position_sub.copy(&local) || local.timestamp == 0 || now < local.timestamp
		    || now - local.timestamp > 500000 || !local.xy_valid || !velocity_feedback(now, local, motion_speed)
		    || !local.heading_good_for_control
		    || !PX4_ISFINITE(local.heading) || !PX4_ISFINITE(local.x) || !PX4_ISFINITE(local.y)
		    || local.xy_reset_counter != _nav_xy_reset || local.heading_reset_counter != _nav_heading_reset
		    || local.ref_timestamp != _nav_reference) {
			set_navigation_state(3);
			_logic.command(vgsolar_mode::Command::CancelNavigation, true);
			_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
			return;
		}

		const auto solution = vgsolar_mode::path_guidance(local.x, local.y, _nav_start_x, _nav_start_y, _nav_target_x, _nav_target_y, fmaxf(.5f, fabsf(_nav_speed_m_s)));
		if (!solution.valid) {
			set_navigation_state(3);
			_logic.command(vgsolar_mode::Command::CancelNavigation, true);
			_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
			return;
		}
		_nav_distance_m = solution.distance_m;
		const float desired_heading = _nav_speed_m_s < 0.f
					      ? vgsolar_mode::wrap_pi(solution.bearing_rad + static_cast<float>(M_PI)) : solution.bearing_rad;
		_nav_heading_error_rad = vgsolar_mode::wrap_pi(local.heading - desired_heading);

		if (!_nav_yaw_align && solution.distance_m <= _nav_radius_m) {
			_velocity_m_s = 0.f;
			if (_nav_arrival_yaw_valid) {
				const float yaw = math::radians(_nav_arrival_yaw_cd * 0.01f);
				_nav_yaw_target_rad = _nav_mode == 3 ? vgsolar_mode::wrap_pi(local.heading + yaw) : yaw;
				_nav_yaw_align = true;
			} else {
				set_navigation_state(2);
				_logic.command(vgsolar_mode::Command::CancelNavigation, true);
			}
		}

		if (_nav_yaw_align) {
			_velocity_m_s = 0.f;
			_yaw_rad = _nav_yaw_target_rad;
			_nav_heading_error_rad = vgsolar_mode::wrap_pi(local.heading - _nav_yaw_target_rad);
			if (fabsf(vgsolar_mode::wrap_pi(local.heading - _nav_yaw_target_rad)) <= math::radians(2.f)) {
				set_navigation_state(2);
				_nav_yaw_align = false;
				_logic.command(vgsolar_mode::Command::CancelNavigation, true);
				_velocity_m_s = 0.f;
			}
			return;
		}
		if (_logic.state() != vgsolar_mode::State::Navigation) { return; }
		_yaw_rad = desired_heading;
		const float braking = sqrtf(2.f * math::constrain(number("VGS_NAV_DEC", .5f), .05f, 3.f)
      * fmaxf(0.f, solution.distance_m - _nav_radius_m));
  const float alignment = fmaxf(0.f, cosf(vgsolar_mode::wrap_pi(local.heading - desired_heading)));
  _velocity_m_s = copysignf(fminf(fabsf(_nav_speed_m_s), braking) * alignment, _nav_speed_m_s);
	}



	bool fresh_inputs(uint64_t now, bool infrared) {
		vgsolar_inputs_s input{};
		return _inputs_sub.copy(&input) && input.enabled && input.timestamp > 0 && now >= input.timestamp
		       && now - input.timestamp < 500000 && (infrared ? input.ir_sample_valid : input.pressure_valid);
	}
	bool range_safe(uint64_t now, bool allow_clear = false)
	{
		if (!_range_enabled) { return true; }
		vgsolar_safety_status_s safety{};
		return _safety_sub.copy(&safety) && safety.enabled && safety.config_valid
		       && safety.timestamp > 0 && now >= safety.timestamp && now - safety.timestamp < 500000
		       && safety.left_valid && safety.right_valid
		       && (allow_clear ? safety.clear_eligible || !safety.stop_request : !safety.stop_request && !safety.latched);
	}
	void enter_estop(uint16_t faults)
	{
		_fault_code |= faults;
  if (_logic.state() == vgsolar_mode::State::Estop) { return; }
		if (_logic.state() == vgsolar_mode::State::Navigation) { set_navigation_state(4); }
		if (_logic.state() == vgsolar_mode::State::Turn) { turn_result(false, 2); }
		_logic.command(vgsolar_mode::Command::Estop, _armed);
		_turn_phase = vgsolar_mode::TurnPhase::Idle;
		_safety_hold = false;
		_brush_params = {};
		stop_motion();
		_suction.emergency_release(hrt_absolute_time());
	}
	void service_release(uint64_t now) {
  vgsolar_inputs_s report{}; vgsolar_robot::Inputs input{};
  if (_inputs_sub.copy(&report) && report.enabled && report.timestamp > 0 && report.timestamp <= now && now - report.timestamp < 500000) {
   input.fresh = true; input.ir_valid = report.ir_sample_valid; input.blade = report.ir_blade_present;
   input.pressure_valid = report.pressure_valid; input.pressure_kpa = report.pressure_kpa;
  }
  _suction.tick(now, true, input);
 }
 void update_robot(uint64_t now)
	{
		if (!range_safe(now)) {
			enter_estop(1u << 11);
		}
		if (_pending_range_clear && now - _clear_requested_at > 500000) { _pending_range_clear = false; }
		if (_pending_range_clear && range_safe(now)) {
			_pending_range_clear = false;
			_logic.command(vgsolar_mode::Command::ClearEstop, _armed);
			_fault_code &= static_cast<uint16_t>(~(1u << 11));
		}
		if (_logic.state() == vgsolar_mode::State::Estop) { service_release(now); return; }
		if (low_battery()) { _brush_params = {}; }
		vgsolar_robot::Inputs input{};
		vgsolar_inputs_s report{};
		if (_inputs_sub.copy(&report) && report.enabled && report.timestamp > 0 && now >= report.timestamp
		    && now - report.timestamp < 500000) {
			input.fresh = true;
			input.ir_valid = report.ir_sample_valid;
			input.blade = report.ir_blade_present;
			input.pressure_valid = report.pressure_valid;
			input.pressure_kpa = report.pressure_kpa;
		}
		if (_suction.lowered() || _suction.busy() || _suction.frozen()) {
			vehicle_attitude_s attitude{};
			bool tilt_bad = true;
			if (_attitude_sub.copy(&attitude) && attitude.timestamp > 0 && now >= attitude.timestamp
			    && now - attitude.timestamp < 500000) {
				const matrix::Eulerf euler{matrix::Quatf{attitude.q}};
				tilt_bad = !PX4_ISFINITE(euler.phi()) || !PX4_ISFINITE(euler.theta())
					   || fabsf(euler.phi()) > math::radians(30.f) || fabsf(euler.theta()) > math::radians(30.f);
			}
			if (tilt_bad) {
				_fault_code |= 1u << 9;
				_safety_hold = true;
				_suction.freeze();
				_brush_params = {};
				stop_motion();
			} else if (_safety_hold) {
				abort_task();
				_suction.unfreeze();
				if (_suction.lowered()) { _suction.raise(now); }
				_safety_hold = false;
				_fault_code &= static_cast<uint16_t>(~(1u << 9));
			}
		}
		const bool actuator_permission = _enabled && _cleaner_enabled && _mode_active && _armed;
		if (!actuator_permission && _logic.state() == vgsolar_mode::State::Turn) { abort_task(); }
		_suction.tick(now, actuator_permission, input);
		if (_suction.fault()) {
			_fault_code |= 1u << 4;
			abort_task();
			_brush_params = {};
		}
	}

	void update_turn(uint64_t now)
	{
		if (_logic.state() != vgsolar_mode::State::Turn) { return; }
		vehicle_local_position_s local{};
		float motion_speed = NAN;
		const bool local_ok = _local_position_sub.copy(&local) && local.timestamp > 0 && now >= local.timestamp
				      && now - local.timestamp < 500000 && velocity_feedback(now, local, motion_speed)
				      && local.heading_good_for_control && PX4_ISFINITE(local.heading)
				      && local.heading_reset_counter == _turn_heading_reset;
		if (_turn_phase != vgsolar_mode::TurnPhase::RaiseSuction && (!local_ok || (_turn_phase == vgsolar_mode::TurnPhase::Turning && now - _turn_phase_start > static_cast<uint64_t>(_turn_timeout_s * 1000000.f)))) {
			turn_result(false, !local_ok ? 9 : 8);
			_turn_phase = vgsolar_mode::TurnPhase::RaiseSuction;
			stop_motion();
		}
		switch (_turn_phase) {
		case vgsolar_mode::TurnPhase::Stopping:
			stop_motion();
			if (local_ok && motion_speed < .05f) {
				_turn_phase = vgsolar_mode::TurnPhase::WaitStopped;
				_turn_phase_start = now;
			}
			break;
		case vgsolar_mode::TurnPhase::WaitStopped:
			stop_motion();
			if (!local_ok || motion_speed >= .05f) {
				_turn_phase = vgsolar_mode::TurnPhase::Stopping;
			} else if (now - _turn_phase_start >= 500000) {
				_turn_phase = vgsolar_mode::TurnPhase::LowerSuction;
				_turn_phase_start = now;
			}
			break;
		case vgsolar_mode::TurnPhase::LowerSuction:
			if (_suction.raised() && !_suction.lower(now)) { _fault_code |= 1u << 4; abort_task(); _suction.emergency_release(hrt_absolute_time()); return; }
			if (_suction.lowered()) {
				_turn_phase = vgsolar_mode::TurnPhase::Turning;
				_turn_phase_start = now;
				_turn_progress.start(local.heading, _turn_rate);
			}
			break;
		case vgsolar_mode::TurnPhase::Turning:
			if (!local_ok || _suction.fault()) { _fault_code |= 1u << 4; _turn_phase = vgsolar_mode::TurnPhase::RaiseSuction; stop_motion(); break; }
			{
				const bool progress_valid = _turn_progress.update(local.heading);
				if (!progress_valid || _turn_progress.reached(_turn_target)) {
					_turn_phase = vgsolar_mode::TurnPhase::RaiseSuction;
					_turn_phase_start = now;
					stop_motion();
				} else {
					_velocity_m_s = _turn_moving ? _turn_speed : 0.f;
					_yaw_rate_rad_s = _turn_rate;
				}
			}
			break;
		case vgsolar_mode::TurnPhase::RaiseSuction:
			stop_motion();
			if (_suction.raised()) {
				turn_result(true, 0);
				_logic.disarm();
				_turn_phase = vgsolar_mode::TurnPhase::Idle;
			} else if (_suction.state() != vgsolar_robot::SuctionState::Venting
				   && _suction.state() != vgsolar_robot::SuctionState::Raising
				   && _suction.state() != vgsolar_robot::SuctionState::WaitUp) {
				if (!_suction.raise(now)) { _fault_code |= 1u << 4; abort_task(); _suction.emergency_release(hrt_absolute_time()); }
			}
			break;
		default:
			break;
		}
	}

	void update_vehicle_mode()
	{
		vehicle_status_s status{};
		if (!_vehicle_status_sub.copy(&status) || _mode_id < 0) { return; }
		if (status.timestamp == 0 || hrt_absolute_time() < status.timestamp || hrt_absolute_time() - status.timestamp > 2000000) {
			abort_task(); _suction.emergency_release(hrt_absolute_time()); _armed = false; publish_cleaner(false); publish_mode_status(false); return;
		}
		if (status.nav_state != _mode_id || !_enabled) {
			if (_mode_active) { _brush_params = {}; abort_task(); }
			_suction.emergency_release(hrt_absolute_time());
			_safety_hold = false;
			_armed = false;
			publish_cleaner(false);
			_logic.disarm();
			_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
			_last_motion_command = 0;
			_mode_active = false;
			vgsolar_command_s command{};
			while (_command_sub.update(&command)) {
				if (command.timestamp == 0 || command.timestamp > hrt_absolute_time()
				    || hrt_absolute_time() - command.timestamp > COMMAND_STALE_US) { continue; }
				process_parameter_command(command);
			}
			publish_mode_status(false);
			return;
		}
		const bool mode_entry = !_mode_active;
		if (!_mode_active) {
			_brush_params = {};
			if (_suction.fault()) { _suction.clear_fault(); }
			_fault_code = 0;
			_nav_state = 0;
			_nav_origin_valid = false;
			_nav_origin_invalidated = false;
			_nav_frame_seen = false;
			vehicle_local_position_s local{};
			const uint64_t entered_at = hrt_absolute_time();
			if (_local_position_sub.copy(&local) && local.timestamp > 0 && entered_at >= local.timestamp
			    && entered_at - local.timestamp < 500000 && local.xy_valid && PX4_ISFINITE(local.x) && PX4_ISFINITE(local.y)) {
				_nav_origin_x = local.x;
				_nav_origin_y = local.y;
				_nav_origin_valid = true;
			}
			_mode_active = true;
		}
		actuator_armed_s actuator{};
		const uint64_t arm_now = hrt_absolute_time();
		const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
				   && (!status.safety_button_available || status.safety_off)
				   && _actuator_armed_sub.copy(&actuator) && actuator.armed && !actuator.lockdown
				   && !actuator.kill && !actuator.termination && !actuator.in_esc_calibration_mode
				   && actuator.timestamp > 0 && arm_now >= actuator.timestamp && arm_now - actuator.timestamp < 2000000;
		const bool arm_edge = armed && !_armed;
		_armed = armed;
		observe_navigation_frame(arm_now);
		vgsolar_command_s command{}, system{}, turn{}, nav{}, speed_command{};
		bool have_system = false, have_turn = false, have_nav = false, have_speed = false;
		for (unsigned count = 0; count < 32 && _command_sub.update(&command); ++count) {
			if (command.timestamp == 0 || command.timestamp > arm_now || arm_now - command.timestamp > COMMAND_STALE_US) { continue; }
			if (command.command == vgsolar_command_s::CMD_SYSTEM) { system = command; have_system = true; }
			else if (command.command == vgsolar_command_s::CMD_PARAM_WRITE || command.command == vgsolar_command_s::CMD_PARAM_READ) {
				process_parameter_command(command);
			} else if (!arm_edge && !mode_entry) {
				if (command.command == vgsolar_command_s::CMD_TURN) { turn = command; have_turn = true; }
				else if (command.command == vgsolar_command_s::CMD_POSITION) { nav = command; have_nav = true; }
				else if (command.command == vgsolar_command_s::CMD_SPEED) { speed_command = command; have_speed = true; }
			}
		}
		// Same-cycle priority matches APM: system > turn > navigation > speed_command.
		if (have_system) { process_command(system, armed); }
		else if (have_turn) { process_command(turn, armed); }
		else if (have_nav) { process_command(nav, armed); }
		else if (have_speed) { process_command(speed_command, armed); }
		if (!armed) {
			abort_task();
			_suction.emergency_release(hrt_absolute_time());
			_safety_hold = false;
			publish_cleaner(false);
			_logic.disarm();
			_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
			_last_motion_command = 0;
			publish_mode_status(true);
			_brush_params = {};
			return;
		}

		const uint64_t now = hrt_absolute_time();
		if (_last_motion_command != 0 && now >= _last_motion_command && now - _last_motion_command >= MOTION_TIMEOUT_US) {
			event(3, vgsolar_command_s{}, false, 6);
			_logic.timeout();
			_velocity_m_s = _yaw_rad = _yaw_rate_rad_s = 0.f;
			_last_motion_command = 0;
			_brush_params = {};
		}

		update_robot(now);
		if (_logic.state() == vgsolar_mode::State::Yaw || _logic.state() == vgsolar_mode::State::YawRate) {
			vehicle_local_position_s local{};
			_local_position_sub.copy(&local);
			float motion_speed = NAN;
			const bool heading_ok = local.timestamp > 0 && now >= local.timestamp && now - local.timestamp < 500000
						&& local.heading_good_for_control && PX4_ISFINITE(local.heading);
			if (!velocity_feedback(now, local, motion_speed)
			    || (_logic.state() == vgsolar_mode::State::Yaw && !heading_ok)) {
				abort_task();
			}
		}
		if (!_safety_hold) {
			update_turn(now);
			update_navigation(now);
		}
		if (_safety_hold) { stop_motion(); }
		if (_safety_hold && _logic.state() != vgsolar_mode::State::Estop) {
			rover_attitude_setpoint_s attitude{};
			attitude.timestamp = now;
			attitude.yaw_setpoint = NAN;
			_attitude_pub.publish(attitude);
		}
		publish_cleaner(armed);
		if (!_safety_hold && (_logic.state() == vgsolar_mode::State::Yaw || _logic.state() == vgsolar_mode::State::YawRate
		    || _logic.state() == vgsolar_mode::State::Navigation || _logic.state() == vgsolar_mode::State::Turn)) {
			rover_speed_setpoint_s speed{};
			speed.timestamp = now;
			speed.speed_body_x = _velocity_m_s;
			speed.speed_body_y = NAN;
			_speed_pub.publish(speed);
			if (_logic.state() == vgsolar_mode::State::Yaw || _logic.state() == vgsolar_mode::State::Navigation) {
				rover_attitude_setpoint_s attitude{};
				attitude.timestamp = now;
				attitude.yaw_setpoint = _yaw_rad;
				_attitude_pub.publish(attitude);
			} else {
				rover_attitude_setpoint_s attitude{};
				attitude.timestamp = now;
				attitude.yaw_setpoint = NAN;
				_attitude_pub.publish(attitude);
				rover_rate_setpoint_s rate{};
				rate.timestamp = now;
				rate.yaw_rate_setpoint = _yaw_rate_rad_s;
				_rate_pub.publish(rate);
			}
		} else {
			// The differential speed controller retains its last setpoint, so publish zero while this mode owns control.
			rover_speed_setpoint_s speed{};
			speed.timestamp = now;
			speed.speed_body_x = 0.f;
			speed.speed_body_y = NAN;
			_speed_pub.publish(speed);
			rover_attitude_setpoint_s attitude{};
			attitude.timestamp = now;
			attitude.yaw_setpoint = NAN;
			_attitude_pub.publish(attitude);
			rover_rate_setpoint_s rate{};
			rate.timestamp = now;
			rate.yaw_rate_setpoint = 0.f;
			_rate_pub.publish(rate);
		}
		publish_mode_status(true);
	}

	void publish_mode_status(bool active)
	{
		vgsolar_mode_status_s report{};
		report.timestamp = hrt_absolute_time();
  if (_logged_state != static_cast<uint8_t>(_logic.state()) || _logged_phase != static_cast<uint8_t>(_turn_phase) || _logged_active != active) {
   event(2, vgsolar_command_s{}, true, 0);
   _logged_state = static_cast<uint8_t>(_logic.state()); _logged_phase = static_cast<uint8_t>(_turn_phase); _logged_active = active;
  }
  if (_logged_vacuum_warning != _suction.vacuum_warning()) {
   event(4, vgsolar_command_s{}, !_suction.vacuum_warning(), 0); _logged_vacuum_warning = _suction.vacuum_warning();
  }
		report.state = static_cast<uint8_t>(_logic.state());
		report.mode_id = _mode_id < 0 ? 255 : static_cast<uint8_t>(_mode_id);
		report.motion_allowed = active && _armed && !_safety_hold
					&& (_logic.state() == vgsolar_mode::State::Yaw || _logic.state() == vgsolar_mode::State::YawRate
					    || _logic.state() == vgsolar_mode::State::Navigation
					    || (_logic.state() == vgsolar_mode::State::Turn && _turn_phase == vgsolar_mode::TurnPhase::Turning));
		report.suction_state = static_cast<uint8_t>(_suction.state());
		report.suction_frozen = _suction.frozen();
		report.vacuum_warning = _suction.vacuum_warning();
  report.cleaner_output_permitted = active && _armed && _enabled && _cleaner_enabled && !_suction.fault();
  report.release_pending = _suction.state() == vgsolar_robot::SuctionState::Venting || _suction.state() == vgsolar_robot::SuctionState::Raising || _suction.state() == vgsolar_robot::SuctionState::WaitUp;
		report.velocity_m_s = _velocity_m_s;
		report.active = active;
		report.turn_phase = static_cast<uint8_t>(_turn_phase);
		report.turning_motion = _logic.state() == vgsolar_mode::State::Turn && _suction.lowered() && !_safety_hold;
		report.fault_code = _fault_code | (_nav_state == 3 ? 1u << 10 : 0);
		report.navigation_state = _nav_state;
		report.navigation_event_timestamp = _nav_event_timestamp;
		report.nav_mode = _nav_mode;
		report.nav_distance_cm = vgsolar_mode::navigation_distance_cm(_nav_distance_m);
		report.nav_heading_error_cd = static_cast<int16_t>(fminf(fmaxf(math::degrees(_nav_heading_error_rad) * 100.f, -32768.f), 32767.f));
		_mode_status_pub.publish(report);
	}

	void handle_registration_replies()
	{
		register_ext_component_reply_s reply{};
		while (_reply_sub.update(&reply)) {
			if (reply.request_id == REQUEST_ID && reply.success) {
				_mode_id = reply.mode_id;
				_arming_check_id = reply.arming_check_id;
				configure_mode();
				PX4_INFO("Commander registered VGSolar external mode (nav_state=%d)", (int)_mode_id);
			}
		}
	}

	void answer_arming_checks()
	{
		arming_check_request_s request{};
		while (_arming_request_sub.update(&request)) {
			if (_arming_check_id < 0 || !(request.valid_registrations_mask & (1u << _arming_check_id))) { continue; }
			arming_check_reply_s reply{};
			reply.timestamp = hrt_absolute_time();
			reply.request_id = request.request_id;
			reply.registration_id = _arming_check_id;
			reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
			reply.can_arm_and_run = _enabled && (!_cleaner_enabled || _suction.config().valid());
			_arming_reply_pub.publish(reply);
		}
	}

	void Run() override
	{
		if (should_exit()) {
			_armed = false;
			abort_task();
			publish_mode_status(false);
			_suction.emergency_release(hrt_absolute_time());
			_brush_params = {};
			publish_cleaner(false);
			unregister_mode();
			ScheduleClear();
			exit_and_cleanup();
			return;
		}
		if (!_request_sent || (_mode_id < 0 && hrt_absolute_time() - _last_registration > 1000000)) {
			load_config(); register_mode(); _last_registration = hrt_absolute_time(); return;
		}
		if (hrt_absolute_time() - _last_config >= 1000000) {
			// Live hardware configuration changes require a disarmed mode re-entry.
			vehicle_status_s vehicle{};
			const bool disarmed = _vehicle_status_sub.copy(&vehicle) && vehicle.arming_state != vehicle_status_s::ARMING_STATE_ARMED;
			if (disarmed) { load_config(); }
			else {
				_enabled = _enabled && integer("VGS_ENABLE", 0) == 1;
				_cleaner_enabled = _cleaner_enabled && integer("VGS_CLN_EN", 0) == 1;
			}
			_range_enabled = integer("VGS_SAFE_EN", 0) == 1;
			_last_config = hrt_absolute_time();
		}
		handle_registration_replies();
		answer_arming_checks();
		update_vehicle_mode();
	}

	uORB::Publication<vgsolar_event_s> _event_pub{ORB_ID(vgsolar_event)};
 bool _turn_result_sent{true}; vgsolar_command_s _turn_command{};
 uint8_t _logged_state{255}, _logged_phase{255}, _logged_reason{0};
 bool _logged_active{false}, _logged_vacuum_warning{false};
 uint64_t _last_speed_event{0}; vgsolar_command_s _logged_speed{};
 bool _enabled{false}, _cleaner_enabled{false}, _armed{false}, _safety_hold{false};
	bool _wheel_feedback_enabled{false};
	float _wheel_radius_m{0.f};
	uORB::SubscriptionData<wheel_encoders_s> _wheel_sub{ORB_ID(wheel_encoders)};
	uORB::SubscriptionData<vgsolar_encoder_status_s> _wheel_capture_sub{ORB_ID(vgsolar_encoder_status)};
	bool _range_enabled{false}, _pending_range_clear{false};
	float _cruise_speed{1.f}, _turn_timeout_s{15.f}, _turn_speed{.3f}, _nav_default_radius{2.f};
	vgsolar_robot::Suction _suction{};
	vgsolar_mode::TurnPhase _turn_phase{vgsolar_mode::TurnPhase::Idle};
	uint64_t _turn_start{0}, _turn_phase_start{0}, _last_config{0}, _last_registration{0}, _clear_requested_at{0};
	float _turn_target{0.f}, _turn_rate{0.f};
	vgsolar_mode::TurnProgress _turn_progress{};
	bool _turn_moving{false};
	uint16_t _fault_code{0};
	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::SubscriptionData<vgsolar_inputs_s> _inputs_sub{ORB_ID(vgsolar_inputs)};
	uORB::SubscriptionData<vgsolar_safety_status_s> _safety_sub{ORB_ID(vgsolar_safety_status)};
	uORB::SubscriptionData<vehicle_attitude_s> _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Publication<actuator_servos_s> _servo_pub{ORB_ID(actuator_servos)};
	uORB::Publication<vgsolar_safety_request_s> _safety_request_pub{ORB_ID(vgsolar_safety_request)};
	static constexpr uint64_t REQUEST_ID = 0x5647534f4c4152ULL;
	bool _request_sent{false};
	int8_t _mode_id{-1};
	int8_t _arming_check_id{-1};
	static constexpr uint64_t COMMAND_STALE_US = 200000;
	static constexpr uint64_t MOTION_TIMEOUT_US = 500000;
	static constexpr uint8_t SYSTEM_ESTOP = 1;
	static constexpr uint8_t SYSTEM_ESTOP_CLEAR = 2;
	static constexpr uint8_t SPEED_YAW = 1;
	static constexpr uint8_t SPEED_YAWRATE = 2;
	static constexpr uint8_t NAV_CANCEL = 4;
	uint64_t _last_motion_command{0};
	uint32_t _rejected_commands{0};
	bool _mode_active{false};
	float _velocity_m_s{0.f}, _yaw_rad{0.f}, _yaw_rate_rad_s{0.f};
	vgsolar_mode::BrushParameters _brush_params{};
	vgsolar_mode::Logic _logic{};
	bool _nav_origin_valid{false}, _nav_origin_invalidated{false}, _nav_frame_seen{false};
	uint8_t _nav_frame_xy_reset{0};
	uint64_t _nav_frame_reference{0}, _nav_reference{0}, _nav_event_timestamp{0};
	float _nav_origin_x{0.f}, _nav_origin_y{0.f};
	float _nav_start_x{0.f}, _nav_start_y{0.f};
	float _nav_target_x{0.f}, _nav_target_y{0.f};
	float _nav_radius_m{2.f}, _nav_speed_m_s{0.f}, _nav_distance_m{0.f};
	float _nav_heading_error_rad{0.f}, _nav_yaw_target_rad{0.f};
	uint8_t _nav_mode{0}, _nav_state{0}, _nav_xy_reset{0}, _nav_heading_reset{0}, _turn_heading_reset{0};
	bool _nav_arrival_yaw_valid{false}, _nav_yaw_align{false};
	uint16_t _nav_arrival_yaw_cd{0};
	uORB::Publication<register_ext_component_request_s> _register_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> _config_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> _arming_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Subscription _reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<vehicle_local_position_s> _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _command_sub{ORB_ID(vgsolar_command)};
	uORB::SubscriptionData<battery_status_s> _battery_sub{ORB_ID(battery_status)};
	uORB::Publication<vgsolar_link_ack_s> _ack_pub{ORB_ID(vgsolar_link_ack)};
	uORB::Publication<vgsolar_param_feedback_s> _param_feedback_pub{ORB_ID(vgsolar_param_feedback)};
	uORB::Publication<rover_speed_setpoint_s> _speed_pub{ORB_ID(rover_speed_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s> _attitude_pub{ORB_ID(rover_attitude_setpoint)};
	uORB::Publication<rover_rate_setpoint_s> _rate_pub{ORB_ID(rover_rate_setpoint)};
	uORB::Publication<vgsolar_mode_status_s> _mode_status_pub{ORB_ID(vgsolar_mode_status)};
};

extern "C" __EXPORT int vgsolar_mode_main(int argc, char *argv[])
{
	return VgsolarMode::main(argc, argv);
}
