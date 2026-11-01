// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <stdint.h>
#include <cmath>

namespace vgsolar_mode
{
enum class State : uint8_t { Standby, Yaw, YawRate, Turn, Navigation, Estop };
enum class TurnPhase : uint8_t { Idle, Stopping, WaitStopped, LowerSuction, Turning, RaiseSuction };
enum class NavigationState : uint8_t { Idle, Active, Arrived, Failed, Cancelled };

struct NavigationSolution {
	float distance_m{NAN};
	float bearing_rad{NAN};
	bool valid{false};
};

inline float wrap_pi(float angle)
{
	return std::isfinite(angle) ? remainderf(angle, 2.f * static_cast<float>(M_PI)) : NAN;
}

// Accumulate directed displacement, not total travelled angle: oscillation and
// motion opposite to the request must not complete a turn.
class TurnProgress
{
public:
	bool start(float heading, float commanded_rate) {
		_valid = std::isfinite(heading) && std::isfinite(commanded_rate) && fabsf(commanded_rate) > 0.f;
		_heading = heading;
		_direction = commanded_rate < 0.f ? -1.f : 1.f;
		_angle = 0.f;
		return _valid;
	}
	bool update(float heading) {
		if (!_valid || !std::isfinite(heading)) { _valid = false; return false; }
		_angle += _direction * wrap_pi(heading - _heading);
		_heading = heading;
		return true;
	}
	bool reached(float target) const {
		return _valid && std::isfinite(target) && target > 0.f && _angle >= target;
	}
	float angle() const { return _angle; }
private:
	float _heading{0.f}, _direction{1.f}, _angle{0.f};
	bool _valid{false};
};

inline NavigationSolution solve_navigation(float current_north_m, float current_east_m,
						   float target_north_m, float target_east_m)
{
	NavigationSolution solution{};
	if (!std::isfinite(current_north_m) || !std::isfinite(current_east_m) ||
	    !std::isfinite(target_north_m) || !std::isfinite(target_east_m)) { return solution; }
	const float north = target_north_m - current_north_m;
	const float east = target_east_m - current_east_m;
	solution.distance_m = hypotf(north, east);
	solution.bearing_rad = atan2f(east, north);
	solution.valid = std::isfinite(solution.distance_m) && std::isfinite(solution.bearing_rad);
	return solution;
}

inline NavigationSolution path_guidance(float north, float east, float start_n, float start_e,
 float goal_n, float goal_e, float lookahead) {
 const auto goal = solve_navigation(north, east, goal_n, goal_e);
 const float dn = goal_n - start_n, de = goal_e - start_e;
 const float length = hypotf(dn, de);
 if (!goal.valid || !std::isfinite(length) || length <= .001f) { return goal; }
 const float along = ((north-start_n)*dn + (east-start_e)*de) / length;
 const float ahead = fminf(length, fmaxf(0.f, along) + fmaxf(.1f, lookahead));
 auto guidance = solve_navigation(north, east, start_n + dn * ahead/length, start_e + de * ahead/length);
 guidance.distance_m = goal.distance_m;
 return guidance;
}
inline uint32_t navigation_distance_cm(float distance_m)
{
	if (!std::isfinite(distance_m) || distance_m <= 0.f) { return 0; }
	const double cm = static_cast<double>(distance_m) * 100.;
	return cm >= UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(cm);
}

inline bool valid_turn(uint8_t mode, uint8_t direction, uint16_t angle_cd, uint16_t rate_cds)
{
	return (mode == 1 || mode == 2) && (direction == 1 || direction == 2)
	       && angle_cd > 0 && angle_cd <= 36000 && rate_cds <= 18000;
}

inline float turn_rate_rad_s(uint8_t direction, uint16_t rate_cds)
{
	const float rate = static_cast<float>(rate_cds < 100 ? 100 : rate_cds) * 0.01f * static_cast<float>(M_PI) / 180.f;
	return direction == 1 ? -rate : rate;
}

inline float protocol_yaw_rate(int16_t centidegrees_s)
{
	// NCU positive means left, whereas PX4 NED yaw positive means right.
	return -static_cast<float>(centidegrees_s) * .01f * static_cast<float>(M_PI) / 180.f;
}

inline float speed_output(State state, float requested_speed)
{
	return state == State::Yaw || state == State::YawRate ? requested_speed : 0.f;
}

struct BrushParameters {
	bool front{false};
	bool rear{false};
	uint8_t power_percent{0};
};

inline bool read_brush_parameter(uint16_t index, const BrushParameters &params, uint32_t &value)
{
	switch (index) {
	case 0x0101: value = params.front; return true;
	case 0x0102: value = params.rear; return true;
	case 0x0103: value = params.front && params.rear; return true;
	case 0x0104: value = params.power_percent; return true;
	default: return false;
	}
}

inline bool write_brush_parameter(uint16_t index, uint8_t type, uint32_t value, bool low_battery,
				  BrushParameters &params, uint32_t &feedback)
{
	if (type != 3) { return false; }
	if (low_battery && value != 0 && (index == 0x0101 || index == 0x0102 || index == 0x0103)) { return false; }
	switch (index) {
	case 0x0101: params.front = value != 0; feedback = params.front; return true;
	case 0x0102: params.rear = value != 0; feedback = params.rear; return true;
	case 0x0103: params.front = params.rear = value != 0; feedback = value != 0; return true;
	case 0x0104: params.power_percent = value > 100 ? 100 : static_cast<uint8_t>(value); feedback = params.power_percent; return true;
	default: return false;
	}
}
enum class Command : uint8_t { SpeedYaw, SpeedYawRate, Turn, Navigation, Estop, ClearEstop, CancelNavigation };

inline bool valid_speed(uint8_t mode, int16_t velocity_cms, int16_t yaw_data)
{
	if (velocity_cms < -500 || velocity_cms > 500) { return false; }
	if (mode == 1) { return true; }
	if (mode == 2) { return yaw_data >= -18000 && yaw_data <= 18000; }
	return false;
}

struct Transition {
	State state{State::Standby};
	bool accepted{false};
	bool stop_motion{true};
	bool reset_target{false};
};

class Logic
{
public:
	Transition command(Command command, bool armed)
	{
		Transition result{_state, false, true, false};
		if (command == Command::Estop) {
			_state = State::Estop;
			result = {_state, true, true, true};
			return result;
		}
		if (command == Command::ClearEstop && _state == State::Estop) {
			_state = State::Standby;
			result = {_state, true, true, true};
			return result;
		}
		if (!armed || _state == State::Estop) { return result; }
		if (_state == State::Turn) { return result; }

		switch (command) {
		case Command::SpeedYaw: _state = State::Yaw; break;
		case Command::SpeedYawRate: _state = State::YawRate; break;
		case Command::Turn:
			if (_state == State::Turn) { return result; }
			_state = State::Turn;
			break;
		case Command::Navigation: _state = State::Navigation; break;
		case Command::CancelNavigation:
			if (_state != State::Navigation) { return result; }
			_state = State::Standby;
			break;
		default: return result;
		}

		return {_state, true, _state == State::Standby, false};
	}

	Transition disarm()
	{
		if (_state != State::Estop) { _state = State::Standby; }
		return {_state, true, true, true};
	}

	Transition timeout()
	{
		if (_state == State::Yaw || _state == State::YawRate) {
			_state = State::Standby;
			return {_state, true, true, true};
		}
		return {_state, false, _state == State::Standby || _state == State::Estop, false};
	}

	State state() const { return _state; }

private:
	State _state{State::Standby};
};
}
