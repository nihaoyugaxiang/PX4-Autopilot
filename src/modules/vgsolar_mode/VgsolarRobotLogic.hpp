// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <stdint.h>

namespace vgsolar_robot
{
enum class SuctionState : uint8_t { Raised, Lowering, WaitDown, Vacuum, Lowered, Venting, Raising, WaitUp, Fault };
struct Config {
	float raised_pwm{1900.f}, lowered_pwm{1100.f}, slew_us_s{400.f}, vacuum_kpa{-70.f}, vacuum_hysteresis_kpa{5.f};
	uint32_t lift_delay_ms{2000}, vacuum_delay_ms{10000}, vent_delay_ms{2000}, action_timeout_ms{30000};
	uint32_t lift_timeout_ms{5000}, vacuum_debounce_ms{300};
	bool use_ir{true}, use_pressure{true};
	bool valid() const {
		return std::isfinite(raised_pwm) && raised_pwm >= 1000.f && raised_pwm <= 2000.f
		       && std::isfinite(lowered_pwm) && lowered_pwm >= 1000.f && lowered_pwm <= 2000.f
		       && std::isfinite(slew_us_s) && slew_us_s >= 50.f && slew_us_s <= 5000.f
		       && std::isfinite(vacuum_kpa) && vacuum_kpa >= -100.f && vacuum_kpa < 0.f
		       && lift_delay_ms <= 5000 && vacuum_delay_ms >= 100 && vacuum_delay_ms <= 10000
		       && vent_delay_ms >= 100 && vent_delay_ms <= 5000 && action_timeout_ms >= 1000
		       && action_timeout_ms <= 60000 && lift_timeout_ms >= 500 && lift_timeout_ms <= 15000
		       && vacuum_debounce_ms <= 5000 && std::isfinite(vacuum_hysteresis_kpa)
		       && vacuum_hysteresis_kpa >= 0.f && vacuum_hysteresis_kpa <= 50.f;
	}
};
struct Inputs {
	bool fresh{false}, ir_valid{false}, blade{false}, pressure_valid{false};
	float pressure_kpa{NAN};
};
class Suction
{
public:
	void configure(const Config &config) { _config = config; }
	const Config &config() const { return _config; }
	bool lower(uint64_t now) {
		if (!_config.valid() || _fault || _frozen || _state != SuctionState::Raised) { return false; }
		_seen_blade = false; _vac_since = _vac_lost = _blade_since = 0; _action_start = now;
		_state = SuctionState::Lowering; _phase_start = now; return true;
	}
	bool raise(uint64_t now) {
		if (_fault || _frozen) { return false; }
		_action_start = now; _phase_start = now; _blade_since = 0;
		_state = SuctionState::Venting; _pump = _sealed = false; return true;
	}
	// Disarm, mode exit and ESTOP release vacuum and command the raised endpoint.
	void release() {
		_state = SuctionState::Raised; _pump = _sealed = _frozen = false;
		_pwm = _config.raised_pwm; _last_update = 0;
	}
	// A request is not proof of physical completion. Repeated requests never restart vent timing.
 void emergency_release(uint64_t now) {
  if (_state == SuctionState::Raised) { _pump = _sealed = false; return; }
  _frozen = false;
  if (_state == SuctionState::Venting || _state == SuctionState::Raising || _state == SuctionState::WaitUp) { return; }
  _fault = false; raise(now);
 }
 void clear_fault() { _fault = false; _vacuum_warning = false; release(); }
	void freeze() {
  if (!_frozen) { _frozen_ready = _state == SuctionState::Lowered || _state == SuctionState::Venting
      || _state == SuctionState::Raising || _state == SuctionState::WaitUp; }
  _frozen = true; _pump = false;
  if (_frozen_ready) { _sealed = true; _pwm = _config.lowered_pwm; }
 }
	void unfreeze() {
		_frozen = false;
		if (_frozen_ready) { _state = SuctionState::Lowered; _pump = _sealed = true; }
		else { release(); }
	}
	void tick(uint64_t now, bool enabled, const Inputs &input) {
		const float dt = _last_update != 0 && now >= _last_update ? fminf((now - _last_update) * 1e-6f, 0.1f) : 0.f;
		_last_update = now;
		if (!enabled) { emergency_release(now); return; }
		if (!_config.valid()) { fail(); return; }
		if (_frozen) { _pump = false; if (_frozen_ready) { observe_vacuum(now, input); } return; }
		if (_fault) { _pump = _sealed = false; return; }
		if (busy() && now >= _action_start && now - _action_start > _config.action_timeout_ms * 1000ULL) { fail(); return; }
		const bool releasing = _state == SuctionState::Venting || _state == SuctionState::Raising || _state == SuctionState::WaitUp;
		const bool needs_sensor = _state != SuctionState::Raised && _state != SuctionState::Lowered;
		if (needs_sensor && ((_config.use_ir && (!input.fresh || !input.ir_valid))
		    || (_config.use_pressure && !releasing && (!input.fresh || !input.pressure_valid || !std::isfinite(input.pressure_kpa))))) {
			fail(); return;
		}
		switch (_state) {
		case SuctionState::Raised: _pwm = _config.raised_pwm; _pump = _sealed = false; break;
		case SuctionState::Lowering:
			_pump = _sealed = false;
			if (input.ir_valid && input.blade) { _seen_blade = true; }
			if (ramp(_config.lowered_pwm, dt)) { _state = SuctionState::WaitDown; _phase_start = now; }
			break;
		case SuctionState::WaitDown:
			if (input.ir_valid && input.blade) { _seen_blade = true; }
			if ((_config.use_ir && _seen_blade && !input.blade)
			    || (!_config.use_ir && elapsed(now, _config.lift_delay_ms))) {
				_state = SuctionState::Vacuum; _phase_start = now; _sealed = _pump = true; _vac_since = 0;
			} else if (_config.use_ir && elapsed(now, _config.lift_timeout_ms)) { fail(); }
			break;
		case SuctionState::Vacuum:
			_pump = _sealed = true;
			if (_config.use_pressure) {
				if (input.pressure_kpa <= _config.vacuum_kpa) {
					if (_vac_since == 0) { _vac_since = now; }
					if (now - _vac_since >= _config.vacuum_debounce_ms * 1000ULL) { _state = SuctionState::Lowered; }
				} else { _vac_since = 0; }
				if (_state == SuctionState::Vacuum && elapsed(now, _config.vacuum_delay_ms)) { fail(); }
			} else if (elapsed(now, _config.vacuum_delay_ms)) { _state = SuctionState::Lowered; }
			break;
		case SuctionState::Lowered:
			_pump = _sealed = true;
			observe_vacuum(now, input);
			break;
		case SuctionState::Venting:
			_pump = _sealed = false;
			if (elapsed(now, _config.vent_delay_ms)) { _state = SuctionState::Raising; _phase_start = now; }
			break;
		case SuctionState::Raising:
			if (input.ir_valid && input.blade && _blade_since == 0) { _blade_since = now; }
			if (ramp(_config.raised_pwm, dt)) { _state = SuctionState::WaitUp; _phase_start = now; }
			break;
		case SuctionState::WaitUp:
			if (input.ir_valid && input.blade && _blade_since == 0) { _blade_since = now; }
			if ((!_config.use_ir && elapsed(now, _config.lift_delay_ms))
			    || (_config.use_ir && _blade_since != 0 && now - _blade_since >= _config.lift_delay_ms * 1000ULL)) {
				_state = SuctionState::Raised;
			} else if (_config.use_ir && elapsed(now, _config.lift_timeout_ms)) { fail(); }
			break;
		case SuctionState::Fault: break;
		}
	}
	bool lowered() const { return _state == SuctionState::Lowered && !_fault; }
	bool raised() const { return _state == SuctionState::Raised && !_fault; }
	bool busy() const { return _state != SuctionState::Raised && _state != SuctionState::Lowered && _state != SuctionState::Fault; }
	bool fault() const { return _fault; }
	bool vacuum_warning() const { return _vacuum_warning; }
	bool frozen() const { return _frozen; }
	bool pump() const { return _pump; }
	bool sealed() const { return _sealed; }
	float pwm() const { return _pwm; }
	SuctionState state() const { return _state; }
private:
 void observe_vacuum(uint64_t now, const Inputs &input) {
  if (!_config.use_pressure || !input.fresh || !input.pressure_valid || !std::isfinite(input.pressure_kpa)) { _vac_lost = 0; return; }
  if (input.pressure_kpa > _config.vacuum_kpa + _config.vacuum_hysteresis_kpa) {
   if (_vac_lost == 0) { _vac_lost = now; }
   if (now - _vac_lost >= _config.vacuum_debounce_ms * 1000ULL) { _vacuum_warning = true; }
  } else { _vac_lost = 0; if (input.pressure_kpa <= _config.vacuum_kpa) { _vacuum_warning = false; } }
 }
	bool elapsed(uint64_t now, uint32_t ms) const { return now >= _phase_start && now - _phase_start >= ms * 1000ULL; }
	bool ramp(float target, float dt) {
		const float step = _config.slew_us_s * dt;
		if (fabsf(target - _pwm) <= step) { _pwm = target; return true; }
		_pwm += target > _pwm ? step : -step; return false;
	}
	void fail() { _fault = true; _state = SuctionState::Fault; _pump = _sealed = false; }
	Config _config{};
	SuctionState _state{SuctionState::Raised};
	float _pwm{1900.f};
	uint64_t _last_update{0}, _action_start{0}, _phase_start{0}, _vac_since{0}, _vac_lost{0}, _blade_since{0};
	bool _frozen_ready{false}, _vacuum_warning{false};
	bool _pump{false}, _sealed{false}, _frozen{false}, _fault{false}, _seen_blade{false};
};
}
