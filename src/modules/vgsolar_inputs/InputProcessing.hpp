// SPDX-License-Identifier: BSD-3-Clause
// VGSolar输入处理：纯计算层，可在主机测试，不访问ADC/GPIO或执行器。
#pragma once
#include <stdint.h>
#include <math.h>

namespace vgsolar_inputs
{
enum class PressureStatus : uint8_t { Waiting = 0, WarmingUp, Healthy, InvalidVoltage, Stale, InvalidConfig, MissingChannel, AdcError, Disabled };

struct Calibration {
	float v_min{0.1f}, v_max{3.1f}, p_min{0.f}, p_max{-100.f}, filter_hz{5.f};
	bool valid() const
	{
		return isfinite(v_min) && isfinite(v_max) && v_min >= 0.02f && v_max <= 3.28f && v_max > v_min
		       && isfinite(p_min) && isfinite(p_max) && p_min >= -150.f && p_min <= 50.f && p_max >= -150.f && p_max <= 50.f
		       && fabsf(p_max - p_min) > 1e-6f && isfinite(filter_hz) && filter_hz >= 0.f && filter_hz <= 20.f;
	}
};

// 读取现有adc_report的参考电压/分辨率，不假定固定12位或16位，不重配ADC。
inline float adc_voltage(int32_t raw, uint32_t resolution, float reference)
{
	if (raw < 0 || resolution == 0 || static_cast<uint32_t>(raw) >= resolution
	    || !isfinite(reference) || reference <= 0.f) { return NAN; }

	return static_cast<float>(raw) * reference / static_cast<float>(resolution);
}

class Pressure
{
public:
	static constexpr uint64_t StaleUs = 250000;
	void configure(const Calibration &cal)
	{
		_cal = cal;
		_have_sample = false;
		reset_validity();
		_status = cal.valid() ? PressureStatus::Waiting : PressureStatus::InvalidConfig;
	}

	void sample(float voltage, uint64_t stamp, uint64_t now)
	{
		if (!_cal.valid()) { reject(PressureStatus::InvalidConfig); return; }

		// 陈旧/未来/重复时间戳不能用于刷新健康状态或累计100ms预热。
		if (stamp == 0 || stamp > now || now - stamp >= StaleUs) { reject(PressureStatus::Stale); return; }
		if (_have_sample && stamp <= _last_stamp) { tick(now); return; }

		const bool continuity = _have_sample && stamp - _last_stamp < StaleUs;
		const float dt = continuity ? (stamp - _last_stamp) * 1e-6f : 0.02f;
		_last_stamp = stamp;
		_have_sample = true;
		_voltage = voltage;

		if (!continuity) { reset_validity(); }

		// 保留APM端点外0.1V容差，同时额外拒绝靠近0/3.3V电源轨的输入。
		// 即便电压在窗口内，也无法证明传感器已连接（原硬件R95未装）。
		const float lower = fmaxf(0.02f, _cal.v_min - 0.1f);
		const float upper = fminf(3.28f, _cal.v_max + 0.1f);

		if (!isfinite(voltage)) { reject(PressureStatus::AdcError); return; }
		if (voltage < lower || voltage > upper) { reject(PressureStatus::InvalidVoltage); return; }

		_raw = _cal.p_min + (voltage - _cal.v_min) * (_cal.p_max - _cal.p_min) / (_cal.v_max - _cal.v_min);

		if (!_valid_run) {
			_valid_run = true;
			_valid_since = stamp;
			_filtered = _raw; // 失效后恢复时重新起算，避免混入旧压力。
		} else {
			const float alpha = _cal.filter_hz > 0.f ? dt / (dt + 1.f / (6.283185307f * _cal.filter_hz)) : 1.f;
			_filtered += alpha * (_raw - _filtered);
		}

		_status = stamp - _valid_since >= 100000 ? PressureStatus::Healthy : PressureStatus::WarmingUp;
	}

	void missing(uint64_t stamp)
	{
		_last_stamp = stamp;
		_have_sample = true;
		_voltage = NAN;
		reject(_cal.valid() ? PressureStatus::MissingChannel : PressureStatus::InvalidConfig);
	}

	void tick(uint64_t now)
	{
		if (_cal.valid() && _have_sample && (now < _last_stamp || now - _last_stamp >= StaleUs)) { reject(PressureStatus::Stale); }
	}

	PressureStatus status() const { return _status; }
	bool healthy() const { return _status == PressureStatus::Healthy; }
	float voltage() const { return _voltage; } // 最近原始电压，仅供诊断，不能代替healthy()。
	float raw_kpa() const { return _raw; }
	float kpa() const { return healthy() ? _filtered : NAN; }
	uint64_t timestamp() const { return _have_sample ? _last_stamp : 0; }

private:
	void reset_validity() { _valid_run = false; _raw = NAN; _filtered = NAN; }
	void reject(PressureStatus status) { reset_validity(); _status = status; }
	Calibration _cal;
	PressureStatus _status{PressureStatus::Waiting};
	bool _have_sample{false}, _valid_run{false};
	uint64_t _last_stamp{0}, _valid_since{0};
	float _voltage{NAN}, _raw{NAN}, _filtered{NAN};
};

// 红外输入只去抖和解释电平，不把静态低电平直接认定为机械放下成功。
class Infrared
{
public:
	void configure(bool invert, uint32_t debounce_ms)
	{
		_invert = invert;
		_delay = static_cast<uint64_t>(debounce_ms) * 1000;
		_initialized = false;
		_valid = false;
	}
	void sample(bool raw, uint64_t now)
	{
		if (!_initialized || now < _last_sample || now - _last_sample >= Pressure::StaleUs) {
			_initialized = true;
			_valid = false;
			_candidate = raw;
			_since = now;
		}
		_last_sample = now;
		_raw = raw;

		if (raw != _candidate) { _candidate = raw; _since = now; }
		if (now - _since >= _delay) { _stable = _candidate; _valid = true; }
	}
	bool raw() const { return _raw; }
	bool stable() const { return _stable; }
	bool valid(uint64_t now) const { return _valid && now >= _last_sample && now - _last_sample < Pressure::StaleUs; }
	bool blade_present() const { return _invert ? !_stable : _stable; }
private:
	bool _initialized{false}, _valid{false}, _invert{false}, _raw{false}, _candidate{false}, _stable{false};
	uint64_t _since{0}, _last_sample{0}, _delay{30000};
};
}
