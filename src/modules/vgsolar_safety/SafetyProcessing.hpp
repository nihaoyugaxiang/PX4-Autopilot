// SPDX-License-Identifier: BSD-3-Clause
// 纯计算层：边缘检测和锁存安全请求，可在主机测试，不访问硬件或执行器。
#pragma once

#include <stdint.h>
#include <math.h>

namespace vgsolar_safety
{
static constexpr uint64_t SensorStaleUs = 500000;
enum class State : uint8_t { Waiting = 0, Safe, Pending, Latched, ConfigInvalid };

struct RangeSample {
	bool received{false};
	uint64_t timestamp{0};
	float min_distance{NAN};
	float max_distance{NAN};
	float distance{NAN};
	int8_t signal_quality{0};
	uint8_t type{0};
	uint8_t orientation{0};
	uint8_t mode{0};

	// 方向/模式/信号质量/量程/新鲜度须同时成立；-1表示质量未知，0表示无效。
	bool valid(uint64_t now, uint8_t expected_orientation) const
	{
		return received && timestamp > 0 && timestamp <= now && now - timestamp < SensorStaleUs
		       && mode == 1 && type == 1 && orientation == expected_orientation
		       && (signal_quality == -1 || (signal_quality > 0 && signal_quality <= 100))
		       && isfinite(min_distance) && isfinite(max_distance) && min_distance >= 0.f
		       && max_distance > min_distance && isfinite(distance)
		       && distance >= min_distance && distance <= max_distance;
	}
};

struct Status {
	uint64_t timestamp{0};
	uint64_t left_timestamp{0};
	uint64_t right_timestamp{0};
	float left_distance_m{NAN};
	float right_distance_m{NAN};
	float max_distance_cm{15.f};
	uint32_t debounce_ms{100};
	uint32_t unsafe_duration_ms{0};
	State state{State::Waiting};
	bool config_valid{true};
	bool left_valid{false};
	bool right_valid{false};
	bool left_unsafe{true};
	bool right_unsafe{true};
	bool clear_eligible{false};
	bool clear_attempted{false};
	bool clear_succeeded{false};
	bool latched{false};
	bool stop_request{false};
};

class Monitor
{
public:
	// 参数更新不能清除已有锁存；非法配置立即锁存并报告CONFIG_INVALID。
	void configure(float max_distance_cm, int32_t debounce_ms)
	{
		_max_distance_cm = max_distance_cm;
		_debounce_ms = debounce_ms >= 0 ? static_cast<uint32_t>(debounce_ms) : 0;
		_config_valid = isfinite(max_distance_cm) && max_distance_cm >= 1.f && max_distance_cm <= 450.f
				&& debounce_ms >= 10 && debounce_ms <= 2000;

		if (!_config_valid) { _latched = true; }
	}

	// 左右任一路失效或超限进入同一连续计时；只有两路都正常才重置计时。
	Status update(const RangeSample &left, const RangeSample &right, uint64_t now, bool clear_requested = false)
	{
		Status result{};
		result.timestamp = now;
		result.left_timestamp = left.timestamp;
		result.right_timestamp = right.timestamp;
		result.left_distance_m = left.received ? left.distance : NAN;
		result.right_distance_m = right.received ? right.distance : NAN;
		result.max_distance_cm = _max_distance_cm;
		result.debounce_ms = _debounce_ms;
		result.config_valid = _config_valid;
		result.left_valid = left.valid(now, 7);
		result.right_valid = right.valid(now, 1);
		// 以米比较，15cm本身正常，严格大于阈值才超限。
		const float limit_m = _max_distance_cm / 100.f;
		result.left_unsafe = !result.left_valid || !_config_valid || left.distance > limit_m;
		result.right_unsafe = !result.right_valid || !_config_valid || right.distance > limit_m;
		const bool unsafe = result.left_unsafe || result.right_unsafe;
		result.clear_eligible = _config_valid && !unsafe;

		if (unsafe) {
			if (!_tracking_unsafe || now < _unsafe_since) {
				_tracking_unsafe = true;
				_unsafe_since = now;
			}

			const uint64_t elapsed = now - _unsafe_since;
			result.unsafe_duration_ms = static_cast<uint32_t>(elapsed / 1000 > UINT32_MAX ? UINT32_MAX : elapsed / 1000);

			if (_config_valid && elapsed >= static_cast<uint64_t>(_debounce_ms) * 1000) { _latched = true; }

		} else {
			_tracking_unsafe = false;
			_unsafe_since = 0;
		}

		// 显式清除只在当轮两路均有效、未超限且配置正确时生效。
		// 在危险期间发送的清除不会排队到未来恢复时执行。
		if (clear_requested) {
			result.clear_attempted = true;

			if (result.clear_eligible && _latched) {
				_latched = false;
				result.clear_succeeded = true;
			}
		}

		result.latched = _latched;
		result.stop_request = _latched;
		result.state = !_config_valid ? State::ConfigInvalid : _latched ? State::Latched
			       : unsafe ? State::Pending : State::Safe;
		return result;
	}

private:
	float _max_distance_cm{15.f};
	uint32_t _debounce_ms{100};
	uint64_t _unsafe_since{0};
	bool _config_valid{true};
	bool _tracking_unsafe{false};
	bool _latched{false};
};
}
