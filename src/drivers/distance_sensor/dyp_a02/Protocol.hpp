// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace dyp_a02
{
// Matches the VGSolar APM protocol: FF, distance MSB, LSB, sum (mod 256).
class Parser
{
public:
	bool feed(uint8_t byte, uint64_t now, uint16_t &millimetres)
	{
		if (_used && now - _last_byte > 50000) {
			_used = 0;
			++partial_timeouts;
		}

		_last_byte = now;

		if (!_used && byte != 0xff) { return false; }

		_buffer[_used++] = byte;

		if (_used < sizeof(_buffer)) { return false; }

		if (static_cast<uint8_t>(_buffer[0] + _buffer[1] + _buffer[2]) == _buffer[3]) {
			millimetres = (static_cast<uint16_t>(_buffer[1]) << 8) | _buffer[2];
			_used = 0;
			return true;
		}

		++checksum_errors;

		// Retain a possible overlapping header, including FF payload bytes.
		size_t next = 1;

		while (next < sizeof(_buffer) && _buffer[next] != 0xff) { ++next; }

		_used = sizeof(_buffer) - next;

		for (size_t i = 0; i < _used; ++i) { _buffer[i] = _buffer[next + i]; }

		return false;
	}

	uint32_t checksum_errors{0};
	uint32_t partial_timeouts{0};
private:
	uint8_t _buffer[4] {};
	size_t _used{0};
	uint64_t _last_byte{0};
};

class Sample
{
public:
	static constexpr uint64_t TimeoutUs = 500000;
	void receive(uint16_t mm, uint64_t now) { _mm = mm; _time = now; _have_frame = true; }
	bool valid(uint64_t now) const { return _have_frame && now - _time < TimeoutUs && _mm >= 30 && _mm <= 4500; }
	uint16_t mm() const { return _mm; }
	// Finite, below-range sentinel also survives MAVLink's uint16 cm conversion.
	float distance_m(uint64_t now) const { return valid(now) ? _mm * 0.001f : 0.f; }
private:
	uint64_t _time{0};
	uint16_t _mm{0};
	bool _have_frame{false};
};
}
