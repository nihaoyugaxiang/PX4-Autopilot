// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>

namespace vgsolar_esc
{
inline uint16_t crc16(const uint8_t *data, size_t size) {
	uint16_t crc = 0xffff;
	for (size_t i = 0; i < size; ++i) {
		crc ^= data[i];
		for (unsigned bit = 0; bit < 8; ++bit) { crc = (crc & 1) ? (crc >> 1) ^ 0xa001 : crc >> 1; }
	}
	return crc;
}
inline void request(uint8_t address, uint8_t *out) {
	out[0] = address; out[1] = 3; out[2] = 0; out[3] = 0x20; out[4] = 0; out[5] = 8;
	const uint16_t crc = crc16(out, 6); out[6] = static_cast<uint8_t>(crc); out[7] = static_cast<uint8_t>(crc >> 8);
}
struct Sample {
	uint16_t fault{0}, rpm{0}, direction{0};
	int16_t temperature_c{0};
	float current_a{0.f}, voltage_v{0.f};
	uint32_t hall{0};
};
enum class Result { None, Valid, Exception };
class Parser
{
public:
	uint32_t crc_errors{0}, bad_responses{0}, exceptions{0};
	void reset() { _size = 0; }
	Result push(uint8_t byte, uint8_t expected, Sample &sample) {
		if (_size == sizeof(_buffer)) { drop(); }
		_buffer[_size++] = byte;
		while (_size >= 2) {
			if (_buffer[0] != expected || (_buffer[1] != 3 && _buffer[1] != 0x83)) { drop(); continue; }
			const size_t need = _buffer[1] == 0x83 ? 5 : 21;
			if (need == 21 && _size >= 3 && _buffer[2] != 16) { ++bad_responses; drop(); continue; }
			if (_size < need) { return Result::None; }
			const uint16_t crc = static_cast<uint16_t>(_buffer[need - 2]) | (static_cast<uint16_t>(_buffer[need - 1]) << 8);
			if (crc16(_buffer, need - 2) != crc) { ++crc_errors; drop(); continue; }
			if (need == 5) { ++exceptions; reset(); return Result::Exception; }
			uint16_t reg[8]{};
			for (unsigned i = 0; i < 8; ++i) { reg[i] = (static_cast<uint16_t>(_buffer[3 + i * 2]) << 8) | _buffer[4 + i * 2]; }
			const int16_t temp = static_cast<int16_t>(reg[3]);
			if (temp < -40 || temp > 150 || reg[5] > 3) { ++bad_responses; drop(); continue; }
			sample.fault = reg[0]; sample.current_a = reg[1] * .01f; sample.rpm = reg[2];
			sample.temperature_c = temp; sample.voltage_v = reg[4] * .1f; sample.direction = reg[5];
			sample.hall = (static_cast<uint32_t>(reg[6]) << 16) | reg[7];
			reset(); return Result::Valid;
		}
		return Result::None;
	}
private:
	void drop() { if (_size > 0) { --_size; memmove(_buffer, _buffer + 1, _size); } }
	uint8_t _buffer[21]{};
	size_t _size{0};
};
}
