#pragma once

#include <cstddef>
#include <cstdint>

namespace vgsolar_link
{

constexpr uint8_t FRAME_HEADER_1 = 0xA5;
constexpr uint8_t FRAME_HEADER_2 = 0x5A;
constexpr uint8_t FRAME_TAIL = 0xFF;
constexpr uint8_t SOURCE_NCU = 0xAA;
constexpr uint8_t SOURCE_FCU = 0xBB;
constexpr uint8_t CMD_STATUS = 0x01;
constexpr uint8_t CMD_ACK = 0x02;
constexpr uint8_t STATUS_BODY_LENGTH = 34;
constexpr size_t FRAME_OVERHEAD = 7;
constexpr size_t STATUS_FRAME_LENGTH = FRAME_OVERHEAD + STATUS_BODY_LENGTH;
constexpr uint16_t RANGE_INVALID_CM = 0xFFFF;

enum class RxResult : uint8_t { None, Valid, BadChecksum, BadLength, Timeout, RejectedSource };

struct RxFrame {
	uint8_t source{0};
	uint8_t command{0};
	uint8_t length{0};
	uint8_t body[32]{};
};

class Parser
{
public:
	RxResult push(uint8_t byte, uint64_t now_us, RxFrame &frame)
	{
		if (_state != 0 && now_us >= _start_us && now_us - _start_us > FRAME_TIMEOUT_US) {
			reset();
			start_if_header(byte, now_us);
			return RxResult::Timeout;
		}

		switch (_state) {
		case 0:
			if (byte == FRAME_HEADER_1) { _state = 1; _start_us = now_us; }
			return RxResult::None;
		case 1:
			if (byte == FRAME_HEADER_2) { _buffer[0] = FRAME_HEADER_1; _buffer[1] = FRAME_HEADER_2; _count = 2; _state = 2; }
			else { _state = byte == FRAME_HEADER_1 ? 1 : 0; if (_state == 1) { _start_us = now_us; } }
			return RxResult::None;
		case 2:
			if (byte != SOURCE_NCU) { reset(); start_if_header(byte, now_us); return RxResult::RejectedSource; }
			break;
		case 3:
			break;
		case 4:
			if (byte == 0 || byte > _max_body_length) { reset(); start_if_header(byte, now_us); return RxResult::BadLength; }
			break;
		default: break;
		}

		if (_count >= sizeof(_buffer)) { reset(); start_if_header(byte, now_us); return RxResult::BadLength; }
		_buffer[_count++] = byte;

		if (_state == 2) { _state = 3; return RxResult::None; }
		if (_state == 3) { _command = byte; _state = 4; return RxResult::None; }
		if (_state == 4) {
			_body_length = byte;
			_state = 5;
			return RxResult::None;
		}

		const size_t expected = FRAME_OVERHEAD + _body_length;
		if (_count < expected) { return RxResult::None; }
		if (_count != expected) { reset(); start_if_header(byte, now_us); return RxResult::BadLength; }
		if (_buffer[_count - 1] != FRAME_TAIL) { reset(); start_if_header(byte, now_us); return RxResult::BadLength; }

		uint8_t checksum = 0;
		for (size_t i = 2; i < _count - 2; ++i) { checksum = static_cast<uint8_t>(checksum + _buffer[i]); }
		if (checksum != _buffer[_count - 2]) { reset(); start_if_header(byte, now_us); return RxResult::BadChecksum; }

		frame.source = _buffer[2];
		frame.command = _command;
		frame.length = _body_length;
		for (unsigned i = 0; i < frame.length; ++i) { frame.body[i] = _buffer[5 + i]; }
		reset();
		return RxResult::Valid;
	}

	RxResult expire(uint64_t now_us)
	{
		if (_state != 0 && now_us >= _start_us && now_us - _start_us > FRAME_TIMEOUT_US) { reset(); return RxResult::Timeout; }
		return RxResult::None;
	}

private:
	static constexpr uint64_t FRAME_TIMEOUT_US = 200000;
	void start_if_header(uint8_t byte, uint64_t now_us)
	{
		if (byte == FRAME_HEADER_1) { _state = 1; _start_us = now_us; }
	}
	void reset() { _state = 0; _count = 0; _body_length = 0; _command = 0; }
	static constexpr uint8_t _max_body_length = 15;
	uint8_t _buffer[64]{};
	uint64_t _start_us{0};
	size_t _count{0};
	uint8_t _state{0};
	uint8_t _body_length{0};
	uint8_t _command{0};
};

struct StatusFields {
	uint8_t battery_percent{0};
	int32_t longitude{0};
	int32_t latitude{0};
	uint16_t heading{0};
	int16_t velocity{0};
	int16_t left_track_velocity{0};
	int16_t right_track_velocity{0};
	int16_t roll{0};
	int16_t pitch{0};
	uint8_t control_mode{0};
	uint8_t motion_state{0};
	uint16_t fault_code{0};
	uint8_t gps_status{0};
	uint16_t left_out_cm{RANGE_INVALID_CM};
	uint16_t left_in_cm{RANGE_INVALID_CM};
	uint16_t right_in_cm{RANGE_INVALID_CM};
	uint16_t right_out_cm{RANGE_INVALID_CM};
};

inline void put_u16(uint8_t *out, uint16_t value)
{
	out[0] = static_cast<uint8_t>(value);
	out[1] = static_cast<uint8_t>(value >> 8);
}

inline void put_i16(uint8_t *out, int16_t value) { put_u16(out, static_cast<uint16_t>(value)); }

inline void put_i32(uint8_t *out, int32_t value)
{
	const uint32_t raw = static_cast<uint32_t>(value);
	out[0] = static_cast<uint8_t>(raw);
	out[1] = static_cast<uint8_t>(raw >> 8);
	out[2] = static_cast<uint8_t>(raw >> 16);
	out[3] = static_cast<uint8_t>(raw >> 24);
}

inline size_t build_status_frame(const StatusFields &fields, uint8_t *out, size_t capacity)
{
	if (out == nullptr || capacity < STATUS_FRAME_LENGTH) { return 0; }
	out[0] = FRAME_HEADER_1;
	out[1] = FRAME_HEADER_2;
	out[2] = SOURCE_FCU;
	out[3] = CMD_STATUS;
	out[4] = STATUS_BODY_LENGTH;
	uint8_t *body = out + 5;
	body[0] = fields.battery_percent;
	put_i32(body + 1, fields.longitude);
	put_i32(body + 5, fields.latitude);
	put_u16(body + 9, fields.heading);
	put_i16(body + 11, fields.velocity);
	put_i16(body + 13, fields.left_track_velocity);
	put_i16(body + 15, fields.right_track_velocity);
	put_i16(body + 17, fields.roll);
	put_i16(body + 19, fields.pitch);
	body[21] = fields.control_mode;
	body[22] = fields.motion_state;
	put_u16(body + 23, fields.fault_code);
	body[25] = fields.gps_status;
	put_u16(body + 26, fields.left_out_cm);
	put_u16(body + 28, fields.left_in_cm);
	put_u16(body + 30, fields.right_in_cm);
	put_u16(body + 32, fields.right_out_cm);
	uint8_t checksum = 0;
	for (size_t i = 2; i < STATUS_FRAME_LENGTH - 2; ++i) { checksum = static_cast<uint8_t>(checksum + out[i]); }
	out[STATUS_FRAME_LENGTH - 2] = checksum;
	out[STATUS_FRAME_LENGTH - 1] = FRAME_TAIL;
	return STATUS_FRAME_LENGTH;
}

inline size_t build_failed_ack(uint8_t command, uint8_t *out, size_t capacity)
{
	if (out == nullptr || capacity < FRAME_OVERHEAD + 2) { return 0; }
	out[0] = FRAME_HEADER_1;
	out[1] = FRAME_HEADER_2;
	out[2] = SOURCE_FCU;
	out[3] = CMD_ACK;
	out[4] = 2;
	out[5] = command;
	out[6] = 2; // NCU command rejected: this PX4 stage is read-only.
	uint8_t checksum = 0;
	for (size_t i = 2; i < 7; ++i) { checksum = static_cast<uint8_t>(checksum + out[i]); }
	out[7] = checksum;
	out[8] = FRAME_TAIL;
	return 9;
}

} // namespace vgsolar_link
