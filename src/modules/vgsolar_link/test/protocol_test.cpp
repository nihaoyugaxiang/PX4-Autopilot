// Host-side framing tests; no PX4 runtime or hardware is required.
#include "../Protocol.hpp"
#include <assert.h>
#include <stdio.h>
#include <string.h>

using namespace vgsolar_link;

static size_t make_ncu_frame(uint8_t command, const uint8_t *body, uint8_t length, uint8_t *out)
{
	out[0] = FRAME_HEADER_1;
	out[1] = FRAME_HEADER_2;
	out[2] = SOURCE_NCU;
	out[3] = command;
	out[4] = length;
	memcpy(out + 5, body, length);
	uint8_t checksum = 0;
	for (size_t i = 2; i < static_cast<size_t>(5 + length); ++i) { checksum = static_cast<uint8_t>(checksum + out[i]); }
	out[5 + length] = checksum;
	out[6 + length] = FRAME_TAIL;
	return length + FRAME_OVERHEAD;
}

int main()
{
	uint8_t nav[15]{};
	assert(build_navigation_frame(1, 3, 0x12345678, -123, nav, sizeof(nav)) == 15);
	assert(nav[2] == 0xbb && nav[3] == 4 && nav[4] == 8 && nav[5] == 1 && nav[6] == 3);
	assert(nav[7] == 0x78 && nav[10] == 0x12 && nav[11] == 0x85 && nav[12] == 0xff && nav[14] == 0xff);
	uint8_t nav_checksum = 0;
	for (unsigned i = 2; i < 13; ++i) { nav_checksum = static_cast<uint8_t>(nav_checksum + nav[i]); }
	assert(nav[13] == nav_checksum);
	assert(build_navigation_frame(1, 3, 1, 0, nav, 14) == 0);
	StatusFields fields{};
	fields.battery_percent = 87;
	fields.longitude = 123456789;
	fields.latitude = -987654321;
	fields.heading = 35999;
	fields.roll = -123;
	fields.left_out_cm = 314;
	uint8_t status[STATUS_FRAME_LENGTH]{};
	assert(build_status_frame(fields, status, sizeof(status)) == 41);
	assert(status[0] == 0xA5 && status[1] == 0x5A && status[2] == 0xBB && status[3] == 1 && status[4] == 34);
	assert(status[5] == 87 && status[6] == 0x15 && status[7] == 0xCD && status[8] == 0x5B && status[9] == 0x07);
	assert(status[10] == 0x4F && status[11] == 0x97 && status[12] == 0x21 && status[13] == 0xC5);
	assert(status[14] == 0x9F && status[15] == 0x8C);
	assert(status[22] == 0x85 && status[23] == 0xFF);
	assert(status[24] == 0x00 && status[25] == 0x00);
	assert(status[26] == 0 && status[27] == 0);
	assert(status[28] == 0 && status[29] == 0 && status[30] == 0);
	assert(status[31] == 0x3A && status[32] == 0x01);
	assert(status[33] == 0xFF && status[34] == 0xFF && status[35] == 0xFF && status[36] == 0xFF);
	assert(status[37] == 0xFF && status[38] == 0xFF);
	assert(status[40] == 0xFF);
	uint8_t checksum = 0;
	for (size_t i = 2; i < 39; ++i) { checksum = static_cast<uint8_t>(checksum + status[i]); }
	assert(status[39] == checksum);
	assert(build_status_frame(fields, status, sizeof(status) - 1) == 0);

	const uint8_t speed_body[] = {2, 100, 0, 0xCB, 0xFD};
	uint8_t ncu_frame[32]{};
	const size_t ncu_length = make_ncu_frame(1, speed_body, sizeof(speed_body), ncu_frame);
	Parser parser;
	RxFrame frame{};
	for (size_t i = 0; i < ncu_length; ++i) {
		const RxResult result = parser.push(ncu_frame[i], 1000 + i * 100, frame);
		assert(result == (i + 1 == ncu_length ? RxResult::Valid : RxResult::None));
	}
	assert(frame.command == 1 && frame.length == sizeof(speed_body) && frame.body[4] == 0xFD);
	uint8_t turn_frame[32]{};
	const uint8_t turn_body[] = {1, 0, 0, 0, 0, 0};
	const size_t turn_length = make_ncu_frame(2, turn_body, sizeof(turn_body), turn_frame);
	unsigned parsed_frames = 0;
	for (size_t i = 0; i < ncu_length + turn_length; ++i) {
		const uint8_t byte = i < ncu_length ? ncu_frame[i] : turn_frame[i - ncu_length];
		if (parser.push(byte, 30000 + i, frame) == RxResult::Valid) {
			++parsed_frames;
			assert(frame.command == (parsed_frames == 1 ? 1 : 2));
		}
	}
	assert(parsed_frames == 2);
	Parser bad_checksum;
	uint8_t bad_frame[32]{};
	const size_t bad_frame_length = make_ncu_frame(1, speed_body, sizeof(speed_body), bad_frame);
	bad_frame[5] ^= 0x01;
	RxResult result = RxResult::None;
	for (size_t i = 0; i < bad_frame_length; ++i) { result = bad_checksum.push(bad_frame[i], 10000 + i, frame); }
	assert(result == RxResult::BadChecksum);
	for (size_t i = 0; i < ncu_length; ++i) {
		assert(parser.push(ncu_frame[i], 12000 + i, frame) == (i + 1 == ncu_length ? RxResult::Valid : RxResult::None));
	}
	assert(frame.command == 1);

	Parser bad_length;
	uint8_t invalid_length[8] = {FRAME_HEADER_1, FRAME_HEADER_2, SOURCE_NCU, 1, 0, 0, FRAME_TAIL};
	assert(bad_length.push(invalid_length[0], 15000, frame) == RxResult::None);
	assert(bad_length.push(invalid_length[1], 15001, frame) == RxResult::None);
	assert(bad_length.push(invalid_length[2], 15002, frame) == RxResult::None);
	assert(bad_length.push(invalid_length[3], 15003, frame) == RxResult::None);
	assert(bad_length.push(invalid_length[4], 15004, frame) == RxResult::BadLength);
	for (size_t i = 0; i < ncu_length; ++i) {
		assert(bad_length.push(ncu_frame[i], 15100 + i, frame) == (i + 1 == ncu_length ? RxResult::Valid : RxResult::None));
	}

	Parser max_length;
	uint8_t maximum_body[15]{};
	uint8_t max_frame[32]{};
	const size_t max_frame_length = make_ncu_frame(6, maximum_body, sizeof(maximum_body), max_frame);
	RxResult max_result = RxResult::None;
	for (size_t i = 0; i < max_frame_length; ++i) { max_result = max_length.push(max_frame[i], 17000 + i, frame); }
	assert(max_result == RxResult::Valid && frame.length == 15);
	Parser wrong_source;
	assert(wrong_source.push(FRAME_HEADER_1, 19000, frame) == RxResult::None);
	assert(wrong_source.push(FRAME_HEADER_2, 19001, frame) == RxResult::None);
	assert(wrong_source.push(SOURCE_FCU, 19002, frame) == RxResult::RejectedSource);
	for (size_t i = 0; i < ncu_length; ++i) {
		assert(wrong_source.push(ncu_frame[i], 19100 + i, frame) == (i + 1 == ncu_length ? RxResult::Valid : RxResult::None));
	}

	Parser timeout;
	timeout.push(FRAME_HEADER_1, 1000, frame);
	assert(timeout.expire(202000) == RxResult::Timeout);
	assert(timeout.push(FRAME_HEADER_1, 203000, frame) == RxResult::None);
	timeout.push(FRAME_HEADER_2, 203100, frame);
	timeout.push(SOURCE_NCU, 203200, frame);
	assert(timeout.expire(403201) == RxResult::Timeout);
	for (size_t i = 0; i < ncu_length; ++i) {
		assert(timeout.push(ncu_frame[i], 404000 + i, frame) == (i + 1 == ncu_length ? RxResult::Valid : RxResult::None));
	}

	uint8_t ack[9]{};
	assert(build_failed_ack(1, ack, sizeof(ack)) == 9);
	assert(ack[2] == SOURCE_FCU && ack[3] == CMD_ACK && ack[5] == 1 && ack[6] == 2 && ack[8] == FRAME_TAIL);
	checksum = 0;
	for (size_t i = 2; i < 7; ++i) { checksum = static_cast<uint8_t>(checksum + ack[i]); }
	assert(ack[7] == checksum);

	uint8_t param_feedback[FRAME_OVERHEAD + 7]{};
	assert(build_param_feedback(0x0104, 3, 100, param_feedback, sizeof(param_feedback)) == sizeof(param_feedback));
	assert(param_feedback[2] == SOURCE_FCU && param_feedback[3] == CMD_PARAM_FEEDBACK && param_feedback[4] == 7);
	assert(param_feedback[5] == 0x04 && param_feedback[6] == 0x01 && param_feedback[7] == 3);
	assert(param_feedback[8] == 100 && param_feedback[9] == 0 && param_feedback[10] == 0 && param_feedback[11] == 0);
	checksum = 0;
	for (size_t i = 2; i < sizeof(param_feedback) - 2; ++i) { checksum = static_cast<uint8_t>(checksum + param_feedback[i]); }
	assert(param_feedback[sizeof(param_feedback) - 2] == checksum && param_feedback[sizeof(param_feedback) - 1] == FRAME_TAIL);
	assert(build_param_feedback(0x0104, 3, 100, param_feedback, sizeof(param_feedback) - 1) == 0);
	puts("PASS: APM status vector, capacity, split/concatenated NCU frames, malformed-frame recovery, timeout, reject ACK");
}
