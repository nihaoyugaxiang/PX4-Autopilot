// SPDX-License-Identifier: BSD-3-Clause
#include "../Protocol.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
using namespace vgsolar_esc;
int main() {
	uint8_t query[8]{}; request(2, query);
	assert(query[0] == 2 && query[1] == 3 && query[3] == 0x20 && query[5] == 8);
	assert(crc16(query, 8) == 0);
	const uint8_t text[] = "123456789"; assert(crc16(text, 9) == 0x4b37);
	uint16_t values[8] = {0x100, 1234, 3456, static_cast<uint16_t>(-20), 487, 2, 0xabcd, 0x1234};
	uint8_t frame[21]{2, 3, 16};
	for (unsigned i = 0; i < 8; ++i) { frame[3 + i * 2] = values[i] >> 8; frame[4 + i * 2] = values[i]; }
	uint16_t crc = crc16(frame, 19); frame[19] = crc; frame[20] = crc >> 8;
	Parser parser; Sample sample{};
	for (unsigned i = 0; i < 20; ++i) { assert(parser.push(frame[i], 2, sample) == Result::None); }
	assert(parser.push(frame[20], 2, sample) == Result::Valid);
	assert(sample.fault == 0x100 && sample.rpm == 3456 && sample.temperature_c == -20 && sample.hall == 0xabcd1234);
	assert(sample.current_a > 12.339f && sample.current_a < 12.341f);
	const Sample valid = sample;
	frame[19] ^= 1;
	for (uint8_t b : frame) { assert(parser.push(b, 2, sample) == Result::None); }
	assert(sample.hall == valid.hall && parser.crc_errors == 1);
	frame[19] ^= 1;
	for (uint8_t b : frame) { assert(parser.push(b, 1, sample) == Result::None); }
	parser.reset(); parser.push(0x99, 2, sample); parser.push(2, 2, sample);
	Result result = Result::None;
	for (uint8_t b : frame) { result = parser.push(b, 2, sample); }
	assert(result == Result::Valid);
	uint8_t exception[5]{2, 0x83, 2};
	crc = crc16(exception, 3); exception[3] = crc; exception[4] = crc >> 8;
	for (uint8_t b : exception) { result = parser.push(b, 2, sample); }
	assert(result == Result::Exception && sample.hall == valid.hall && parser.exceptions == 1);
	puts("PASS: Modbus CRC golden vector, read-only query, split/noisy/wrong-address frames, exception and atomic data");
}
