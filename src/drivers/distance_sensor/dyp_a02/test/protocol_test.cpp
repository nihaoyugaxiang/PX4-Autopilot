// SPDX-License-Identifier: BSD-3-Clause
#include "../Protocol.hpp"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
#include <math.h>

using dyp_a02::Parser;
using dyp_a02::Sample;

static bool frame(Parser &p, uint16_t mm, uint64_t now, uint16_t &result, bool corrupt = false)
{
	const uint8_t hi = mm >> 8, lo = mm & 0xff;
	const uint8_t bytes[] {0xff, hi, lo, static_cast<uint8_t>(0xff + hi + lo + (corrupt ? 1 : 0))};
	bool accepted = false;

	for (uint8_t byte : bytes) { accepted |= p.feed(byte, now++, result); }

	return accepted;
}

int main()
{
	uint16_t mm = 0;
	Parser p;
	// Literal APM golden frame, not generated using the parser checksum.
	const uint8_t golden[] {0xff, 0x07, 0xa1, 0xa7};

	for (unsigned i = 0; i < 3; ++i) { assert(!p.feed(golden[i], 1000 + i * 1000, mm)); }

	assert(p.feed(golden[3], 4000, mm) && mm == 1953);
	assert(!frame(p, 150, 5000, mm, true));
	assert(p.checksum_errors == 1);
	assert(frame(p, 160, 6000, mm) && mm == 160);
	// Adjacent packets; FF in MSB/LSB and in checksum must not resync early.
	for (uint16_t value : {uint16_t(255), uint16_t(65280), uint16_t(65535), uint16_t(256), uint16_t(30), uint16_t(4500)}) {
		assert(frame(p, value, 10000, mm) && mm == value);
	}

	assert(!p.feed(0xff, 20000, mm));
	assert(!p.feed(0x07, 21000, mm));
	assert(!p.feed(0xa1, 100000, mm)); // expired partial packet
	assert(!p.feed(0xa7, 101000, mm));
	assert(p.partial_timeouts == 1);
	assert(frame(p, 150, 102000, mm) && mm == 150);
	Parser overlap;
	for (uint8_t byte : {0xff, 0x12, 0xff, 0x00, 0x96}) { assert(!overlap.feed(byte, 100, mm)); }

	assert(overlap.feed(0x95, 100, mm) && mm == 150);
	// Exhaust all payload values, including embedded header bytes.
	Parser exhaustive;

	for (unsigned value = 0; value <= 65535; ++value) {
		assert(frame(exhaustive, value, 100, mm) && mm == value);
	}

	// Independent channels: a truncated left stream must not affect the right.
	Parser left, right;
	assert(!left.feed(0xff, 100, mm));
	assert(frame(right, 500, 100, mm) && mm == 500);
	assert(frame(right, 150, 200, mm) && mm == 150);
	Sample sample;
	assert(!sample.valid(100));
	assert(sample.distance_m(100) == 0.f);
	sample.receive(150, 1000);
	assert(sample.valid(1000) && sample.valid(500999));
	assert(fabsf(sample.distance_m(1000) - 0.15f) < 0.00001f);
	assert(!sample.valid(501000));
	assert(sample.distance_m(501000) == 0.f);
	// An invalid decoded packet revokes the previous good value immediately.
	for (uint16_t value : {uint16_t(0), uint16_t(29), uint16_t(4501), uint16_t(65535)}) {
		sample.receive(value, 600000);
		assert(!sample.valid(600000));
		assert(sample.distance_m(600000) == 0.f);
	}

	for (uint16_t value : {uint16_t(30), uint16_t(150), uint16_t(160), uint16_t(4500)}) {
		sample.receive(value, 700000);
		assert(sample.valid(700000) && sample.mm() == value);
	}

	puts("PASS: golden/split/sticky/corrupt/overlap/partial-timeout/65536-payloads/channel-isolation/invalid/stale/recovery");
}
