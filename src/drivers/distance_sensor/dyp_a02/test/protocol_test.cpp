// SPDX-License-Identifier: BSD-3-Clause
#include "../Protocol.hpp"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
#include <math.h>

using dyp_a02::Parser;
using dyp_a02::Sample;

// 测试辅助函数：构造指定毫米数的帧；corrupt=true时故意将校验和加1。
// 这里只喂协议层字节，不需要串口或真实传感器。
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
	// 固定写出的黄金帧应解析为1953mm，避免完全依赖辅助函数生成测试数据。
	const uint8_t golden[] {0xff, 0x07, 0xa1, 0xa7};

	// 前3字节分别送入，模拟分包；最后的校验字节到达前不能报告完整帧。
	for (unsigned i = 0; i < 3; ++i) { assert(!p.feed(golden[i], 1000 + i * 1000, mm)); }

	assert(p.feed(golden[3], 4000, mm) && mm == 1953);
	// 拒绝坏校验，记录计数，紧接着的合法帧应可恢复解析。
	assert(!frame(p, 150, 5000, mm, true));
	assert(p.checksum_errors == 1);
	assert(frame(p, 160, 6000, mm) && mm == 160);
	// 连续帧及距离字节中的FF都不能被错误地当作新帧头。
	for (uint16_t value : {uint16_t(255), uint16_t(65280), uint16_t(65535), uint16_t(256), uint16_t(30), uint16_t(4500)}) {
		assert(frame(p, value, 10000, mm) && mm == value);
	}

	// 留下半帧，后续字节到达时已超过50ms，应丢弃旧半帧再恢复收包。
	assert(!p.feed(0xff, 20000, mm));
	assert(!p.feed(0x07, 21000, mm));
	assert(!p.feed(0xa1, 100000, mm)); // 半帧已过期，不能继续拼成旧帧。
	assert(!p.feed(0xa7, 101000, mm));
	assert(p.partial_timeouts == 1);
	assert(frame(p, 150, 102000, mm) && mm == 150);
	// 错帧中夹着下一帧头时，验证保留重叠字节后仍能得到150mm。
	Parser overlap;
	for (uint8_t byte : {0xff, 0x12, 0xff, 0x00, 0x96}) { assert(!overlap.feed(byte, 100, mm)); }

	assert(overlap.feed(0x95, 100, mm) && mm == 150);
	// 穷举全部16位负载，验证字节组合不会破坏解包。
	// 解析成功不代表该数值在传感器有效量程内，量程由后面的Sample测试判断。
	Parser exhaustive;

	for (unsigned value = 0; value <= 65535; ++value) {
		assert(frame(exhaustive, value, 100, mm) && mm == value);
	}

	// 左路停在半帧状态时，右路应继续正常解析，验证独立状态隔离。
	Parser left, right;
	assert(!left.feed(0xff, 100, mm));
	assert(frame(right, 500, 100, mm) && mm == 500);
	assert(frame(right, 150, 200, mm) && mm == 150);
	// 没有样本时即为无效；失效输出必须为0，不能伪装成一个有效距离。
	Sample sample;
	assert(!sample.valid(100));
	assert(sample.distance_m(100) == 0.f);
	// 单位换算及500ms边界：499999us仍有效，恰好500000us应无效。
	sample.receive(150, 1000);
	assert(sample.valid(1000) && sample.valid(500999));
	assert(fabsf(sample.distance_m(1000) - 0.15f) < 0.00001f);
	assert(!sample.valid(501000));
	assert(sample.distance_m(501000) == 0.f);
	// 新收到的越量程完整帧必须撤销之前的有效值，不能继续沿用旧数据。
	for (uint16_t value : {uint16_t(0), uint16_t(29), uint16_t(4501), uint16_t(65535)}) {
		sample.receive(value, 600000);
		assert(!sample.valid(600000));
		assert(sample.distance_m(600000) == 0.f);
	}

	// 量程两端包含在有效范围内；150与160mm都有效，边缘安全阈值不在驱动层判断。
	for (uint16_t value : {uint16_t(30), uint16_t(150), uint16_t(160), uint16_t(4500)}) {
		sample.receive(value, 700000);
		assert(sample.valid(700000) && sample.mm() == value);
	}

	puts("PASS: golden/split/sticky/corrupt/overlap/partial-timeout/65536-payloads/channel-isolation/invalid/stale/recovery");
}
