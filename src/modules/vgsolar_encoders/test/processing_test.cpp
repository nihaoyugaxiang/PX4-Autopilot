// SPDX-License-Identifier: BSD-3-Clause
// 仅测试AB相纯解码逻辑，不访问GPIO或连接实物。
#include "../QuadratureDecoder.hpp"
#include <assert.h>
#include <stdio.h>

using vgsolar_encoders::QuadratureDecoder;

int main()
{
	QuadratureDecoder decoder;
	decoder.reset(0b00);
	assert(decoder.update(0b01) == 1);
	assert(decoder.update(0b11) == 1);
	assert(decoder.update(0b10) == 1);
	assert(decoder.update(0b00) == 1);
	assert(decoder.snapshot().count == 4);

	assert(decoder.update(0b10) == -1);
	assert(decoder.update(0b11) == -1);
	assert(decoder.update(0b01) == -1);
	assert(decoder.update(0b00) == -1);
	assert(decoder.snapshot().count == 0);

	// 同一相位不计数；单步有效反向边沿可撤销此前计数。
	assert(decoder.update(0b00) == 0);
	assert(decoder.update(0b01) == 1);
	assert(decoder.update(0b00) == -1);
	assert(decoder.snapshot().count == 0);

	// 两位同时变化无法判定方向，拒绝计数并单独增加诊断计数。
	assert(decoder.update(0b11) == 0);
	const auto result = decoder.snapshot();
	assert(result.count == 0);
	assert(result.transitions == 10);
	assert(result.invalid_transitions == 1);
	assert(decoder.update(0b10) == 1); // 跳变后以新相位恢复，不能沿用错误发生前的相位。

	// 从四个初始相位分别检查下一步的全部16种组合。
	const uint8_t cycle[] = {0b00, 0b01, 0b11, 0b10};
	for (unsigned start = 0; start < 4; ++start) {
		for (unsigned end = 0; end < 4; ++end) {
			decoder.reset(cycle[start]);
			const unsigned advance = (end + 4 - start) % 4;
			const int expected = advance == 1 ? 1 : (advance == 3 ? -1 : 0);
			assert(decoder.update(cycle[end]) == expected);
			assert(decoder.snapshot().count == expected);
			assert(decoder.snapshot().invalid_transitions == (advance == 2 ? 1u : 0u));
		}
	}

	// 两轮各自积累相反方向，启动时不在00也不应多计初始相位。
	QuadratureDecoder other;
	decoder.reset(0b11);
	other.reset(0b11);
	for (unsigned edge = 1; edge <= 100000; ++edge) {
		decoder.update(cycle[(2 + edge) % 4]);
		other.update(cycle[(2 + 400000 - edge) % 4]);
	}
	assert(decoder.snapshot().count == 100000);
	assert(other.snapshot().count == -100000);
	assert(decoder.snapshot().transitions == 100000 && decoder.snapshot().invalid_transitions == 0);
	decoder.reset(0b10);
	assert(decoder.snapshot().count == 0 && decoder.snapshot().transitions == 0);

	puts("PASS: all 16 phase transitions, forward/reverse cycles, duplicate/reversal/invalid recovery, independent wheels, 100k signed edges, restart");
}
