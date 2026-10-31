// SPDX-License-Identifier: BSD-3-Clause
// 无硬件依赖的AB相状态机，方向符号表示相位顺序，需实板确认前进方向。
#pragma once

#include <stdint.h>

namespace vgsolar_encoders
{
// 相位编码为(A << 1) | B；00->01->11->10->00记为正向。
static constexpr int8_t TransitionSteps[16] = {
	0, 1, -1, 0,
	-1, 0, 0, 1,
	1, 0, 0, -1,
	0, -1, 1, 0
};

struct DecoderSnapshot {
	int64_t count{0};
	uint32_t transitions{0};
	uint32_t invalid_transitions{0};
};

class QuadratureDecoder
{
public:
	void reset(uint8_t initial_phase)
	{
		_previous_phase = initial_phase & 0x3;
		_count = 0;
		_transitions = 0;
		_invalid_transitions = 0;
	}

	int8_t update(uint8_t phase)
	{
		phase &= 0x3;

		if (phase == _previous_phase) {
			return 0;
		}

		const int8_t step = TransitionSteps[(_previous_phase << 2) | phase];
		_previous_phase = phase;

		if (step == 0) {
			++_invalid_transitions;
			return 0;
		}

		_count += step;
		++_transitions;
		return step;
	}

	DecoderSnapshot snapshot() const
	{
		return {_count, _transitions, _invalid_transitions};
	}

private:
	uint8_t _previous_phase{0};
	int64_t _count{0};
	uint32_t _transitions{0};
	uint32_t _invalid_transitions{0};
};
}
