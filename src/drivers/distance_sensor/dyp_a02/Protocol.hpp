// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace dyp_a02
{
// 纯协议层，不操作串口或PX4消息，便于直接在电脑上验证。
// 沿用VGSolar APM协议：[FF帧头][距离高字节][距离低字节][前三字节之和的低8位]。
class Parser
{
public:
	// 每调用一次输入一个字节。只有完整帧校验通过才返回true并更新millimetres。
	// now单位为微秒，是驱动读到字节的时间，不是传感器硬件采样时间。
	bool feed(uint8_t byte, uint64_t now, uint16_t &millimetres)
	{
		// 有未完成的半帧且后续字节迟到超过50ms时，放弃旧半帧，重新同步。
		if (_used && now - _last_byte > 50000) {
			_used = 0;
			++partial_timeouts;
		}

		_last_byte = now;

		// 尚未开始收帧时忽略噪声；开始收帧后，距离数据中的FF仍按普通数据处理。
		if (!_used && byte != 0xff) { return false; }

		_buffer[_used++] = byte;

		// 一次read可能只有部分数据，留下状态，等下一次补齐4字节。
		if (_used < sizeof(_buffer)) { return false; }

		if (static_cast<uint8_t>(_buffer[0] + _buffer[1] + _buffer[2]) == _buffer[3]) {
			// 距离为大端16位整数，单位mm；这里仅解包，量程检查交给Sample。
			millimetres = (static_cast<uint16_t>(_buffer[1]) << 8) | _buffer[2];
			_used = 0;
			return true;
		}

		++checksum_errors;

		// 校验失败后，当前4字节内可能已经包含下一帧开头。
		// 从第1字节起寻找FF并保留其后的字节，避免直接清空而漏掉重叠帧头。
		size_t next = 1;

		while (next < sizeof(_buffer) && _buffer[next] != 0xff) { ++next; }

		_used = sizeof(_buffer) - next;

		for (size_t i = 0; i < _used; ++i) { _buffer[i] = _buffer[next + i]; }

		return false;
	}

	uint32_t checksum_errors{0}; // 已收满4字节但校验不通过的次数。
	uint32_t partial_timeouts{0}; // 后续字节到达时发现半帧已过期的次数。
private:
	uint8_t _buffer[4] {}; // 当前候选帧；每个通道各有一份，左右互不干扰。
	size_t _used{0}; // 已缓存字节数，0表示正在寻找帧头。
	uint64_t _last_byte{0}; // 最近一次输入字节的时间，单位us。
};

// 样本状态层：把“收到完整帧”与“该距离当前可以使用”分开判断。
class Sample
{
public:
	// 输入新鲜度阈值500ms；它不是电机停车时间，也不控制执行器。
	static constexpr uint64_t TimeoutUs = 500000;
	// 包括越量程完整帧在内，都替换旧样本；避免新帧无效时仍使用旧有效距离。
	void receive(uint16_t mm, uint64_t now) { _mm = mm; _time = now; _have_frame = true; }
	// 三个条件必须同时满足：收到过完整帧、未到500ms、量程为30～4500mm。
	// 量程沿用APM工程配置；15cm边缘阈值属于后续业务判断。
	bool valid(uint64_t now) const { return _have_frame && now - _time < TimeoutUs && _mm >= 30 && _mm <= 4500; }
	// 原始最近样本值；调用方使用前仍须检查valid()。
	uint16_t mm() const { return _mm; }
	// 有效时毫米转米；无效时返回低于量程的0，并由驱动配套发布quality=0。
	// 使用有限值，避免MAVLink流直接将NaN转换成无符号厘米数；0不代表安全距离。
	float distance_m(uint64_t now) const { return valid(now) ? _mm * 0.001f : 0.f; }
private:
	uint64_t _time{0}; // 最近校验正确帧的接收时间，单位us。
	uint16_t _mm{0}; // 最近完整帧中的距离，单位mm，可能越量程。
	bool _have_frame{false}; // 区分“尚未收到数据”与“收到0mm无效数据”。
};
}
