// SPDX-License-Identifier: BSD-3-Clause
// 纯计算层主机测试：覆盖距离边界、消抖、无效输入、锁存和受限清除。
#include "../SafetyProcessing.hpp"
#include <assert.h>
#include <stdio.h>

using namespace vgsolar_safety;

static RangeSample sample(float distance, uint64_t timestamp, uint8_t orientation)
{
	RangeSample value{};
	value.received = true;
	value.timestamp = timestamp;
	value.min_distance = 0.03f;
	value.max_distance = 4.5f;
	value.distance = distance;
	value.signal_quality = -1;
	value.type = 1;
	value.orientation = orientation;
	value.mode = 1;
	return value;
}

int main()
{
	Monitor monitor;
	RangeSample left = sample(0.15f, 1000, 7);
	RangeSample right = sample(0.10f, 1000, 1);
	Status status = monitor.update(left, right, 1000);
	assert(status.state == State::Safe && !status.stop_request && status.clear_eligible);

	// 等于阈值安全；超过阈值的异常必须连续达到100ms。
	left = sample(0.1501f, 2000, 7);
	status = monitor.update(left, right, 2000);
	assert(status.state == State::Pending && !status.stop_request);
	status = monitor.update(left, right, 101999);
	assert(status.state == State::Pending && !status.stop_request);
	status = monitor.update(left, right, 102000);
	assert(status.state == State::Latched && status.stop_request && status.left_unsafe);

	// 安全读数不能自动解锁；危险期间发送clear不会被延迟执行。
	left = sample(0.10f, 103000, 7);
	right = sample(0.12f, 103000, 1);
	status = monitor.update(left, right, 103000);
	assert(status.state == State::Latched && status.stop_request);
	left = sample(0.20f, 104000, 7);
	status = monitor.update(left, right, 104000, true);
	assert(status.clear_attempted && !status.clear_succeeded && status.stop_request);

	// 只有发送显式清除且两路都有效、未超限时才解除锁存。
	left = sample(0.15f, 105000, 7);
	status = monitor.update(left, right, 105000, true);
	assert(status.clear_attempted && status.clear_succeeded && !status.stop_request);
	assert(status.state == State::Safe);

	// 50ms越界毛刺恢复后重置计时，不会错误锁存。
	Monitor transient;
	left = sample(0.30f, 2000, 7);
	right = sample(0.10f, 2000, 1);
	transient.update(left, right, 2000);
	left = sample(0.10f, 52000, 7);
	right = sample(0.10f, 52000, 1);
	status = transient.update(left, right, 52000);
	assert(status.state == State::Safe && !status.stop_request);

	// 缺失侧和陈旧样本均作为不安全输入，并按同一100ms规则锁存。
	Monitor missing;
	RangeSample absent{};
	right = sample(0.10f, 1000, 1);
	status = missing.update(absent, right, 1000);
	assert(status.state == State::Pending && !status.stop_request);
	status = missing.update(absent, right, 101000);
	assert(status.state == State::Latched && status.stop_request);

	Monitor stale;
	left = sample(0.10f, 1000, 7);
	right = sample(0.10f, 1000, 1);
	stale.update(left, right, 1000);
	status = stale.update(left, right, 501000);
	assert(status.state == State::Pending && !status.left_valid && !status.right_valid);
	status = stale.update(left, right, 601000);
	assert(status.state == State::Latched && status.stop_request);

	// 未来时间戳、错误方向、零质量和无效质量均不能被当成有效测距。
	Monitor invalid;
	left = sample(0.10f, 1001, 7);
	right = sample(0.10f, 900, 7);
	status = invalid.update(left, right, 1000);
	assert(!status.left_valid && !status.right_valid);
	right = sample(0.10f, 1000, 1);
	right.signal_quality = 0;
	status = invalid.update(left, right, 1000);
	assert(!status.right_valid);
	right.signal_quality = -2;
	status = invalid.update(left, right, 1000);
	assert(!status.right_valid);

	// 非法参数直接锁存并报告配置故障，不能通过清除命令隐藏。
	Monitor bad_config;
	bad_config.configure(0.f, 0);
	status = bad_config.update(absent, absent, 1000, true);
	assert(!status.config_valid && !status.clear_eligible && status.stop_request);
	assert(status.state == State::ConfigInvalid && !status.clear_succeeded);

	puts("PASS: threshold/debounce; invalid, missing, stale and future samples; latched request; guarded clear; invalid config");
}
