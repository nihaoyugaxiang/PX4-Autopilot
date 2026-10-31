// SPDX-License-Identifier: BSD-3-Clause
// 测试真实纯计算代码，不访问GPIO/ADC，也不驱动任何输出。
#include "../InputProcessing.hpp"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
using namespace vgsolar_inputs;

static void warm(Pressure &p, float voltage, uint64_t start)
{
	for (unsigned i = 0; i <= 5; ++i) {
		const uint64_t stamp = start + i * 20000;
		p.sample(voltage, stamp, stamp);
	}
}

int main()
{
	// 端点/中点与不同ADC分辨率，先验证单位换算和报告元数据。
	assert(fabsf(adc_voltage(32768, 65536, 3.3f) - 1.65f) < 1e-6f);
	assert(fabsf(adc_voltage(2048, 4096, 3.3f) - 1.65f) < 1e-6f);
	assert(isnan(adc_voltage(-1, 65536, 3.3f)));
	assert(isnan(adc_voltage(65536, 65536, 3.3f)));
	assert(isnan(adc_voltage(0, 0, 3.3f)));
	assert(isnan(adc_voltage(100, 65536, NAN)));
	assert(isnan(adc_voltage(100, 65536, 0.f)));

	for (float voltage : {0.1f, 1.6f, 3.1f}) {
		Pressure p;
		p.configure(Calibration{});
		assert(!p.healthy());
		warm(p, voltage, 1000);
		assert(p.healthy());
		const float expected = (voltage - 0.1f) * (-100.f / 3.f);
		assert(fabsf(p.kpa() - expected) < 0.001f);
	}

	// 满100ms才恢复健康；重复消息不能伪造连续新采样。
	Pressure p;
	p.configure(Calibration{});
	p.sample(1.6f, 1000, 1000);
	p.sample(1.6f, 1000, 101000);
	assert(!p.healthy() && isnan(p.kpa()));
	p.sample(1.6f, 100999, 100999);
	assert(!p.healthy());
	p.sample(1.6f, 101000, 101000);
	assert(p.healthy());
	p.tick(350999);
	assert(p.healthy());
	p.tick(351000);
	assert(p.status() == PressureStatus::Stale && isnan(p.kpa()));
	p.sample(1.6f, 400000, 400000);
	assert(!p.healthy());
	warm(p, 1.6f, 420000);
	assert(p.healthy());

	// 单次无效也立即撤销健康，近电源轨和坏ADC不可继承旧压力。
	for (float invalid : {0.f, 0.01f, 3.29f, 3.3f, NAN}) {
		p.sample(invalid, 600000, 600000);
		assert(!p.healthy() && isnan(p.kpa()));
		warm(p, 1.6f, 620000);
		assert(p.healthy());
		p.configure(Calibration{});
		warm(p, 1.6f, 1000);
	}

	p.missing(150000);
	assert(p.status() == PressureStatus::MissingChannel && isnan(p.kpa()));
	p.tick(400000);
	assert(p.status() == PressureStatus::Stale);
	p.configure(Calibration{});
	p.sample(1.6f, 100000, 99999);
	assert(!p.healthy());
	p.sample(1.6f, 1000, 251000);
	assert(!p.healthy());

	// 配置错误必须暴露，不能静默回退或保持原先的健康输出。
	Calibration bad;
	bad.v_max = bad.v_min;
	p.configure(bad);
	warm(p, 1.6f, 1000);
	assert(p.status() == PressureStatus::InvalidConfig && isnan(p.kpa()));
	bad = Calibration{}; bad.p_max = bad.p_min;
	assert(!bad.valid());
	bad = Calibration{}; bad.filter_hz = NAN;
	assert(!bad.valid());
	bad = Calibration{}; bad.filter_hz = 21.f;
	assert(!bad.valid());
	Calibration unfiltered; unfiltered.filter_hz = 0.f;
	p.configure(unfiltered); warm(p, 0.1f, 1000);
	p.sample(3.1f, 121000, 121000);
	assert(fabsf(p.kpa() + 100.f) < 0.001f);
	p.configure(Calibration{}); warm(p, 0.1f, 1000);
	p.sample(3.1f, 121000, 121000);
	assert(p.kpa() < 0.f && p.kpa() > -100.f); // 5Hz滤波平滑阶跃。

	// 红外启动需稳定满30ms；小于30ms的毛刺不能改变已接受电平。
	Infrared ir;
	ir.configure(false, 30);
	ir.sample(false, 0);
	assert(!ir.valid(0));
	ir.sample(false, 29999);
	assert(!ir.valid(29999));
	ir.sample(false, 30000);
	assert(ir.valid(30000) && !ir.blade_present());
	ir.sample(true, 40000);
	ir.sample(false, 50000);
	assert(!ir.stable());
	ir.sample(true, 60000);
	ir.sample(true, 89999);
	assert(!ir.stable());
	ir.sample(true, 90000);
	assert(ir.stable() && ir.blade_present());
	assert(!ir.valid(340000));
	ir.sample(true, 400000);
	assert(!ir.valid(400000));
	ir.sample(true, 430000);
	assert(ir.valid(430000));
	ir.configure(true, 0);
	ir.sample(false, 0);
	assert(ir.valid(0) && ir.blade_present());
	ir.sample(true, 1000);
	assert(!ir.blade_present());
	puts("PASS: ADC resolutions/errors; pressure endpoints/warmup/duplicate/stale/invalid/config/filter/recovery; IR debounce/glitches/polarity/stale/restart");
}
