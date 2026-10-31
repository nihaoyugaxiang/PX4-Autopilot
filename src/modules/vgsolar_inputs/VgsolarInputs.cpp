// SPDX-License-Identifier: BSD-3-Clause
// 只读吸盘输入模块：订阅板载ADC、读取红外GPIO，不写PWM/阀/泵。
#include "InputProcessing.hpp"
#include <board_config.h>
#include <drivers/drv_adc.h>
#include <px4_platform/gpio.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vgsolar_inputs.h>
#include <string.h>

#ifndef GPIO_VG_IR_INPUT
#error "vgsolar_inputs requires VGSolar board pressure and infrared pin definitions"
#endif

using vgsolar_inputs::PressureStatus;
static_assert(static_cast<uint8_t>(PressureStatus::Disabled) == vgsolar_inputs_s::PRESSURE_DISABLED, "message status mismatch");
static_assert((ADC_CHANNELS & (1u << VGSOLAR_PRESSURE_ADC_CHANNEL)) != 0, "pressure channel absent from board ADC scan");

class VgsolarInputs : public ModuleBase<VgsolarInputs>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VgsolarInputs() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		apply_parameters();
	}
	static int task_spawn(int, char *[])
	{
		auto *instance = new VgsolarInputs();

		if (!instance) { return PX4_ERROR; }

		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleOnInterval(20000); // 50Hz处理/发布，ADC本身仍由board_adc采集。
		return PX4_OK;
	}
	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_USAGE_NAME("vgsolar_inputs", "module");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}
	int print_status() override
	{
		// 从uORB获取一致的消息快照，避免直接跨线程读取内部计算状态。
		uORB::Subscription sub{ORB_ID(vgsolar_inputs)};
		vgsolar_inputs_s data{};

		if (!sub.copy(&data)) { PX4_INFO("waiting for first publication"); return 0; }

		PX4_INFO("PC0 ADC%u: raw=%" PRId32 " resolution=%" PRIu32 " Vref=%.3f V", data.pressure_adc_channel,
			 data.adc_raw, data.adc_resolution, (double)data.adc_v_ref);
		PX4_INFO("voltage=%.4f V raw=%.2f kPa filtered=%.2f kPa valid=%u status=%u",
			 (double)data.pressure_voltage_v, (double)data.pressure_raw_kpa, (double)data.pressure_kpa,
			 (unsigned)data.pressure_valid, data.pressure_status);
		PX4_INFO("PB5: raw=%u debounced=%u blade_present=%u sample_valid=%u",
			 (unsigned)data.ir_raw, (unsigned)data.ir_debounced, (unsigned)data.ir_blade_present, (unsigned)data.ir_sample_valid);
		PX4_INFO("GPIO sample validity does not prove wiring or lift position; input only");
		return 0;
	}
private:
	void apply_parameters()
	{
		updateParams();
		vgsolar_inputs::Calibration cal;
		cal.v_min = _param_v_min.get();
		cal.v_max = _param_v_max.get();
		cal.p_min = _param_p_min.get();
		cal.p_max = _param_p_max.get();
		cal.filter_hz = _param_filter.get();
		static_assert(sizeof(cal) == 5 * sizeof(float), "calibration comparison requires no padding");

		// 其他模块的参数变化不应反复重置压力预热与红外消抖。
		if (!_configured || memcmp(&cal, &_last_cal, sizeof(cal)) != 0) {
			_pressure.configure(cal);
			_last_cal = cal;
		}

		const int invert = _param_ir_invert.get();
		const int debounce = _param_ir_debounce.get();

		if (!_configured || invert != _last_invert || debounce != _last_debounce) {
			_ir_config_valid = (invert == 0 || invert == 1) && debounce >= 0 && debounce <= 500;
			_ir.configure(invert == 1, _ir_config_valid ? debounce : 30);
			_last_invert = invert;
			_last_debounce = debounce;
		}

		_configured = true;
	}

	void Run() override
	{
		if (should_exit()) {
			// stop时主动撤销有效性；若模块意外中断，消费者仍须自己检查timestamp新鲜度。
			vgsolar_inputs_s stopped{};
			stopped.timestamp = hrt_absolute_time();
			stopped.pressure_status = vgsolar_inputs_s::PRESSURE_DISABLED;
			stopped.pressure_voltage_v = stopped.pressure_raw_kpa = stopped.pressure_kpa = NAN;
			stopped.adc_raw = -1;
			stopped.pressure_adc_channel = VGSOLAR_PRESSURE_ADC_CHANNEL;
			_pub.publish(stopped);
			publish_debug_vect(stopped.timestamp, NAN, NAN, false);
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		parameter_update_s update{};

		if (_parameter_sub.update(&update)) { apply_parameters(); }

		adc_report_s adc{};
		const bool adc_updated = _adc_sub.update(&adc);
		// 取消息后再读当前时间，避免更高优先级ADC刚发布的数据被误判为来自未来。
		const hrt_abstime now = hrt_absolute_time();

		if (adc_updated) {
			_adc_raw = -1;
			_adc_resolution = adc.resolution;
			_adc_reference = adc.v_ref;
			bool found = false;

			// 只消费板内ADC的PC0/通道10，避免将别的ADC源或数组下标当成压力通道。
			if (adc.device_id == BUILTIN_ADC_DEVID) {
				for (unsigned i = 0; i < sizeof(adc.channel_id) / sizeof(adc.channel_id[0]); ++i) {
					if (adc.channel_id[i] == VGSOLAR_PRESSURE_ADC_CHANNEL) {
						_adc_raw = adc.raw_data[i];
						_pressure.sample(vgsolar_inputs::adc_voltage(_adc_raw, adc.resolution, adc.v_ref), adc.timestamp, now);
						found = true;
						break;
					}
				}
			}

			if (!found) { _pressure.missing(adc.timestamp); }
		}

		_pressure.tick(now); // ADC中断时仍周期撤销旧压力，不能只在收到新帧时检查。
		_ir.sample(px4_arch_gpioread(GPIO_VG_IR_INPUT), now);
		vgsolar_inputs_s report{};
		report.timestamp = now;
		report.timestamp_sample = _pressure.timestamp();
		report.adc_raw = _adc_raw;
		report.adc_resolution = _adc_resolution;
		report.adc_v_ref = _adc_reference;
		report.pressure_voltage_v = _pressure.voltage();
		report.pressure_raw_kpa = _pressure.raw_kpa();
		report.pressure_kpa = _pressure.kpa();
		report.pressure_status = static_cast<uint8_t>(_pressure.status());
		report.pressure_valid = _pressure.healthy();
		report.ir_raw = _ir.raw();
		report.ir_debounced = _ir.stable();
		report.ir_sample_valid = _ir_config_valid && _ir.valid(now);
		report.ir_blade_present = report.ir_sample_valid && _ir.blade_present();
		report.pressure_adc_channel = VGSOLAR_PRESSURE_ADC_CHANNEL;
		report.enabled = true;
		_pub.publish(report);
		publish_debug_vect(now, report.pressure_voltage_v, report.pressure_kpa, report.pressure_valid);
	}

	void publish_debug_vect(uint64_t timestamp, float voltage, float pressure_kpa, bool valid)
	{
		// QGC MAVLink Inspector显示：x=PC0电压(V)，y=滤波压力(kPa)，z=压力有效标志。
		debug_vect_s debug{};
		debug.timestamp = timestamp;
		strncpy(debug.name, "VGS_PRESS", sizeof(debug.name));
		debug.name[sizeof(debug.name) - 1] = '\0';
		debug.x = voltage;
		debug.y = pressure_kpa;
		debug.z = valid ? 1.f : 0.f;
		_debug_pub.publish(debug);
	}

	uORB::Subscription _adc_sub{ORB_ID(adc_report)};
	uORB::Subscription _parameter_sub{ORB_ID(parameter_update)};
	uORB::Publication<vgsolar_inputs_s> _pub{ORB_ID(vgsolar_inputs)};
	uORB::Publication<debug_vect_s> _debug_pub{ORB_ID(debug_vect)};
	vgsolar_inputs::Pressure _pressure;
	vgsolar_inputs::Infrared _ir;
	vgsolar_inputs::Calibration _last_cal;
	bool _configured{false}, _ir_config_valid{false};
	int _last_invert{0}, _last_debounce{30};
	int32_t _adc_raw{-1};
	uint32_t _adc_resolution{0};
	float _adc_reference{NAN};
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VGS_P_VMIN>) _param_v_min,
		(ParamFloat<px4::params::VGS_P_VMAX>) _param_v_max,
		(ParamFloat<px4::params::VGS_P_PMIN>) _param_p_min,
		(ParamFloat<px4::params::VGS_P_PMAX>) _param_p_max,
		(ParamFloat<px4::params::VGS_P_FILT>) _param_filter,
		(ParamInt<px4::params::VGS_IR_INV>) _param_ir_invert,
		(ParamInt<px4::params::VGS_IR_DEB>) _param_ir_debounce
	)
};

extern "C" __EXPORT int vgsolar_inputs_main(int argc, char *argv[])
{
	return VgsolarInputs::main(argc, argv);
}
