// SPDX-License-Identifier: BSD-3-Clause
// 仅采集左右轮AB相原始计数；不控制执行器，也不假定编码器每圈计数。
#include "QuadratureDecoder.hpp"
#include <board_config.h>
#include <px4_platform/gpio.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/vgsolar_encoder_status.h>
#include <string.h>

class VgsolarEncoders : public ModuleBase<VgsolarEncoders>, public px4::ScheduledWorkItem
{
public:
	VgsolarEncoders() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		_contexts[0] = {this, false};
		_contexts[1] = {this, false};
		_contexts[2] = {this, true};
		_contexts[3] = {this, true};
	}

	~VgsolarEncoders() override { stop_capture(); }

	static int task_spawn(int, char *[])
	{
		auto *instance = new VgsolarEncoders();

		if (!instance) { return PX4_ERROR; }

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (!instance->start_capture()) {
			_object.store(nullptr);
			_task_id = -1;
			delete instance;
			return PX4_ERROR;
		}

		instance->ScheduleOnInterval(20000); // 50Hz发布诊断；计数在GPIO边沿中断中更新。
		return PX4_OK;
	}

	static int custom_command(int, char *[]) { return print_usage("unknown command"); }

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_USAGE_NAME("vgsolar_encoders", "module");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	int print_status() override
	{
		uORB::Subscription sub{ORB_ID(vgsolar_encoder_status)};
		vgsolar_encoder_status_s data{};

		if (!sub.copy(&data)) { PX4_INFO("waiting for first publication"); return 0; }

		PX4_INFO("active=%u right=%" PRId64 " (delta=%" PRId32 ", dir=%" PRId8 ", invalid=%" PRIu32 ")",
			 (unsigned)data.capture_active, data.right_count, data.right_delta, data.right_direction,
			 data.right_invalid_transitions);
		PX4_INFO("left=%" PRId64 " (delta=%" PRId32 ", dir=%" PRId8 ", invalid=%" PRIu32 ")",
			 data.left_count, data.left_delta, data.left_direction, data.left_invalid_transitions);
		PX4_INFO("raw transitions only; direction sign is phase order, not verified chassis-forward direction");
		return 0;
	}

private:
	struct PinContext {
		VgsolarEncoders *owner{nullptr};
		bool left{false};
	};

	struct WheelSnapshot {
		vgsolar_encoders::DecoderSnapshot decoder{};
		uint64_t last_edge{0};
	};

	static int gpio_callback(int, void *, void *arg)
	{
		auto *context = static_cast<PinContext *>(arg);
		context->owner->sample_edge(context->left);
		return 0;
	}

	static uint32_t encoder_pin(unsigned index)
	{
		switch (index) {
		case 0: return GPIO_VG_ENCODER_RIGHT_A;
		case 1: return GPIO_VG_ENCODER_RIGHT_B;
		case 2: return GPIO_VG_ENCODER_LEFT_A;
		default: return GPIO_VG_ENCODER_LEFT_B;
		}
	}

	uint8_t read_phase(bool left) const
	{
		const uint32_t pin_a = left ? GPIO_VG_ENCODER_LEFT_A : GPIO_VG_ENCODER_RIGHT_A;
		const uint32_t pin_b = left ? GPIO_VG_ENCODER_LEFT_B : GPIO_VG_ENCODER_RIGHT_B;
		const uint8_t a = px4_arch_gpioread(pin_a) ? 1 : 0;
		const uint8_t b = px4_arch_gpioread(pin_b) ? 1 : 0;
		return (a << 1) | b;
	}

	bool start_capture()
	{
		for (unsigned i = 0; i < 4; ++i) { px4_arch_configgpio(encoder_pin(i)); }

		// 台架启动时应先让车轮静止；以当前AB电平初始化，之后只计新的有效相位跳变。
		_right.reset(read_phase(false));
		_left.reset(read_phase(true));
		_right_last_edge = 0;
		_left_last_edge = 0;
		_attached_mask = 0;

		for (unsigned i = 0; i < 4; ++i) {
			if (px4_arch_gpiosetevent(encoder_pin(i), true, true, false, gpio_callback, &_contexts[i]) != PX4_OK) {
				PX4_ERR("failed to attach encoder GPIO interrupt for pin index %u", i);
				stop_capture();
				return false;
			}

			_attached_mask |= 1u << i;
		}

		_active = true;
		return true;
	}

	void stop_capture()
	{
		_active = false;

		for (unsigned i = 0; i < 4; ++i) {
			if ((_attached_mask & (1u << i)) != 0) {
				px4_arch_gpiosetevent(encoder_pin(i), false, false, false, nullptr, nullptr);
			}
		}

		_attached_mask = 0;
	}

	void sample_edge(bool left)
	{
		// 工作队列读取快照时屏蔽中断，确保相位、计数和时间戳来自同一份状态。
		irqstate_t flags = px4_enter_critical_section();
		const uint8_t phase = read_phase(left);

		if (left && _left.update(phase) != 0) {
			_left_last_edge = hrt_absolute_time();
		} else if (!left && _right.update(phase) != 0) {
			_right_last_edge = hrt_absolute_time();
		}

		px4_leave_critical_section(flags);
	}

	static int32_t clamp_delta(int64_t value)
	{
		if (value > INT32_MAX) { return INT32_MAX; }
		if (value < INT32_MIN) { return INT32_MIN; }
		return static_cast<int32_t>(value);
	}

	void Run() override
	{
		if (should_exit()) {
			stop_capture();
			publish_debug(publish_status());
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		const auto report = publish_status();

		if (++_debug_cycle >= 5) {
			_debug_cycle = 0;
			publish_debug(report);
		}
	}

	vgsolar_encoder_status_s publish_status()
	{
		// 同一临界区复制两轮，取时间戳后再恢复中断，避免样本时间晚于发布快照。
		irqstate_t flags = px4_enter_critical_section();
		const WheelSnapshot right{_right.snapshot(), _right_last_edge};
		const WheelSnapshot left{_left.snapshot(), _left_last_edge};
		const uint64_t now = hrt_absolute_time();
		const bool active = _active;
		px4_leave_critical_section(flags);
		vgsolar_encoder_status_s report{};
		report.timestamp = now;
		report.timestamp_sample = right.last_edge > left.last_edge ? right.last_edge : left.last_edge;
		report.right_count = right.decoder.count;
		report.left_count = left.decoder.count;
		report.right_delta = clamp_delta(report.right_count - _previous_right_count);
		report.left_delta = clamp_delta(report.left_count - _previous_left_count);
		report.right_transitions = right.decoder.transitions;
		report.left_transitions = left.decoder.transitions;
		report.right_invalid_transitions = right.decoder.invalid_transitions;
		report.left_invalid_transitions = left.decoder.invalid_transitions;
		report.right_direction = report.right_delta > 0 ? 1 : (report.right_delta < 0 ? -1 : 0);
		report.left_direction = report.left_delta > 0 ? 1 : (report.left_delta < 0 ? -1 : 0);
		report.capture_active = active;
		_encoder_pub.publish(report);
		_previous_right_count = report.right_count;
		_previous_left_count = report.left_count;
		return report;
	}

	void publish_debug(const vgsolar_encoder_status_s &report)
	{
		debug_array_s debug{};
		debug.timestamp = report.timestamp;
		debug.id = 1;
		strncpy(debug.name, "VGS_ENC", sizeof(debug.name));
		debug.name[sizeof(debug.name) - 1] = '\0';
		debug.data[0] = static_cast<float>(report.right_count);
		debug.data[1] = static_cast<float>(report.left_count);
		debug.data[2] = static_cast<float>(static_cast<uint64_t>(report.right_invalid_transitions) + report.left_invalid_transitions);
		debug.data[3] = report.capture_active ? 1.f : 0.f;
		_debug_pub.publish(debug); // 0=右计数，1=左计数，2=无效跳变总数，3=采集活动标志。
	}

	uORB::Publication<vgsolar_encoder_status_s> _encoder_pub{ORB_ID(vgsolar_encoder_status)};
	uORB::Publication<debug_array_s> _debug_pub{ORB_ID(debug_array)};
	vgsolar_encoders::QuadratureDecoder _right;
	vgsolar_encoders::QuadratureDecoder _left;
	PinContext _contexts[4]{};
	uint64_t _right_last_edge{0};
	uint64_t _left_last_edge{0};
	int64_t _previous_right_count{0};
	int64_t _previous_left_count{0};
	uint8_t _debug_cycle{0};
	uint8_t _attached_mask{0};
	bool _active{false};
};

extern "C" __EXPORT int vgsolar_encoders_main(int argc, char *argv[])
{
	return VgsolarEncoders::main(argc, argv);
}
