// SPDX-License-Identifier: BSD-3-Clause
// 只读订阅DYP-A02距离，发布边缘安全请求；不写PWM，不发解锁/上锁/终止命令。
#include "SafetyProcessing.hpp"
#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vgsolar_safety_status.h>
#include <uORB/topics/vgsolar_safety_request.h>
#include <string.h>

using vgsolar_safety::State;
static_assert(static_cast<uint8_t>(State::Safe) == vgsolar_safety_status_s::STATE_SAFE, "state mismatch");
static_assert(static_cast<uint8_t>(State::Pending) == vgsolar_safety_status_s::STATE_PENDING, "state mismatch");
static_assert(static_cast<uint8_t>(State::Latched) == vgsolar_safety_status_s::STATE_LATCHED, "state mismatch");
static_assert(static_cast<uint8_t>(State::ConfigInvalid) == vgsolar_safety_status_s::STATE_CONFIG_INVALID, "state mismatch");

class VgsolarSafety : public ModuleBase<VgsolarSafety>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VgsolarSafety() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		apply_parameters();
	}

	static int task_spawn(int, char *[])
	{
		auto *instance = new VgsolarSafety();

		if (!instance) { return PX4_ERROR; }

		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleOnInterval(20000); // 50Hz软件检查，不是传感器采样率或硬实时保证。
		return PX4_OK;
	}

	static int custom_command(int argc, char *argv[])
	{
		if (argc == 1 && strcmp(argv[0], "clear") == 0) {
			auto *instance = get_instance();

			if (!instance) { PX4_ERR("not running"); return PX4_ERROR; }

			// 命令线程只写原子请求，实际判断和修改锁存由工作队列完成。
			instance->_clear_requested.store(1);
			PX4_INFO("clear queued for one cycle; check status/clear counters for result");
			return PX4_OK;
		}

		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_USAGE_NAME("vgsolar_safety", "module");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_COMMAND("clear");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	int print_status() override
	{
		// 读取一致的uORB快照，避免status线程直接访问运行中的状态机。
		uORB::Subscription sub{ORB_ID(vgsolar_safety_status)};
		vgsolar_safety_status_s data{};

		if (!sub.copy(&data)) { PX4_INFO("waiting for first publication"); return 0; }

		PX4_INFO("state=%s enabled=%u request=%u latched=%u config_valid=%u", state_name(data.state),
			 (unsigned)data.enabled, (unsigned)data.stop_request, (unsigned)data.latched, (unsigned)data.config_valid);
		PX4_INFO("left(ttyS7/7)=%.3f m valid=%u unsafe=%u; right(ttyS1/1)=%.3f m valid=%u unsafe=%u",
			 (double)data.left_distance_m, (unsigned)data.left_valid, (unsigned)data.left_unsafe,
			 (double)data.right_distance_m, (unsigned)data.right_valid, (unsigned)data.right_unsafe);
		PX4_INFO("limit=%.1f cm debounce=%" PRIu32 " ms unsafe=%" PRIu32 " ms clear_eligible=%u",
			 (double)data.max_distance_cm, data.debounce_ms, data.unsafe_duration_ms, (unsigned)data.clear_eligible);
		PX4_INFO("clear_count=%" PRIu32 " rejected_clear_count=%" PRIu32, data.clear_count, data.rejected_clear_count);
		PX4_INFO("REQUEST ONLY: no actuator stop consumer; no PWM or commander command");
		return 0;
	}

private:
	static const char *state_name(uint8_t state)
	{
		switch (state) {
		case vgsolar_safety_status_s::STATE_SAFE: return "SAFE";
		case vgsolar_safety_status_s::STATE_PENDING: return "PENDING";
		case vgsolar_safety_status_s::STATE_LATCHED: return "LATCHED";
		case vgsolar_safety_status_s::STATE_CONFIG_INVALID: return "CONFIG_INVALID";
		case vgsolar_safety_status_s::STATE_STOPPED: return "STOPPED";
		default: return "WAITING";
		}
	}

	void apply_parameters()
	{
		updateParams();
		_monitor.configure(_param_max_distance.get(), _param_debounce.get());
	}

	static vgsolar_safety::RangeSample to_sample(const distance_sensor_s &data)
	{
		vgsolar_safety::RangeSample sample{};
		sample.received = true;
		sample.timestamp = data.timestamp;
		sample.min_distance = data.min_distance;
		sample.max_distance = data.max_distance;
		sample.distance = data.current_distance;
		sample.signal_quality = data.signal_quality;
		sample.type = data.type;
		sample.orientation = data.orientation;
		sample.mode = data.mode;
		return sample;
	}

	void Run() override
	{
		if (should_exit()) {
			// stop保留最后锁存值，但enabled=false；未来消费者仍须自行检查新鲜度/启用状态。
			_last_status.timestamp = hrt_absolute_time();
			_last_status.state = vgsolar_safety_status_s::STATE_STOPPED;
			_last_status.enabled = false;
			_status_pub.publish(_last_status);
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		parameter_update_s update{};

		if (_parameter_sub.update(&update)) { apply_parameters(); }

		for (auto &sub : _distance_subs) {
			distance_sensor_s data{};

			if (!sub.update(&data)) { continue; }

			// 按设备类型+串口总线+方向识别，不依赖不稳定的distance_sensor实例编号。
			// 当前板级接线固定为左UART8/ttyS7(7)、右USART2/ttyS1(1)。
			device::Device::DeviceId id{};
			id.devid = data.device_id;

			if (id.devid_s.devtype != DRV_DIST_DEVTYPE_DYP_A02
			    || id.devid_s.bus_type != device::Device::DeviceBusType_SERIAL) { continue; }

			if (id.devid_s.bus == 7 && data.orientation == 7) { _left = to_sample(data); }
			else if (id.devid_s.bus == 1 && data.orientation == 1) { _right = to_sample(data); }
		}

		// 读完消息再取当前时间；未来/陈旧时间戳不能使通道恢复有效。
		vgsolar_safety_request_s request{};
		const uint64_t now = hrt_absolute_time();
		const bool ncu_clear = _clear_sub.update(&request) && request.clear && request.timestamp > 0
				       && now >= request.timestamp && now - request.timestamp < 200000;
		const auto result = _monitor.update(_left, _right, now, _clear_requested.fetch_and(0) != 0 || ncu_clear);

		if (result.clear_attempted) {
			if (result.clear_succeeded) {
				++_clear_count;
				PX4_INFO("request cleared; both readings are currently safe");
			} else {
				++_rejected_clear_count;
				PX4_WARN("clear rejected: require a latch, valid config and both readings safe");
			}
		}

		vgsolar_safety_status_s report{};
		report.timestamp = result.timestamp;
		report.left_timestamp = result.left_timestamp;
		report.right_timestamp = result.right_timestamp;
		report.left_distance_m = result.left_distance_m;
		report.right_distance_m = result.right_distance_m;
		report.max_distance_cm = result.max_distance_cm;
		report.debounce_ms = result.debounce_ms;
		report.unsafe_duration_ms = result.unsafe_duration_ms;
		report.clear_count = _clear_count;
		report.rejected_clear_count = _rejected_clear_count;
		report.state = static_cast<uint8_t>(result.state);
		report.config_valid = result.config_valid;
		report.left_valid = result.left_valid;
		report.right_valid = result.right_valid;
		report.left_unsafe = result.left_unsafe;
		report.right_unsafe = result.right_unsafe;
		report.clear_eligible = result.clear_eligible;
		report.latched = result.latched;
		report.stop_request = result.stop_request;
		report.enabled = true;
		_status_pub.publish(report);
		_last_status = report;
	}

	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_subs{ORB_ID::distance_sensor};
	uORB::Subscription _parameter_sub{ORB_ID(parameter_update)};
	uORB::Publication<vgsolar_safety_status_s> _status_pub{ORB_ID(vgsolar_safety_status)};
	vgsolar_safety::Monitor _monitor;
	vgsolar_safety::RangeSample _left{}, _right{};
	vgsolar_safety_status_s _last_status{};
	uORB::Subscription _clear_sub{ORB_ID(vgsolar_safety_request)};
	px4::atomic<uint32_t> _clear_requested{0};
	uint32_t _clear_count{0}, _rejected_clear_count{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VGS_RF_MAX>) _param_max_distance,
		(ParamInt<px4::params::VGS_RF_DBNC>) _param_debounce
	)
};

extern "C" __EXPORT int vgsolar_safety_main(int argc, char *argv[])
{
	return VgsolarSafety::main(argc, argv);
}
