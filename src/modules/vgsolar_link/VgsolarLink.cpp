// SPDX-License-Identifier: BSD-3-Clause
// USART3上的NCU只读链路：回传状态并拒绝全部控制/参数命令。
#include "Protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <string.h>

class VgsolarLink : public ModuleBase<VgsolarLink>, public px4::ScheduledWorkItem
{
public:
	VgsolarLink() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~VgsolarLink() override { close_uart(); }

	static int task_spawn(int, char *[])
	{
		auto *instance = new VgsolarLink();
		if (!instance) { return PX4_ERROR; }
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (!instance->open_uart()) {
			_object.store(nullptr);
			_task_id = -1;
			delete instance;
			return PX4_ERROR;
		}
		instance->ScheduleOnInterval(10000); // 100Hz解析；周期状态帧在内部限为10Hz。
		return PX4_OK;
	}

	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }
		PRINT_MODULE_USAGE_NAME("vgsolar_link", "module");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("uart=%s active=%u rx=%" PRIu32 " bad_checksum=%" PRIu32 " bad_length=%" PRIu32
			 " timeout=%" PRIu32 " rejected=%" PRIu32 " tx=%" PRIu32 " tx_errors=%" PRIu32 " rx_errors=%" PRIu32,
			 SERIAL_PATH, _active.load(), _rx_frames.load(), _bad_checksum.load(), _bad_length.load(), _timeouts.load(),
			 _rejected_commands.load(), _tx_frames.load(), _tx_errors.load(), _rx_errors.load());
		PX4_INFO("read-only: no motion, emergency-stop, parameter write or reboot command is executed");
		return 0;
	}

private:
	bool open_uart()
	{
		_fd = ::open(SERIAL_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (_fd < 0) { PX4_ERR("cannot open %s", SERIAL_PATH); return false; }
		termios config{};
		if (tcgetattr(_fd, &config) != 0) { PX4_ERR("tcgetattr failed for %s", SERIAL_PATH); close_uart(); return false; }
		cfmakeraw(&config);
		cfsetispeed(&config, B115200);
		cfsetospeed(&config, B115200);
		config.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
		#ifdef CRTSCTS
		config.c_cflag &= ~CRTSCTS;
		#endif
		config.c_cflag |= CS8 | CLOCAL | CREAD;
		config.c_cc[VMIN] = 0;
		config.c_cc[VTIME] = 0;
		if (tcsetattr(_fd, TCSANOW, &config) != 0 || tcflush(_fd, TCIFLUSH) != 0) {
			PX4_ERR("UART setup failed for %s", SERIAL_PATH);
			close_uart();
			return false;
		}
		PX4_INFO("read-only NCU link opened %s at 115200 8N1", SERIAL_PATH);
		_active.store(true);
		return true;
	}

	void close_uart()
	{
		if (_fd >= 0) { ::close(_fd); _fd = -1; }
		_active.store(false);
	}

	static int32_t scale_i32(double value, double multiplier)
	{
		if (!PX4_ISFINITE(value)) { return 0; }
		const double scaled = value * multiplier;
		if (scaled > INT32_MAX) { return INT32_MAX; }
		if (scaled < INT32_MIN) { return INT32_MIN; }
		return static_cast<int32_t>(lround(scaled));
	}

	static int16_t scale_i16(double value, double multiplier)
	{
		if (!PX4_ISFINITE(value)) { return 0; }
		const double scaled = value * multiplier;
		if (scaled > INT16_MAX) { return INT16_MAX; }
		if (scaled < INT16_MIN) { return INT16_MIN; }
		return static_cast<int16_t>(lround(scaled));
	}

	static uint16_t distance_cm(const distance_sensor_s &data)
	{
		if (data.type != distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND
		    || data.mode == distance_sensor_s::MODE_DISABLED || data.signal_quality == 0
		    || !PX4_ISFINITE(data.current_distance) || data.current_distance < data.min_distance
		    || data.current_distance > data.max_distance) {
			return vgsolar_link::RANGE_INVALID_CM;
		}
		return static_cast<uint16_t>(math::constrain(lroundf(data.current_distance * 100.f), 0l, 65534l));
	}

	vgsolar_link::StatusFields collect_status()
	{
		using namespace vgsolar_link;
		StatusFields fields{};
		fields.left_out_cm = fields.left_in_cm = fields.right_in_cm = fields.right_out_cm = RANGE_INVALID_CM;
		const uint64_t now = hrt_absolute_time();

		battery_status_s battery{};
		if (_battery_sub.copy(&battery) && battery.timestamp != 0 && now >= battery.timestamp
		    && now - battery.timestamp < 1000000 && PX4_ISFINITE(battery.remaining)
		    && battery.remaining >= 0.f && battery.remaining <= 1.f) {
			fields.battery_percent = static_cast<uint8_t>(lroundf(battery.remaining * 100.f));
		}

		vehicle_local_position_s local{};
		bool local_heading_valid = false;
		if (_local_position_sub.copy(&local) && local.timestamp != 0 && now >= local.timestamp && now - local.timestamp < 500000) {
			if (PX4_ISFINITE(local.heading)) {
				float heading = math::degrees(local.heading);
				if (heading < 0.f) { heading += 360.f; }
				fields.heading = static_cast<uint16_t>(lroundf(heading * 100.f)) % 36000;
				local_heading_valid = true;
			}
			if (local.v_xy_valid && PX4_ISFINITE(local.vx) && PX4_ISFINITE(local.vy)) {
				fields.velocity = scale_i16(hypotf(local.vx, local.vy), 100.0);
			}
		}

		vehicle_attitude_s attitude{};
		if (_attitude_sub.copy(&attitude) && attitude.timestamp != 0 && now >= attitude.timestamp
		    && now - attitude.timestamp < 500000) {
			const matrix::Eulerf euler{matrix::Quatf{attitude.q}};
			fields.roll = scale_i16(math::degrees(euler.phi()), 100.0);
			fields.pitch = scale_i16(math::degrees(euler.theta()), 100.0);
			if (!local_heading_valid && PX4_ISFINITE(euler.psi())) {
				float heading = math::degrees(euler.psi());
				if (heading < 0.f) { heading += 360.f; }
				fields.heading = static_cast<uint16_t>(lroundf(heading * 100.f)) % 36000;
			}
		}

		sensor_gps_s gps{};
		if (_gps_sub.copy(&gps) && gps.timestamp != 0 && now >= gps.timestamp && now - gps.timestamp < 1000000) {
			if (gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && gps.fix_type <= sensor_gps_s::FIX_TYPE_RTK_FIXED) {
				fields.longitude = scale_i32(gps.longitude_deg, 10000000.0);
				fields.latitude = scale_i32(gps.latitude_deg, 10000000.0);
				fields.gps_status = gps.fix_type == sensor_gps_s::FIX_TYPE_RTK_FIXED ? 3 :
					((gps.fix_type == sensor_gps_s::FIX_TYPE_3D || gps.fix_type == sensor_gps_s::FIX_TYPE_RTK_FLOAT
					  || gps.fix_type == sensor_gps_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL) ? 2 : 1);
			}
		}

		for (auto &sub : _distance_subs) {
			distance_sensor_s data{};
			if (!sub.copy(&data) || data.timestamp == 0 || now < data.timestamp || now - data.timestamp > 500000) { continue; }
			device::Device::DeviceId id{};
			id.devid = data.device_id;
			if (id.devid_s.devtype != DRV_DIST_DEVTYPE_DYP_A02 || id.devid_s.bus_type != device::Device::DeviceBusType_SERIAL) { continue; }
			const uint16_t cm = distance_cm(data);
			if (id.devid_s.bus == 7 && data.orientation == 7) { fields.left_out_cm = fields.left_in_cm = cm; }
			else if (id.devid_s.bus == 1 && data.orientation == 1) { fields.right_in_cm = fields.right_out_cm = cm; }
		}
		// 本阶段未标定轮速、未实现控制状态；按 APM 状态帧字段发送静止/待机。
		fields.left_track_velocity = 0;
		fields.right_track_velocity = 0;
		fields.control_mode = 0;
		fields.motion_state = 0;
		fields.fault_code = 0;
		return fields;
	}

	bool queue_tx_frame(const uint8_t *frame, size_t length)
	{
		if (_tx_offset != 0) {
			const size_t remaining = _tx_length - _tx_offset;
			memmove(_tx_buffer, _tx_buffer + _tx_offset, remaining);
			_tx_length = remaining;
			for (size_t i = 0; i < _tx_queued_frames; ++i) { _tx_frame_ends[i] -= _tx_offset; }
			_tx_offset = 0;
		}
		if (length > sizeof(_tx_buffer) - _tx_length || _tx_queued_frames >= TX_FRAME_QUEUE_CAPACITY) {
			_tx_errors.fetch_add(1);
			return false;
		}
		memcpy(_tx_buffer + _tx_length, frame, length);
		_tx_length += length;
		_tx_frame_ends[_tx_queued_frames++] = _tx_length;
		return true;
	}

	void flush_tx()
	{
		while (_tx_offset < _tx_length) {
			const ssize_t written = ::write(_fd, _tx_buffer + _tx_offset, _tx_length - _tx_offset);
			if (written > 0) {
				_tx_offset += static_cast<size_t>(written);
				while (_tx_queued_frames > 0 && _tx_frame_ends[0] <= _tx_offset) {
					_tx_frames.fetch_add(1);
					for (size_t i = 1; i < _tx_queued_frames; ++i) { _tx_frame_ends[i - 1] = _tx_frame_ends[i]; }
					--_tx_queued_frames;
				}
				continue;
			}
			if (written < 0 && (errno == EAGAIN || errno == EINTR)) { break; }
			_tx_errors.fetch_add(1);
			_tx_offset = _tx_length = _tx_queued_frames = 0;
			break;
		}
		if (_tx_offset == _tx_length) { _tx_offset = _tx_length = _tx_queued_frames = 0; }
	}

	void send_periodic_status(uint64_t now)
	{
		if (now - _last_tx_us < 100000 || _tx_length != 0) { return; }
		uint8_t frame[vgsolar_link::STATUS_FRAME_LENGTH]{};
		const size_t length = build_status_frame(collect_status(), frame, sizeof(frame));
		if (queue_tx_frame(frame, length)) { _last_tx_us = now; }
	}

	void process_frame(const vgsolar_link::RxFrame &frame)
	{
		_rx_frames.fetch_add(1);
		const uint8_t expected_length[] = {0, 5, 6, 7, 2, 1, 15};
		if (frame.command == 0 || frame.command >= sizeof(expected_length) || frame.length != expected_length[frame.command]) {
			_bad_length.fetch_add(1);
			return;
		}
		// APM speed frames have no ACK. Other recognized commands receive a failure ACK;
		// no command is published to a consumer or used to change system state.
		if (frame.command == 1 || frame.command == 4) {
			_rejected_commands.fetch_add(1);
			return;
		}
		uint8_t ack[vgsolar_link::FRAME_OVERHEAD + 2]{};
		const size_t length = vgsolar_link::build_failed_ack(frame.command, ack, sizeof(ack));
		queue_tx_frame(ack, length);
		_rejected_commands.fetch_add(1);
	}

	void Run() override
	{
		if (should_exit()) {
			close_uart();
			ScheduleClear();
			exit_and_cleanup();
			return;
		}
		if (_fd < 0) { return; }
		const uint64_t now = hrt_absolute_time();
		flush_tx();
		uint8_t bytes[128];
		const ssize_t count = ::read(_fd, bytes, sizeof(bytes));
		if (count > 0) {
			for (ssize_t i = 0; i < count; ++i) {
				const vgsolar_link::RxResult result = _parser.push(bytes[i], hrt_absolute_time(), _rx_frame);
				if (result == vgsolar_link::RxResult::Valid) { process_frame(_rx_frame); }
				else if (result == vgsolar_link::RxResult::BadChecksum) { _bad_checksum.fetch_add(1); }
				else if (result == vgsolar_link::RxResult::BadLength) { _bad_length.fetch_add(1); }
				else if (result == vgsolar_link::RxResult::Timeout) { _timeouts.fetch_add(1); }
				else if (result == vgsolar_link::RxResult::RejectedSource) { _bad_length.fetch_add(1); }
			}
		} else if (count < 0 && errno != EAGAIN && errno != EINTR) {
			_rx_errors.fetch_add(1);
		}
		if (_parser.expire(now) == vgsolar_link::RxResult::Timeout) { _timeouts.fetch_add(1); }
		send_periodic_status(now);
		flush_tx();
	}

	static constexpr const char *SERIAL_PATH = "/dev/ttyS2";
	static constexpr size_t TX_FRAME_QUEUE_CAPACITY = 16;
	int _fd{-1};
	px4::atomic<bool> _active{false};
	uint64_t _last_tx_us{0};
	px4::atomic<uint32_t> _rx_frames{0}, _bad_checksum{0}, _bad_length{0}, _timeouts{0}, _rejected_commands{0};
	px4::atomic<uint32_t> _tx_frames{0}, _tx_errors{0}, _rx_errors{0};
	uint8_t _tx_buffer[128]{};
	size_t _tx_frame_ends[TX_FRAME_QUEUE_CAPACITY]{};
	size_t _tx_queued_frames{0};
	size_t _tx_offset{0};
	size_t _tx_length{0};
	vgsolar_link::Parser _parser{};
	vgsolar_link::RxFrame _rx_frame{};
	uORB::SubscriptionData<battery_status_s> _battery_sub{ORB_ID(battery_status)};
	uORB::SubscriptionData<vehicle_local_position_s> _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionData<vehicle_attitude_s> _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<sensor_gps_s> _gps_sub{ORB_ID(sensor_gps)};
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_subs{ORB_ID::distance_sensor};
};

extern "C" __EXPORT int vgsolar_link_main(int argc, char *argv[])
{
	return VgsolarLink::main(argc, argv);
}
