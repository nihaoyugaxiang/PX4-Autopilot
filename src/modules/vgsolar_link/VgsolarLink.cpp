// SPDX-License-Identifier: BSD-3-Clause
// USART3 NCU link: validated commands and periodic VGSolar feedback.
#include "Protocol.hpp"
#include "StatusProcessing.hpp"
#include <lib/rover_control/VgsolarBattery.hpp>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/vgsolar_encoder_status.h>

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
#include <uORB/topics/wheel_encoders.h>
#include <px4_platform_common/param.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vgsolar_command.h>
#include <uORB/topics/vgsolar_link_status.h>
#include <uORB/topics/vgsolar_link_ack.h>
#include <uORB/topics/vgsolar_mode_status.h>
#include <uORB/topics/vgsolar_param_feedback.h>
#include <uORB/topics/vgsolar_safety_status.h>
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
		PX4_INFO("NCU bridge: commands routed to VGSolar mode; check mode and hardware enables");
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
		PX4_INFO("NCU command/status link opened %s at 115200 8N1", SERIAL_PATH);
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
		SensorDiagnostics diagnostics{};
		fields.left_out_cm = fields.left_in_cm = fields.right_in_cm = fields.right_out_cm = RANGE_INVALID_CM;
		const uint64_t now = hrt_absolute_time();

		battery_status_s battery{};
		if (_battery_sub.copy(&battery) && battery.timestamp != 0 && now >= battery.timestamp
		    && now - battery.timestamp < 1000000) {
			const float remaining = vgsolar::battery_remaining(battery.voltage_v, battery.remaining, battery.connected);
			if (PX4_ISFINITE(remaining)) { fields.battery_percent = static_cast<uint8_t>(lroundf(remaining * 100.f)); }
			if (PX4_ISFINITE(remaining) && fields.battery_percent <= 20) { fields.fault_code |= 1u << 8; }
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
			diagnostics.roll_deg = math::degrees(euler.phi());
			diagnostics.pitch_deg = math::degrees(euler.theta());
			const float norm = matrix::Quatf{attitude.q}.norm();
			diagnostics.attitude_valid = PX4_ISFINITE(norm) && fabsf(norm - 1.f) < .01f
						     && PX4_ISFINITE(diagnostics.roll_deg) && PX4_ISFINITE(diagnostics.pitch_deg);
			fields.roll = scale_i16(math::degrees(euler.phi()), 100.0);
			fields.pitch = scale_i16(math::degrees(euler.theta()), 100.0);
			if (!local_heading_valid && PX4_ISFINITE(euler.psi())) {
				float heading = math::degrees(euler.psi());
				if (heading < 0.f) { heading += 360.f; }
				fields.heading = static_cast<uint16_t>(lroundf(heading * 100.f)) % 36000;
			}
		}

		failsafe_flags_s estimator{};
		diagnostics.estimator_healthy = _failsafe_sub.copy(&estimator) && estimator.timestamp > 0 && now >= estimator.timestamp
						 && now - estimator.timestamp < 2000000 && !estimator.attitude_invalid && !estimator.angular_velocity_invalid;
		sensor_gps_s gps{};
		if (_gps_sub.copy(&gps) && gps.timestamp != 0 && now >= gps.timestamp && now - gps.timestamp < 1000000) {
			if (gps.fix_type >= sensor_gps_s::FIX_TYPE_2D && gps.fix_type <= sensor_gps_s::FIX_TYPE_RTK_FIXED) {
				diagnostics.gps_3d = gps.fix_type >= sensor_gps_s::FIX_TYPE_3D;
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
		float radius = 0.f;
		const param_t radius_handle = param_find("VGS_WHL_RAD");
		if (radius_handle != PARAM_INVALID) { param_get(radius_handle, &radius); }
		wheel_encoders_s wheel{};
		vgsolar_encoder_status_s capture{};
		const bool capture_valid = _wheel_capture_sub.copy(&capture) && capture.capture_active && capture.timestamp > 0
					   && capture.timestamp <= now && now - capture.timestamp < 500000;
		const bool wheel_fresh = _wheel_sub.copy(&wheel) && wheel.timestamp > 0 && wheel.timestamp <= now
					 && now - wheel.timestamp < 500000;
		int32_t left_cpr = 0, right_cpr = 0;
		const param_t left_cpr_handle = param_find("VGS_ENC_L_CPR"), right_cpr_handle = param_find("VGS_ENC_R_CPR");
		if (left_cpr_handle != PARAM_INVALID) { param_get(left_cpr_handle, &left_cpr); }
		if (right_cpr_handle != PARAM_INVALID) { param_get(right_cpr_handle, &right_cpr); }
		diagnostics.left_enabled = left_cpr > 0;
		diagnostics.right_enabled = right_cpr > 0;
		diagnostics.right_healthy = capture_valid && wheel_fresh && PX4_ISFINITE(wheel.wheel_speed[0]);
		diagnostics.left_healthy = capture_valid && wheel_fresh && PX4_ISFINITE(wheel.wheel_speed[1]);
		if (PX4_ISFINITE(radius) && radius > 0.f && radius <= 1.f) {
			if (diagnostics.right_healthy) { fields.right_track_velocity = scale_i16(wheel.wheel_speed[0] * radius, 100.0); }
			if (diagnostics.left_healthy) { fields.left_track_velocity = scale_i16(wheel.wheel_speed[1] * radius, 100.0); }
		}
		fields.fault_code |= sensor_faults(diagnostics);
		bool estop = false, turning = false;
		vgsolar_mode_status_s mode{};
		if (_mode_status_sub.copy(&mode) && mode.timestamp > 0 && now >= mode.timestamp && now - mode.timestamp < 500000) {
			// Keep measured ground velocity; requested speed is logged in mode status.
			if (mode.active) {
				switch (mode.state) {
				case vgsolar_mode_status_s::STATE_YAW: fields.control_mode = 1; break;
				case vgsolar_mode_status_s::STATE_YAWRATE: fields.control_mode = 2; break;
				case vgsolar_mode_status_s::STATE_TURN: fields.control_mode = 3; break;
				case vgsolar_mode_status_s::STATE_NAVIGATION: fields.control_mode = mode.nav_mode == 3 ? 5 : 4; break;
				default: fields.control_mode = 0; break;
				}
			}
			fields.fault_code |= mode.fault_code;
			estop = mode.active && mode.state == vgsolar_mode_status_s::STATE_ESTOP;
			turning = mode.active && mode.turning_motion;
		}
		vgsolar_safety_status_s safety{};
		if (_safety_status_sub.copy(&safety) && safety.timestamp > 0 && now >= safety.timestamp
		    && now - safety.timestamp < 500000 && safety.stop_request) {
			fields.fault_code |= 1u << 11;
		}
		fields.motion_state = motion_state(fields.velocity, estop, turning, fields.fault_code);
		return fields;
	}

	bool queue_tx_frame(const uint8_t *frame, size_t length, bool periodic = false)
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
			if (periodic) { ++_drop_periodic; } else { ++_drop_event; }
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
		if (queue_tx_frame(frame, length, true)) { _last_tx_us = now; }
		vgsolar_mode_status_s mode{};
		if (_mode_status_sub.copy(&mode) && mode.timestamp > 0 && now >= mode.timestamp
		    && now - mode.timestamp < 500000 && mode.navigation_state != 0) {
			const bool terminal = mode.navigation_state != vgsolar_mode_status_s::NAV_ACTIVE;
			if (_navigation_reports.needed(mode.navigation_state, mode.nav_mode, mode.navigation_event_timestamp)) {
				uint8_t nav[15]{};
				const size_t count = vgsolar_link::build_navigation_frame(mode.navigation_state, mode.nav_mode,
						   terminal ? 0 : mode.nav_distance_cm, terminal ? 0 : mode.nav_heading_error_cd, nav, sizeof(nav));
				if (queue_tx_frame(nav, count, true)) {
					_navigation_reports.queued(mode.navigation_state, mode.nav_mode, mode.navigation_event_timestamp);
				}
			}
		}
	}

	static uint16_t get_u16(const uint8_t *data)
	{
		return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
	}

	static int16_t get_i16(const uint8_t *data) { return static_cast<int16_t>(get_u16(data)); }
	static uint32_t get_u32(const uint8_t *data)
	{
		return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8)
		       | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
	}
	static int32_t get_i32(const uint8_t *data) { return static_cast<int32_t>(get_u32(data)); }

	void send_ack(uint8_t command, bool success)
	{
		uint8_t frame[vgsolar_link::FRAME_OVERHEAD + 2]{};
		const size_t length = vgsolar_link::build_ack(command, success, frame, sizeof(frame));
		queue_tx_frame(frame, length);
	}

	void send_param_feedback(const vgsolar_param_feedback_s &feedback)
	{
		uint8_t frame[vgsolar_link::FRAME_OVERHEAD + 7]{};
		const size_t length = vgsolar_link::build_param_feedback(feedback.param_index, feedback.param_type, feedback.param_value, frame, sizeof(frame));
		queue_tx_frame(frame, length);
	}

	void publish_command(const vgsolar_link::RxFrame &frame)
	{
		using namespace vgsolar_link;
		vgsolar_command_s command{};
		command.timestamp = hrt_absolute_time();
		command.command = frame.command;
		const uint8_t *body = frame.body;

		switch (frame.command) {
		case vgsolar_command_s::CMD_SPEED:
			command.control_mode = body[0];
			command.velocity_cms = get_i16(body + 1);
			command.yaw_data = get_i16(body + 3);
			break;
		case vgsolar_command_s::CMD_TURN:
			command.turn_mode = body[0];
			command.turn_direction = body[1];
			command.turn_angle_cd = get_u16(body + 2);
			command.turn_rate_cds = get_u16(body + 4);
			break;
		case vgsolar_command_s::CMD_PARAM_WRITE:
			command.param_index = get_u16(body);
			command.param_type = body[2];
			command.param_value = get_u32(body + 3);
			break;
		case vgsolar_command_s::CMD_PARAM_READ:
			command.param_index = get_u16(body);
			break;
		case vgsolar_command_s::CMD_SYSTEM:
			command.system_command = body[0];
			break;
		case vgsolar_command_s::CMD_POSITION:
			command.nav_mode = body[0];
			command.target_x = get_i32(body + 1);
			command.target_y = get_i32(body + 5);
			command.arrival_yaw_cd = get_u16(body + 9);
			command.arrival_radius_cm = get_u16(body + 11);
			command.cruise_speed_cms = get_i16(body + 13);
			break;
		default:
			return;
		}

		_command_pub.publish(command);
		// Protocol ACK for TURN/SYSTEM means received, not that the operation completed.
		if (frame.command == vgsolar_command_s::CMD_TURN || frame.command == vgsolar_command_s::CMD_SYSTEM) {
			send_ack(frame.command, true);
		}
	}

	void process_frame(const vgsolar_link::RxFrame &frame)
	{
		_rx_frames.fetch_add(1);
		const uint8_t expected_length[] = {0, 5, 6, 7, 2, 1, 15};
		if (frame.command == 0 || frame.command >= sizeof(expected_length) || frame.length != expected_length[frame.command]) {
			_bad_length.fetch_add(1);
			return;
		}
		_last_valid_rx = hrt_absolute_time();
		publish_command(frame);
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
		vgsolar_link_ack_s ack{};
		while (_ack_sub.update(&ack)) { send_ack(ack.command, ack.success); }
		vgsolar_param_feedback_s feedback{};
		while (_param_sub.update(&feedback)) { send_param_feedback(feedback); }
		send_periodic_status(now);
  if (now - _last_diagnostic >= 1000000) {
   _last_diagnostic = now; int32_t level = 1; const param_t p = param_find("VGS_CC_LOG");
   if (p != PARAM_INVALID) { param_get(p, &level); }
   if (level > 0) {
    vgsolar_link_status_s d{}; d.timestamp = now; d.last_valid_rx = _last_valid_rx;
    d.rx_frames = _rx_frames.load(); d.bad_checksum = _bad_checksum.load(); d.bad_length = _bad_length.load();
    d.timeouts = _timeouts.load(); d.tx_frames = _tx_frames.load(); d.tx_errors = _tx_errors.load(); d.rx_errors = _rx_errors.load();
    d.dropped_event = _drop_event; d.dropped_periodic = _drop_periodic; _diagnostic_pub.publish(d);
   }
  }
		flush_tx();
	}

	static constexpr const char *SERIAL_PATH = "/dev/ttyS2";
	static constexpr size_t TX_FRAME_QUEUE_CAPACITY = 16;
	uORB::Publication<vgsolar_link_status_s> _diagnostic_pub{ORB_ID(vgsolar_link_status)};
 uint64_t _last_valid_rx{0}, _last_diagnostic{0};
 uint32_t _drop_event{0}, _drop_periodic{0};
	int _fd{-1};
	px4::atomic<bool> _active{false};
	uint64_t _last_tx_us{0};
	vgsolar_link::NavigationReportTracker _navigation_reports{};
	px4::atomic<uint32_t> _rx_frames{0}, _bad_checksum{0}, _bad_length{0}, _timeouts{0}, _rejected_commands{0};
	px4::atomic<uint32_t> _tx_frames{0}, _tx_errors{0}, _rx_errors{0};
	uint8_t _tx_buffer[128]{};
	size_t _tx_frame_ends[TX_FRAME_QUEUE_CAPACITY]{};
	size_t _tx_queued_frames{0};
	size_t _tx_offset{0};
	size_t _tx_length{0};
	vgsolar_link::Parser _parser{};
	vgsolar_link::RxFrame _rx_frame{};
	uORB::SubscriptionData<failsafe_flags_s> _failsafe_sub{ORB_ID(failsafe_flags)};
	uORB::SubscriptionData<vgsolar_encoder_status_s> _wheel_capture_sub{ORB_ID(vgsolar_encoder_status)};
	uORB::SubscriptionData<wheel_encoders_s> _wheel_sub{ORB_ID(wheel_encoders)};
	uORB::SubscriptionData<battery_status_s> _battery_sub{ORB_ID(battery_status)};
	uORB::SubscriptionData<vehicle_local_position_s> _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionData<vehicle_attitude_s> _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<sensor_gps_s> _gps_sub{ORB_ID(sensor_gps)};
	uORB::SubscriptionMultiArray<distance_sensor_s> _distance_subs{ORB_ID::distance_sensor};
	uORB::Publication<vgsolar_command_s> _command_pub{ORB_ID(vgsolar_command)};
	uORB::Subscription _ack_sub{ORB_ID(vgsolar_link_ack)};
	uORB::Subscription _param_sub{ORB_ID(vgsolar_param_feedback)};
	uORB::SubscriptionData<vgsolar_mode_status_s> _mode_status_sub{ORB_ID(vgsolar_mode_status)};
	uORB::SubscriptionData<vgsolar_safety_status_s> _safety_status_sub{ORB_ID(vgsolar_safety_status)};
};

extern "C" __EXPORT int vgsolar_link_main(int argc, char *argv[])
{
	return VgsolarLink::main(argc, argv);
}
