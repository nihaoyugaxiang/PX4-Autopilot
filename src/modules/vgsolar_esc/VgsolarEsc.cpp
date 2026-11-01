// SPDX-License-Identifier: BSD-3-Clause
// Read-only 2BLD6010 Modbus RTU telemetry on UART4; never writes device registers.
#include "Protocol.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/param.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vgsolar_esc_status.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

class VgsolarEsc : public ModuleBase<VgsolarEsc>, public px4::ScheduledWorkItem
{
public:
	VgsolarEsc() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~VgsolarEsc() override { if (_fd >= 0) { ::close(_fd); } }
	static int task_spawn(int, char *[]) {
		auto *self = new VgsolarEsc();
		if (!self) { return PX4_ERROR; }
		if (!self->init()) { delete self; return PX4_ERROR; }
		_object.store(self); _task_id = task_id_is_work_queue;
		self->ScheduleOnInterval(10000); return PX4_OK;
	}
	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr) {
		if (reason) { PX4_WARN("%s", reason); }
		PRINT_MODULE_USAGE_NAME("vgsolar_esc", "driver");
		PRINT_MODULE_USAGE_COMMAND("start"); PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); return 0;
	}
	int print_status() override {
		PX4_INFO("UART4 /dev/ttyS3 read-only count=%u online=0x%02x timeouts=%u errors=%u",
			 _count, _online, (unsigned)_timeouts, (unsigned)_errors); return 0;
	}
private:
	static int32_t get(const char *name, int32_t fallback) {
		int32_t value = fallback; const param_t handle = param_find(name);
		if (handle != PARAM_INVALID) { param_get(handle, &value); } return value;
	}
	bool init() {
		const int32_t count = get("VGS_ESC_NUM", 1), rate = get("VGS_ESC_RATE", 10), timeout = get("VGS_ESC_TO", 500);
		if (get("VGS_ESC_EN", 0) != 1 || count < 1 || count > 4 || rate < 1 || rate > 20 || timeout < 100 || timeout > 5000) {
			PX4_ERR("disabled or invalid configuration"); return false;
		}
		const int32_t offset = get("VGS_ESC_OFS", 0);
  if (offset < 0 || offset + count > 8) { return false; }
  _offset = offset;
		_count = count; _period_us = 1000000u / (rate * count); _health_us = timeout * 1000u;
		const char *names[4] = {"VGS_ESC_ADDR1", "VGS_ESC_ADDR2", "VGS_ESC_ADDR3", "VGS_ESC_ADDR4"};
		const int32_t defaults[4] = {2, 1, 3, 4};
		for (unsigned i = 0; i < _count; ++i) {
			const int32_t address = get(names[i], defaults[i]);
			if (address < 1 || address > 247) { return false; }
			_address[i] = address;
			for (unsigned j = 0; j < i; ++j) { if (_address[i] == _address[j]) { PX4_ERR("duplicate address"); return false; } }
		}
		_fd = ::open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (_fd < 0) { PX4_ERR("UART4 unavailable"); return false; }
		termios config{};
		if (tcgetattr(_fd, &config) != 0) { return false; }
		cfmakeraw(&config); cfsetispeed(&config, B115200); cfsetospeed(&config, B115200);
		config.c_cflag = (config.c_cflag & ~(PARENB | CSTOPB | CSIZE)) | CS8 | CLOCAL | CREAD;
#ifdef CRTSCTS
		config.c_cflag &= ~CRTSCTS;
#endif
		config.c_cc[VMIN] = 0; config.c_cc[VTIME] = 0;
		if (tcsetattr(_fd, TCSANOW, &config) != 0) { return false; }
		tcflush(_fd, TCIFLUSH); return true;
	}
	void finish(uint64_t now) {
		_waiting = false; _sending = false; _tx_offset = 0;
		_index = (_index + 1) % _count; _next = _sent_at + _period_us; (void)now; _parser.reset();
	}
	void publish(uint64_t now) {
		esc_status_s status{};
		vgsolar_esc_status_s raw{};
		raw.timestamp = status.timestamp = now; raw.count = _count; raw.offset = _offset; status.esc_count = _offset + _count;
		status.counter = ++_counter; status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;
		raw.timeouts = _timeouts; raw.errors = _errors;
		_online = 0;
		for (unsigned i = 0; i < _count; ++i) {
			const auto &sample = _samples[i];
			const bool healthy = _updated[i] != 0 && now >= _updated[i] && now - _updated[i] <= _health_us;
			if (healthy) { _online |= 1u << i; }
			auto &esc = status.esc[_offset + i];
			esc.timestamp = _updated[i]; esc.esc_address = _address[i];
			esc.esc_rpm = sample.direction == 2 ? -static_cast<int32_t>(sample.rpm) : sample.rpm;
			esc.esc_voltage = sample.voltage_v; esc.esc_current = sample.current_a; esc.esc_temperature = sample.temperature_c;
			esc.esc_state = sample.direction; esc.esc_power = -1;
			// Keep vendor faults raw; they are not PX4's standard failures bitmap.
			raw.sample_timestamp[i] = _updated[i]; raw.address[i] = _address[i]; raw.fault_code[i] = sample.fault;
			raw.direction[i] = sample.direction; raw.hall_count[i] = sample.hall;
   raw.success_count[i] = _success[i]; raw.crc_errors[i] = _crc[i]; raw.timeout_count[i] = _timeout[i];
   raw.bad_responses[i] = _bad[i]; raw.exceptions[i] = _exceptions[i];
		}
		raw.online_mask = status.esc_online_flags = _online << _offset;
		status.esc_armed_flags = 0; _standard_pub.publish(status); _raw_pub.publish(raw);
	}
	void Run() override {
		const uint64_t now = hrt_absolute_time();
		if (should_exit()) { _online = 0; for (auto &stamp : _updated) { stamp = 0; } publish(now);
			ScheduleClear(); exit_and_cleanup(); return; }
		uint8_t bytes[64]{};
		// Bounded RX work; late or unsolicited bytes never update another controller.
		for (unsigned batch = 0; batch < 4; ++batch) {
			const ssize_t length = ::read(_fd, bytes, sizeof(bytes));
			if (length <= 0) { if (length < 0 && errno != EAGAIN && errno != EINTR) { ++_errors; } break; }
			for (ssize_t i = 0; i < length; ++i) {
				if (!_waiting) { continue; }
				vgsolar_esc::Sample sample{};
				const auto old_crc = _parser.crc_errors, old_bad = _parser.bad_responses, old_exc = _parser.exceptions;
     const auto result = _parser.push(bytes[i], _address[_index], sample);
     _crc[_index] += _parser.crc_errors - old_crc; _bad[_index] += _parser.bad_responses - old_bad;
     _exceptions[_index] += _parser.exceptions - old_exc;
				if (result == vgsolar_esc::Result::Valid) {
					++_success[_index]; _samples[_index] = sample; _updated[_index] = now; finish(now);
				} else if (result == vgsolar_esc::Result::Exception) { ++_errors; finish(now); }
			}
		}
		if ((_waiting || _sending) && now - _sent_at >= 40000) { ++_timeout[_index]; ++_timeouts; finish(now); }
		if (!_waiting && !_sending && now >= _next) {
			vgsolar_esc::request(_address[_index], _tx); _tx_offset = 0; _sending = true; _sent_at = now; _parser.reset();
		}
		if (_sending) {
			const ssize_t written = ::write(_fd, _tx + _tx_offset, sizeof(_tx) - _tx_offset);
			if (written > 0) {
				_tx_offset += written;
				if (_tx_offset == sizeof(_tx)) { _sending = false; _waiting = true; _sent_at = now; }
			} else if (written < 0 && errno != EAGAIN && errno != EINTR) { ++_errors; finish(now); }
		}
		if (now - _last_publish >= 100000) { publish(now); _last_publish = now; }
	}
	int _fd{-1};
	uint32_t _success[4]{}, _crc[4]{}, _timeout[4]{}, _bad[4]{}, _exceptions[4]{};
 uint8_t _offset{0};
	uint8_t _count{0}, _index{0}, _online{0}, _address[4]{}, _tx[8]{};
	uint16_t _counter{0};
	size_t _tx_offset{0};
	bool _waiting{false}, _sending{false};
	uint64_t _updated[4]{}, _sent_at{0}, _next{0}, _last_publish{0};
	uint32_t _period_us{100000}, _health_us{500000}, _timeouts{0}, _errors{0};
	vgsolar_esc::Sample _samples[4]{};
	vgsolar_esc::Parser _parser{};
	uORB::Publication<esc_status_s> _standard_pub{ORB_ID(esc_status)};
	uORB::Publication<vgsolar_esc_status_s> _raw_pub{ORB_ID(vgsolar_esc_status)};
};
extern "C" __EXPORT int vgsolar_esc_main(int argc, char *argv[]) { return VgsolarEsc::main(argc, argv); }
