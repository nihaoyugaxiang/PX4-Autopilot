// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "Protocol.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/drivers/device/Device.hpp>
#include <drivers/drv_sensor.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

class DypA02 : public ModuleBase<DypA02>, public px4::ScheduledWorkItem
{
public:
	DypA02() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~DypA02() override
	{
		for (auto &channel : _channels) {
			if (channel.fd >= 0) { ::close(channel.fd); }
		}
	}

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_USAGE_NAME("dyp_a02", "driver");
		PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_PARAM_STRING('l', nullptr, "/dev/ttySx", "Left UART (orientation 7)", false);
		PRINT_MODULE_USAGE_PARAM_STRING('r', nullptr, "/dev/ttySx", "Right UART (orientation 1)", false);
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	int print_status() override
	{
		for (unsigned i = 0; i < 2; ++i) {
			const auto &c = _channels[i];
			PX4_INFO("%s %s id=%" PRIu32 " orientation=%u latest_mm=%" PRIu32,
				 i == 0 ? "left" : "right", c.port, c.device_id, i == 0 ? 7u : 1u, c.latest_mm.load());
			PX4_INFO("frames=%" PRIu32 " checksum=%" PRIu32 " partial_timeout=%" PRIu32 " io_errors=%" PRIu32,
				 c.frames.load(), c.checksum.load(), c.partial_timeout.load(), c.io_errors.load());
		}

		PX4_INFO("9600 8N1; latest_mm=4294967295 means invalid/stale; timeout=500 ms");
		return 0;
	}

private:
	struct Channel {
		int fd{-1};
		char port[20] {};
		uint32_t device_id{0};
		PX4Rangefinder rangefinder{0};
		dyp_a02::Parser parser;
		dyp_a02::Sample sample;
		hrt_abstime last_publish{0};
		px4::atomic<uint32_t> frames{0}, checksum{0}, partial_timeout{0}, io_errors{0}, latest_mm{UINT32_MAX};
	};
	Channel _channels[2];
	bool open_channel(unsigned index, const char *port)
	{
		if (!port || strncmp(port, "/dev/ttyS", 9) != 0 || strlen(port) >= sizeof(_channels[index].port)) { return false; }

		char *end = nullptr;
		const long bus = strtol(port + 9, &end, 10);

		if (end == port + 9 || *end || bus < 0 || bus > 31) { return false; }

		auto &c = _channels[index];
		strncpy(c.port, port, sizeof(c.port) - 1);
		c.fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (c.fd < 0) { PX4_ERR("open %s failed: %d", port, errno); return false; }

		termios config{};

		if (tcgetattr(c.fd, &config) != 0) { return false; }

		config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
		config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		config.c_oflag &= ~OPOST;
		config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
		config.c_cflag |= CS8 | CLOCAL | CREAD;
		config.c_cc[VMIN] = 0;
		config.c_cc[VTIME] = 0;

		if (cfsetispeed(&config, B9600) != 0 || cfsetospeed(&config, B9600) != 0
		    || tcsetattr(c.fd, TCSANOW, &config) != 0 || tcflush(c.fd, TCIFLUSH) != 0) { return false; }

		device::Device::DeviceId id{};
		id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
		id.devid_s.bus = bus;
		id.devid_s.devtype = DRV_DIST_DEVTYPE_DYP_A02;
		c.device_id = id.devid;
		c.rangefinder.set_device_id(id.devid);
		c.rangefinder.set_orientation(index == 0 ? 7 : 1);
		c.rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND);
		c.rangefinder.set_min_distance(0.03f);
		c.rangefinder.set_max_distance(4.5f);
		c.rangefinder.set_mode(distance_sensor_s::MODE_ENABLED);
		return true;
	}

	void Run() override
	{
		if (should_exit()) {
			for (auto &c : _channels) {
				c.rangefinder.set_mode(distance_sensor_s::MODE_DISABLED);
				c.rangefinder.update(hrt_absolute_time(), 0.f, 0);
			}

			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		for (auto &c : _channels) {
			uint8_t buffer[128];
			// Bounded nonblocking read: either channel can be silent or noisy.
			const ssize_t count = ::read(c.fd, buffer, sizeof(buffer));
			const hrt_abstime now = hrt_absolute_time();
			bool frame_received = false;

			if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
				c.io_errors.fetch_add(1);
			}

			for (ssize_t j = 0; j < count; ++j) {
				uint16_t mm = 0;

				if (c.parser.feed(buffer[j], now, mm)) {
					c.sample.receive(mm, now);
					c.frames.fetch_add(1);
					frame_received = true;
				}
			}

			c.checksum.store(c.parser.checksum_errors);
			c.partial_timeout.store(c.parser.partial_timeouts);
			const bool valid = c.sample.valid(now);
			c.latest_mm.store(valid ? c.sample.mm() : UINT32_MAX);

			if (frame_received || (!valid && now - c.last_publish >= 100000)) {
				c.rangefinder.update(now, c.sample.distance_m(now), valid ? -1 : 0);
				c.last_publish = now;
			}
		}
	}
};

int DypA02::task_spawn(int argc, char *argv[])
{
	const char *left = nullptr;
	const char *right = nullptr;
	int opt;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((opt = px4_getopt(argc, argv, "l:r:", &myoptind, &myoptarg)) != EOF) {
		if (opt == 'l') { left = myoptarg; }
		else if (opt == 'r') { right = myoptarg; }
		else { return print_usage("invalid option"), PX4_ERROR; }
	}

	if (!left || !right || !strcmp(left, right)) { return print_usage("specify two different UARTs"), PX4_ERROR; }

	auto *instance = new DypA02();

	if (!instance) { return PX4_ERROR; }

	if (!instance->open_channel(0, left) || !instance->open_channel(1, right)) {
		delete instance;
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(10000);
	return PX4_OK;
}

extern "C" __EXPORT int dyp_a02_main(int argc, char *argv[])
{
	return DypA02::main(argc, argv);
}
