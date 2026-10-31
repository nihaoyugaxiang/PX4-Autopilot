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

// 两路自动输出式超声波输入驱动：只采集距离，不执行边缘急停或写执行器。
// ModuleBase提供start/status/stop；ScheduledWorkItem负责周期运行Run()。
class DypA02 : public ModuleBase<DypA02>, public px4::ScheduledWorkItem
{
public:
	// 使用PX4低优先级工作队列，串口读取必须保持非阻塞、单轮工作量有界。
	DypA02() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~DypA02() override
	{
		// 正常stop或启动中途失败都会释放已打开的串口。
		for (auto &channel : _channels) {
			if (channel.fd >= 0) { ::close(channel.fd); }
		}
	}

	static int task_spawn(int argc, char *argv[]); // 解析左右串口参数并启动调度。
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

	// 控制台status诊断：帧数表示校验通过次数，不能代替物理测距精度检查。
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
	// 每路独立持有接收、解包、样本和发布状态；通道0为左，通道1为右。
	struct Channel {
		int fd{-1}; // 串口文件句柄，-1表示尚未打开。
		char port[20] {}; // /dev/ttySx设备路径。
		uint32_t device_id{0}; // 根据串口编号和设备类型生成，区分左右设备。
		PX4Rangefinder rangefinder{0}; // 将距离发布到本路distance_sensor实例。
		dyp_a02::Parser parser; // 可以跨多次read保留半帧。
		dyp_a02::Sample sample; // 最近完整帧以及量程、新鲜度状态。
		hrt_abstime last_publish{0}; // 限制失效状态重复发布的间隔。
		// status与工作项可能分别读取和更新这些字段，因此使用原子变量。
		// latest_mm=UINT32_MAX只用于诊断，表示当前距离无效或过期。
		px4::atomic<uint32_t> frames{0}, checksum{0}, partial_timeout{0}, io_errors{0}, latest_mm{UINT32_MAX};
	};
	Channel _channels[2];
	// 配置单路串口和消息元数据；此处成功只表示端口可用，不表示传感器已收数。
	bool open_channel(unsigned index, const char *port)
	{
		// 仅接受能映射到DeviceId串口总线号的路径，并防止路径缓冲区溢出。
		if (!port || strncmp(port, "/dev/ttyS", 9) != 0 || strlen(port) >= sizeof(_channels[index].port)) { return false; }

		char *end = nullptr;
		const long bus = strtol(port + 9, &end, 10); // 跳过固定前缀/dev/ttyS。

		if (end == port + 9 || *end || bus < 0 || bus > 31) { return false; }

		auto &c = _channels[index];
		strncpy(c.port, port, sizeof(c.port) - 1);
		// 无数据时read立即返回，避免一个传感器断线拖住另一通道。
		c.fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (c.fd < 0) { PX4_ERR("open %s failed: %d", port, errno); return false; }

		termios config{};

		if (tcgetattr(c.fd, &config) != 0) { return false; }

		// 原始二进制模式：关闭字符转换、回显、规范行输入和软/硬件流控。
		config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
		config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		config.c_oflag &= ~OPOST;
		config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
		config.c_cflag |= CS8 | CLOCAL | CREAD; // 8位数据、无校验、1个停止位。
		config.c_cc[VMIN] = 0;
		config.c_cc[VTIME] = 0;

		// 自动输出协议使用9600波特；应用设置后丢弃启动前积累的接收字节。
		if (cfsetispeed(&config, B9600) != 0 || cfsetospeed(&config, B9600) != 0
		    || tcsetattr(c.fd, TCSANOW, &config) != 0 || tcflush(c.fd, TCIFLUSH) != 0) { return false; }

		// DeviceId从零初始化，避免未使用的地址位带入随机值。
		device::Device::DeviceId id{};
		id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
		id.devid_s.bus = bus;
		id.devid_s.devtype = DRV_DIST_DEVTYPE_DYP_A02;
		c.device_id = id.devid;
		c.rangefinder.set_device_id(id.devid);
		// 方向标签和量程沿用APM板级配置，实际安装和精度仍需实板核对。
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
			// 先撤销两路有效读数，再停止调度并让ModuleBase清理对象。
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
			// 每路每轮最多128字节，噪声流也不能无限占用当前工作队列。
			const ssize_t count = ::read(c.fd, buffer, sizeof(buffer));
			const hrt_abstime now = hrt_absolute_time(); // 本批字节的主机接收处理时间。
			bool frame_received = false;

			// 非阻塞端口暂时无数据或被信号中断属于正常情况，其他读错误计数。
			if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
				c.io_errors.fetch_add(1);
			}

			// 解包可跨调度周期；同一批中有多个完整帧时，最终保留最后一帧。
			for (ssize_t j = 0; j < count; ++j) {
				uint16_t mm = 0;

				if (c.parser.feed(buffer[j], now, mm)) {
					// 仅校验正确帧刷新时间；越量程完整帧也覆盖旧样本，由valid撤销有效性。
					c.sample.receive(mm, now);
					c.frames.fetch_add(1);
					frame_received = true;
				}
			}

			// 将工作项内部计数快照提供给status；即使没有新帧也检查是否过期。
			c.checksum.store(c.parser.checksum_errors);
			c.partial_timeout.store(c.parser.partial_timeouts);
			const bool valid = c.sample.valid(now);
			c.latest_mm.store(valid ? c.sample.mm() : UINT32_MAX);

			// 新帧立即发布；无效/断流期间约100ms重发一次失效状态。
			// quality=-1表示未知质量，0表示无效；不能把距离0当成真实贴近读数。
			if (frame_received || (!valid && now - c.last_publish >= 100000)) {
				c.rangefinder.update(now, c.sample.distance_m(now), valid ? -1 : 0);
				c.last_publish = now;
			}
		}
	}
};

int DypA02::task_spawn(int argc, char *argv[])
{
	// -l指定左串口，-r指定右串口；不隐式选择默认口，避免误占其他外设端口。
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

	// 两路都必须配置，且不能让同一个串口被左右两路重复读取。
	if (!left || !right || !strcmp(left, right)) { return print_usage("specify two different UARTs"), PX4_ERROR; }

	auto *instance = new DypA02();

	if (!instance) { return PX4_ERROR; }

	// 任一路初始化失败则整体启动失败；析构函数负责回收已成功打开的另一路。
	if (!instance->open_channel(0, left) || !instance->open_channel(1, right)) {
		delete instance;
		return PX4_ERROR;
	}

	// 两口都准备好后再注册模块对象，并请求每10ms检查一次串口。
	// 此周期是软件检查周期，不是传感器测距频率或硬实时保证。
	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(10000);
	return PX4_OK;
}

extern "C" __EXPORT int dyp_a02_main(int argc, char *argv[])
{
	// NSH命令入口，具体start/status/stop分发由ModuleBase实现。
	return DypA02::main(argc, argv);
}
