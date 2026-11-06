// SPDX-License-Identifier: BSD-3-Clause
#include "host/px4_stubs.hpp"
#include <cassert>
#include "../VgsolarMode.cpp"

struct TestMode : VgsolarMode {
	static void running(bool value) { _task_id = value ? -2 : -1; }
};
static int change = 0;
static uint64_t started = 0;
static void health()
{
	vehicle_status_s vehicle{};
	vehicle.timestamp = host::now;
	vehicle.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	actuator_armed_s armed{};
	armed.timestamp = host::now;
	armed.prearmed = true; // CH8 servo passthrough may remain prearmed.
	if (host::now - started >= 30000) {
		if (change == 1) { vehicle.arming_state = vehicle_status_s::ARMING_STATE_ARMED; }
		if (change == 2) { armed.armed = true; }
		if (change == 3) { armed.kill = true; }
		if (change == 4) { host::parameter<int32_t>("VGS_ENABLE", 1); }
	}
	host::publish("vehicle_status", vehicle);
	host::publish("actuator_armed", armed);
}
static void setup()
{
	host::topics.clear(); host::gpio_writes.clear(); host::parameters.clear(); host::param_names.clear();
	change = 0; started = host::now;
	TestMode::running(false);
	host::parameter<int32_t>("VGS_ENABLE", 0);
	host::parameter<int32_t>("VGS_CLN_EN", 0);
	host::sleep_hook = health;
	health();
}
static int command(bool pump)
{
	char valve_name[] = "test-valve", pump_name[] = "test-pump";
	char *args[] = {pump ? pump_name : valve_name};
	pthread_mutex_lock(&px4_modules_mutex);
	const int result = VgsolarMode::custom_command(1, args);
	pthread_mutex_unlock(&px4_modules_mutex);
	return result;
}
static void off()
{
	assert(!host::gpio[GPIO_VG_PUMP_OFF] && host::gpio[GPIO_VG_VALVE_OFF]);
}
int main()
{
	for (bool pump : {false, true}) {
		setup();
		assert(command(pump) == 0);
		assert(host::now - started == 1000000);
		bool active = false;
		for (const auto &write : host::gpio_writes) {
			if (write.pin == GPIO_VG_PUMP_OFF && write.value) { assert(pump); active = true; }
			if (write.pin == GPIO_VG_VALVE_OFF && !write.value) { assert(!pump); active = true; }
		}
		assert(active); off();
	}
	setup(); TestMode::running(true);
	assert(command(true) != 0 && host::gpio_writes.empty());
	setup(); host::topics.clear();
	assert(command(true) != 0 && host::gpio_writes.empty());
	setup(); host::parameter<int32_t>("VGS_CLN_EN", 1);
	assert(command(false) != 0 && host::gpio_writes.empty());
	setup(); host::parameters.erase("VGS_ENABLE");
	assert(command(false) != 0 && host::gpio_writes.empty());
	setup(); host::now += 1000001;
	assert(command(false) != 0 && host::gpio_writes.empty());
	for (int scenario = 1; scenario <= 4; ++scenario) {
		setup(); change = scenario;
		assert(command(true) != 0);
		assert(host::now - started == 30000); off();
	}
	setup(); host::now += 500000; started = host::now; host::sleep_hook = nullptr;
	assert(command(false) != 0 && host::now - started == 510000); off();
	puts("PASS: production GPIO bench pulse duration/polarity/exclusion, running-mode/missing/disabled/stale guards, arm/kill/config-change abort and cleanup");
}
