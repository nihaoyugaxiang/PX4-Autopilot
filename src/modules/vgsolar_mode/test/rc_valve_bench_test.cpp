// SPDX-License-Identifier: BSD-3-Clause
#include "host/px4_stubs.hpp"
#include <cassert>
#include "../VgsolarMode.cpp"

struct TestMode : VgsolarMode {
	static void running(bool value) { _task_id = value ? -2 : -1; }
};
static uint64_t started;
static int scenario;
static void refresh()
{
	const auto elapsed = host::now - started;
	if (elapsed == 10000) {
		// Starting the automatic mode or a second test must not steal the pins
		// while the long-running command has released the global module mutex.
		assert(pthread_mutex_trylock(&px4_modules_mutex) == 0);
		assert(VgsolarMode::task_spawn(0, nullptr) != 0);
		char name[] = "test-pump"; char *args[] = {name};
		assert(VgsolarMode::custom_command(1, args) != 0);
		pthread_mutex_unlock(&px4_modules_mutex);
	}
	vehicle_status_s vehicle{};
	vehicle.timestamp = host::now;
	vehicle.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	actuator_armed_s armed{}; armed.timestamp = host::now; armed.prearmed = true;
	if (scenario == 8 && elapsed >= 600000) { armed.armed = true; }
	host::publish("vehicle_status", vehicle); host::publish("actuator_armed", armed);
	input_rc_s rc{};
	rc.timestamp = rc.timestamp_last_signal = host::now;
	rc.channel_count = 8;
	rc.values[7] = elapsed < 100000 || (elapsed >= 400000 && elapsed < 500000) || elapsed >= 600000 ? 1000 : 2000;
	// At 200ms revoke reception; recovered HIGH must not reactivate until LOW.
	if (elapsed >= 200000 && elapsed < 300000) {
		if (scenario == 1) { rc.rc_lost = true; }
		if (scenario == 2) { rc.rc_failsafe = true; }
		if (scenario == 3) { rc.timestamp_last_signal -= 400000; }
		if (scenario == 4) { rc.timestamp -= 400000; }
		if (scenario == 5) { rc.channel_count = 7; }
		if (scenario == 6) { rc.values[7] = 2500; }
	}
	if (scenario == 7 && elapsed < 400000) { rc.values[7] = 2000; }
	host::publish("input_rc", rc);
}
static void setup(int which)
{
	host::topics.clear(); host::gpio_writes.clear(); host::parameters.clear(); host::param_names.clear();
	scenario = which; started = host::now;
	TestMode::running(false);
	host::parameter<int32_t>("VGS_ENABLE", 0); host::parameter<int32_t>("VGS_CLN_EN", 0);
	host::sleep_hook = refresh; refresh();
}
static int command(const char *channel = "8")
{
	char name[] = "test-rc-valve";
	char *args[] = {name, const_cast<char *>(channel)};
	pthread_mutex_lock(&px4_modules_mutex);
	const int result = VgsolarMode::custom_command(2, args);
	pthread_mutex_unlock(&px4_modules_mutex);
	return result;
}
int main()
{
	for (int which = 1; which <= 7; ++which) {
		setup(which); assert(command() == 0);
		assert(host::now - started == 60000000);
		unsigned on_count = 0;
		for (const auto &write : host::gpio_writes) {
			if (write.pin == GPIO_VG_PUMP_OFF) { assert(!write.value); }
			if (write.pin == GPIO_VG_VALVE_OFF && !write.value) {
				const auto elapsed = write.time - started;
				assert(elapsed == 500000 || (which != 7 && elapsed == 100000));
				++on_count;
			}
		}
		assert(on_count == (which == 7 ? 1u : 2u));
		assert(host::gpio[GPIO_VG_VALVE_OFF] && !host::gpio[GPIO_VG_PUMP_OFF]);
	}
	setup(8); assert(command() != 0 && host::now - started == 600000);
	assert(host::gpio[GPIO_VG_VALVE_OFF] && !host::gpio[GPIO_VG_PUMP_OFF]);
	for (const char *bad : {"0", "19", "8x", ""}) {
		setup(0); assert(command(bad) != 0 && host::gpio_writes.empty());
	}
	setup(0); TestMode::running(true);
	assert(command() != 0 && host::gpio_writes.empty());
	puts("PASS: production RC8 valve polarity, pump isolation, 60s timeout, LOW-first and reconnect interlock, loss/failsafe/stale/short/invalid RC, arming abort and argument validation");
}
