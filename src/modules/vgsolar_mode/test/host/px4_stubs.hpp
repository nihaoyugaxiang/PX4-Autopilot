// SPDX-License-Identifier: BSD-3-Clause
#pragma once
// Deterministic host substitutes for scheduling, uORB transport, parameters and GPIO.
// Message layouts and quaternion math come from the actual PX4 build/source.
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

#define __EXPORT
#define MODULE_NAME "vgsolar_mode_test"
#define PX4_ERROR -1
#define PX4_OK 0
#define PX4_INFO(...) ((void)0)
#define PX4_WARN(...) ((void)0)
#define PX4_ISFINITE(x) std::isfinite(x)
#define PRINT_MODULE_USAGE_NAME(...) ((void)0)
#define PRINT_MODULE_USAGE_COMMAND(...) ((void)0)
#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS(...) ((void)0)
constexpr int task_id_is_work_queue = -2;
struct orb_metadata { const char *name; };
#define ORB_DECLARE(name) static const orb_metadata __orb_##name{#name}
#define ORB_ID(name) (&__orb_##name)

namespace host {
using Message = std::vector<uint8_t>;
static uint64_t now = 1000000;
static std::map<std::string, std::vector<Message>> topics;
static std::map<int, bool> gpio;
static std::vector<std::string> param_names;
static std::map<std::string, Message> parameters;
static bool reboot_requested = false;
template<typename T> void publish(const char *name, const T &value) {
	Message bytes(sizeof(T)); std::memcpy(bytes.data(), &value, sizeof(T));
	topics[name].push_back(bytes);
}
template<typename T> T latest(const char *name) {
	T value{}; auto found = topics.find(name);
	if (found != topics.end() && !found->second.empty()) {
		const Message &bytes = found->second.back();
		if (bytes.size() == sizeof(T)) { std::memcpy(&value, bytes.data(), sizeof(T)); }
	}
	return value;
}
template<typename T> void parameter(const char *name, const T &value) {
	Message bytes(sizeof(T)); std::memcpy(bytes.data(), &value, sizeof(T)); parameters[name] = bytes;
}
}
inline uint64_t hrt_absolute_time() { return host::now; }
using param_t = int;
constexpr param_t PARAM_INVALID = -1;
inline param_t param_find(const char *name) {
	if (!host::parameters.count(name)) { return PARAM_INVALID; }
	for (unsigned i = 0; i < host::param_names.size(); ++i) { if (host::param_names[i] == name) { return i; } }
	host::param_names.emplace_back(name); return host::param_names.size() - 1;
}
inline int param_get(param_t index, void *value) {
	if (index < 0 || static_cast<unsigned>(index) >= host::param_names.size()) { return -1; }
	const auto &bytes = host::parameters.at(host::param_names[index]);
	std::memcpy(value, bytes.data(), bytes.size()); return 0;
}
constexpr int GPIO_VG_VALVE_OFF = 5;
constexpr int GPIO_VG_PUMP_OFF = 6;
inline void px4_arch_gpiowrite(int pin, bool value) { host::gpio[pin] = value; }
constexpr int REBOOT_REQUEST = 1;
inline void px4_reboot_request(int, unsigned) { host::reboot_requested = true; }
namespace px4 {
namespace wq_configurations { constexpr int lp_default = 0; }
class ScheduledWorkItem {
public:
	ScheduledWorkItem(const char *, int) {}
	virtual ~ScheduledWorkItem() = default;
	virtual void Run() = 0;
	void ScheduleOnInterval(unsigned) {}
	void ScheduleClear() {}
};
}
template<typename T> class ModuleBase {
public:
	virtual ~ModuleBase() = default;
	virtual int print_status() { return 0; }
	bool should_exit() const { return false; }
	void exit_and_cleanup() {}
	static int main(int, char *[]) { return 0; }
protected:
	static std::atomic<T *> _object;
	static int _task_id;
};
template<typename T> std::atomic<T *> ModuleBase<T>::_object{nullptr};
template<typename T> int ModuleBase<T>::_task_id = 0;
namespace uORB {
template<typename T> class Publication {
public:
	explicit Publication(const orb_metadata *meta) : _name(meta->name) {}
	bool publish(const T &value) { host::publish(_name, value); return true; }
private:
	const char *_name;
};
class Subscription {
public:
	explicit Subscription(const orb_metadata *meta) : _name(meta->name) {}
	template<typename T> bool copy(T *value) {
		auto &queue = host::topics[_name];
		if (queue.empty()) { return false; }
		if (queue.back().size() != sizeof(T)) { return false; }
		std::memcpy(value, queue.back().data(), sizeof(T)); _cursor = queue.size(); return true;
	}
	template<typename T> bool update(T *value) {
		auto &queue = host::topics[_name];
		if (_cursor >= queue.size() || queue[_cursor].size() != sizeof(T)) { return false; }
		std::memcpy(value, queue[_cursor++].data(), sizeof(T)); return true;
	}
private:
	const char *_name;
	size_t _cursor{0};
};
template<typename T> class SubscriptionData : public Subscription {
public:
	explicit SubscriptionData(const orb_metadata *meta) : Subscription(meta) {}
};
}
