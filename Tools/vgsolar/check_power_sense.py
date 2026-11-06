#!/usr/bin/env python3
"""Exercise the production power arming check with/without 5V sensing."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'src/modules/commander/HealthAndArmingChecks/checks/powerCheck.cpp').read_text()
production = source[source.index('void PowerChecks::checkAndReport('):]
board = (ROOT / 'boards/vg/vgsolar/src/board_config.h').read_text()
assert '#define BOARD_HAS_NO_5V_SENSE 1' in board
assert '#define ADC_SCALED_V5_CHANNEL' not in board and '#define ADC_5V_RAIL_SENSE' not in board
harness = r"""
#include <cstdint>
#include <cstdlib>
#include <cinttypes>
#include <string>
#include <vector>
#include <algorithm>
struct system_power_s {
 bool usb_connected = false;
 float voltage5v_v = 0.f;
 uint8_t brick_valid = 1;
 bool hipower_5v_oc = false, periph_5v_oc = false;
};
static system_power_s power;
static bool has_power = true;
struct vehicle_status_s { static constexpr int HIL_STATE_ON=1; int hil_state=0; bool power_input_valid=true; };
struct Context { vehicle_status_s value; bool armed=false; const vehicle_status_s &status() const {return value;} bool isArmed() const {return armed;} };
namespace NavModes { constexpr int All=0; }
namespace health_component_t { constexpr int system=0; }
namespace events { namespace Log { constexpr int Error=0; } const char *ID(const char *s) {return s;} void send(const char *, int, const char *) {} }
namespace math { int countSetBits(unsigned n) {return __builtin_popcount(n);} }
struct Report {
 std::vector<std::string> failures;
 template<class... T, class... A> void healthFailure(int, int, const char *id, int, A...) {failures.emplace_back(id);}
 template<class... T, class... A> void armingCheckFailure(int, int, const char *id, int, A...) {failures.emplace_back(id);}
 bool mavlink_log_pub() const {return false;}
};
template<class... A> void mavlink_log_critical(A...) {}
[[maybe_unused]] static uint64_t hrt_absolute_time() {return 1000000;}
constexpr int CBRK_SUPPLY_CHK_KEY=894281;
static bool circuit_breaker_enabled_by_val(int v, int key) {return v==key;}
struct Param {int value=0; int get() const {return value;}};
struct Hysteresis {bool value=false; void set_state_and_update(bool v, uint64_t) {value=v;} bool get_state() const {return value;}};
struct Subscription {bool copy(system_power_s *out) {*out=power; return has_power;}};
class PowerChecks {
public:
 Subscription _system_power_sub;
 Param _param_cbrk_supply_chk, _param_com_power_count{1};
 Hysteresis _voltage_low_hysteresis, _voltage_high_hysteresis;
 bool _overcurrent_warning_sent=false;
 void checkAndReport(const Context &, Report &);
};
static void require(bool value) {if (!value) std::exit(1);}
""" + production + r"""
static bool has(Report &r, const char *id) {return std::find(r.failures.begin(),r.failures.end(),id)!=r.failures.end();}
int main() {
 Context c;
 PowerChecks p;
 Report r;
 p.checkAndReport(c,r);
#ifdef BOARD_HAS_NO_5V_SENSE
 require(r.failures.empty()); // No fabricated zero-volt rejection without a sensor.
#else
 require(has(r,"check_avionics_power_low")); // An actual 0V reading must still reject.
#endif
 r.failures.clear(); power.voltage5v_v=5.f; p.checkAndReport(c,r); require(r.failures.empty());
 r.failures.clear(); power.voltage5v_v=5.8f; p.checkAndReport(c,r);
#ifdef BOARD_HAS_NO_5V_SENSE
 require(r.failures.empty());
#else
 require(has(r,"check_avionics_power_high"));
#endif
 // USB behavior and every non-voltage power guard remain active as before.
 r.failures.clear(); power.usb_connected=true; power.voltage5v_v=0; p.checkAndReport(c,r); require(r.failures.empty());
 r.failures.clear(); power.usb_connected=false; power.voltage5v_v=5; power.brick_valid=0;
 p.checkAndReport(c,r); require(has(r,"check_avionics_power_redundancy"));
 r.failures.clear(); power.brick_valid=1; power.hipower_5v_oc=true; power.periph_5v_oc=true;
 p.checkAndReport(c,r); require(has(r,"check_power_oc_hipower") && has(r,"check_power_oc_periph"));
 r.failures.clear(); c.value.power_input_valid=false; p.checkAndReport(c,r); require(has(r,"check_avionics_power_missing"));
 r.failures.clear(); c.value.power_input_valid=true; has_power=false; p.checkAndReport(c,r); require(has(r,"check_missing_system_power"));
 r.failures.clear(); p._param_cbrk_supply_chk.value=CBRK_SUPPLY_CHK_KEY; p.checkAndReport(c,r); require(r.failures.empty());
 r.failures.clear(); p._param_cbrk_supply_chk.value=0; c.value.hil_state=vehicle_status_s::HIL_STATE_ON; p.checkAndReport(c,r); require(r.failures.empty());
}
"""
with tempfile.TemporaryDirectory(prefix='vgsolar-power-check-') as directory:
    directory = Path(directory)
    cpp = directory / 'check.cpp'
    cpp.write_text(harness)
    for no_sensor in (False, True):
        binary = directory / ('no-sensor' if no_sensor else 'sensor')
        flags = ['-DBOARD_HAS_NO_5V_SENSE=1'] if no_sensor else []
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', *flags, str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('PASS: production power checks with/without 5V sensing; real zero/high voltage rejects, USB behavior, missing source/topic, redundancy and overcurrent preserved')
