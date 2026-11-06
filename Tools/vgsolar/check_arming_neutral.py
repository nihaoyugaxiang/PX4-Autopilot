#!/usr/bin/env python3
"""Run production output-limit state machine with a host clock and PWM-map stub."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'src/lib/mixer_module/mixer_module.cpp').read_text()
start = source.index('void\nMixingOutput::output_limit_calc(')
opening = source.index('{', start)
depth = 1
end = opening + 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
production = source[start:end]
harness = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
using hrt_abstime = uint64_t;
constexpr hrt_abstime operator""_ms(unsigned long long n) { return n * 1000; }
static hrt_abstime now = 1000000;
static hrt_abstime hrt_absolute_time() { return now; }
static hrt_abstime hrt_elapsed_time(const hrt_abstime *t) { return now - *t; }
constexpr int MAX_ACTUATORS = 4;
struct FunctionProvider {
 bool prearm_allowed = true;
 bool allowPrearmControl() const { return prearm_allowed; }
};
class MixingOutput {
public:
 enum class OutputLimitState { OFF, RAMP, ON };
 OutputLimitState _output_state = OutputLimitState::OFF;
 bool _output_ramp_up = true;
 bool prearmed = false;
 hrt_abstime _output_time_armed = 0;
 uint32_t _reversible_mask = 3;
 FunctionProvider *_functions[MAX_ACTUATORS] = {};
 uint16_t _disarmed_value[MAX_ACTUATORS] = {0, 0, 900, 0};
 uint16_t _current_output_value[MAX_ACTUATORS] = {};
 bool isPrearmed() const { return prearmed; }
 // PWM mapping is modeled; the complete production arming state machine is compiled below.
 uint16_t output_limit_calc_single(int i, float v) const {
  return std::isfinite(v) ? uint16_t(std::lround(1500.f + 400.f * v)) : _disarmed_value[i];
 }
 void output_limit_calc(bool armed, int num_channels, const float output[MAX_ACTUATORS]);
};
''' + production + r'''
int main() {
 MixingOutput m;
 FunctionProvider esc{false};
 m._functions[2] = &esc;
 float values[4] = {0.f, 0.f, 0.f, NAN};
 m.output_limit_calc(false, 4, values);
 assert(m._current_output_value[0] == 0 && m._current_output_value[2] == 900);
 const auto start = now;
 for (auto dt : {0ULL, 100000ULL, 250000ULL, 499999ULL, 500000ULL, 600000ULL}) {
  now = start + dt;
  m.output_limit_calc(true, 4, values);
  assert(m._current_output_value[0] == 1500 && m._current_output_value[1] == 1500);
  assert(m._current_output_value[3] == 0);
  if (dt < 500000) {
   assert(m._current_output_value[2] == uint16_t(900 + (dt / 500000.f) * 600));
  } else { assert(m._current_output_value[2] == 1500); }
 }
 m.output_limit_calc(false, 4, values);
 assert(m._current_output_value[0] == 0 && m._current_output_value[1] == 0);
 // A fresh arming cycle must not replay the disarmed-to-center sweep.
 now += 10000;
 m.output_limit_calc(true, 4, values);
 assert(m._current_output_value[0] == 1500 && m._current_output_value[1] == 1500);
 // Valid signed commands pass through; invalid values still revoke output.
 values[0] = -.25f; values[1] = .25f;
 now += 10000;
 m.output_limit_calc(true, 4, values);
 assert(m._current_output_value[0] == 1400 && m._current_output_value[1] == 1600);
 values[0] = NAN;
 m.output_limit_calc(true, 4, values);
 assert(m._current_output_value[0] == 0);
 m.output_limit_calc(false, 4, values);
 assert(m._current_output_value[1] == 0);
 // Only the marked output channel bypasses ramp, including a swapped mapping.
 MixingOutput swapped;
 swapped._reversible_mask = 2;
 values[0] = 0.f; values[1] = 0.f;
 swapped.output_limit_calc(true, 4, values);
 assert(swapped._current_output_value[0] == 0 && swapped._current_output_value[1] == 1500);
 MixingOutput no_ramp;
 no_ramp._output_ramp_up = false;
 no_ramp.output_limit_calc(true, 4, values);
 assert(no_ramp._current_output_value[0] == 1500 && no_ramp._current_output_value[2] == 1500);
 // A servo/RC passthrough that can run prearmed must retain its selected
 // pulse across formal arming, rather than replaying the ESC ramp from zero.
 FunctionProvider servo;
 for (float command : {-1.f, 0.f, 1.f}) {
  MixingOutput lift;
  lift._functions[3] = &servo;
  lift._disarmed_value[3] = 0;
  values[3] = command;
  const auto expected = lift.output_limit_calc_single(3, command);
  lift.prearmed = true;
  lift.output_limit_calc(false, 4, values);
  assert(lift._current_output_value[3] == expected);
  lift.prearmed = false;
  const auto armed_at = now;
  for (auto dt : {0ULL, 100000ULL, 250000ULL, 499999ULL, 500000ULL, 600000ULL}) {
   now = armed_at + dt;
   lift.output_limit_calc(true, 4, values);
   assert(lift._current_output_value[3] == expected);
  }
  values[3] = NAN;
  lift.output_limit_calc(true, 4, values);
  assert(lift._current_output_value[3] == 0);
  lift.output_limit_calc(false, 4, values);
  assert(lift._current_output_value[3] == 0);
 }
}
'''
harness = harness.replace('#include <cassert>', '#include <cassert>\n#include <initializer_list>')
with tempfile.TemporaryDirectory(prefix='vgsolar-arming-neutral-') as directory:
    directory = Path(directory)
    (directory / 'check.cpp').write_text(harness)
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                    str(directory / 'check.cpp'), '-o', str(directory / 'check')], check=True)
    subprocess.run([str(directory / 'check')], check=True)
print('PASS: production output-limit reversible neutral and prearmed servo continuity; signed commands, invalid/disarmed output and ordinary ESC ramp preserved')
