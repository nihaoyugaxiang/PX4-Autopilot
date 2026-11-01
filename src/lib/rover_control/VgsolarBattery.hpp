// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <px4_platform_common/param.h>
namespace vgsolar {
inline float battery_remaining(float voltage, float native_remaining, bool connected) {
 int32_t source = 1; float low = 19.8f, high = 25.2f;
 const param_t src = param_find("VGS_BAT_SRC"), lo = param_find("VGS_BAT_MIN"), hi = param_find("VGS_BAT_MAX");
 if (src != PARAM_INVALID) { param_get(src, &source); }
 if (lo != PARAM_INVALID) { param_get(lo, &low); }
 if (hi != PARAM_INVALID) { param_get(hi, &high); }
 if (!connected) { return NAN; }
 if (source == 0) { return std::isfinite(native_remaining) && native_remaining >= 0.f && native_remaining <= 1.f ? native_remaining : NAN; }
 if (source != 1 || !std::isfinite(voltage) || voltage <= 0.f || !std::isfinite(low) || !std::isfinite(high) || high <= low) { return NAN; }
 return roundf(fminf(100.f, fmaxf(0.f, 100.f * (voltage - low) / (high - low)))) * .01f;
}
}
