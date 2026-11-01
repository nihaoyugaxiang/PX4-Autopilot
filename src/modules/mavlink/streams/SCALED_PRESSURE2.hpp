/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef SCALED_PRESSURE2_HPP
#define SCALED_PRESSURE2_HPP

#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/sensor_baro.h>

#include <board_config.h>
#ifdef VGSOLAR_PRESSURE_ADC_CHANNEL
#include <uORB/topics/vgsolar_inputs.h>
#endif

class MavlinkStreamScaledPressure2 : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamScaledPressure2(mavlink); }

	static constexpr const char *get_name_static() { return "SCALED_PRESSURE2"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SCALED_PRESSURE2; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
#ifdef VGSOLAR_PRESSURE_ADC_CHANNEL
 if (_suction_sub.advertised()) { return MAVLINK_MSG_ID_SCALED_PRESSURE2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }
#endif
		if (_sensor_baro_sub.advertised() || _differential_pressure_sub.advertised()) {
			return MAVLINK_MSG_ID_SCALED_PRESSURE2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
#ifdef VGSOLAR_PRESSURE_ADC_CHANNEL
 uORB::Subscription _suction_sub{ORB_ID(vgsolar_inputs)};
 uint64_t _last_suction{0};
#endif
	explicit MavlinkStreamScaledPressure2(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure), 1};
	uORB::Subscription _sensor_baro_sub{ORB_ID(sensor_baro), 1};

	bool send() override
	{
#ifdef VGSOLAR_PRESSURE_ADC_CHANNEL
 const uint64_t now = hrt_absolute_time();
 if (_suction_sub.advertised() && now - _last_suction >= 500000) {
  vgsolar_inputs_s input{}; _suction_sub.copy(&input); _last_suction = now;
  const bool valid = input.enabled && input.pressure_valid && input.timestamp > 0 && input.timestamp <= now
      && now - input.timestamp < 500000 && PX4_ISFINITE(input.pressure_kpa);
  mavlink_scaled_pressure2_t msg{}; msg.time_boot_ms = now / 1000;
  // APM compatibility: signed suction gauge pressure, kPa -> hPa; not barometric altitude.
  msg.press_abs = valid ? input.pressure_kpa * 10.f : NAN;
  mavlink_msg_scaled_pressure2_send_struct(_mavlink->get_channel(), &msg);
 return true;
 }
 if (_suction_sub.advertised()) { return false; }
#endif

		if (_sensor_baro_sub.updated() || _differential_pressure_sub.updated()) {
			mavlink_scaled_pressure2_t msg{};

			sensor_baro_s sensor_baro;

			if (_sensor_baro_sub.copy(&sensor_baro)) {
				msg.time_boot_ms = sensor_baro.timestamp / 1000;
				msg.press_abs = sensor_baro.pressure * 0.01f; // Pa to hPa
				msg.temperature = roundf(sensor_baro.temperature * 100.f); // cdegC (centidegrees)
			}

			differential_pressure_s differential_pressure;

			if (_differential_pressure_sub.copy(&differential_pressure)) {
				if (msg.time_boot_ms == 0) {
					msg.time_boot_ms = differential_pressure.timestamp / 1000;
				}

				msg.press_diff = differential_pressure.differential_pressure_pa * 0.01f; // Pa to hPa
				msg.temperature_press_diff = roundf(differential_pressure.temperature * 100.f); // cdegC (centidegrees)
			}

			mavlink_msg_scaled_pressure2_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // SCALED_PRESSURE2_HPP
