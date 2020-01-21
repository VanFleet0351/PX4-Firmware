/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "VehicleAirData.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

VehicleAirData::VehicleAirData() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::att_pos_ctrl)
{
	_voter.set_timeout(300000);
}

VehicleAirData::~VehicleAirData()
{
	Stop();
}

bool VehicleAirData::Start()
{
	// force initial updates
	ParametersUpdate(true);

	ScheduleNow();
	return true;
}

void VehicleAirData::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleAirData::SensorCorrectionsUpdate(bool force)
{
	// check if the selected sensor has updated
	if (_sensor_correction_sub.updated() || force) {

		sensor_correction_s corrections{};
		_sensor_correction_sub.copy(&corrections);

		// selected sensor has changed, find updated index
		if ((_corrections_selected_instance < 0) || force) {
			_corrections_selected_instance = -1;

			// find sensor_corrections index
			for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
				if (corrections.baro_device_ids[i] == _selected_sensor_device_id) {
					_corrections_selected_instance = i;
				}
			}
		}

		switch (_corrections_selected_instance) {
		case 0:
			_offset = corrections.baro_offset_0;
			_scale = corrections.baro_scale_0;
			break;

		case 1:
			_offset = corrections.baro_offset_1;
			_scale = corrections.baro_scale_1;
			break;

		case 2:
			_offset = corrections.baro_offset_2;
			_scale = corrections.baro_scale_2;
			break;

		default:
			_offset = 0.f;
			_scale = 1.f;
		}
	}
}

void VehicleAirData::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();
	}
}

void VehicleAirData::Run()
{
	// TODO:
	// 1. grab all sensor data and push into voter
	// 2. check failover
	//     - register callback to new primary sensor
	// 3. take current primary data and publish vehicle_air_data
	//
	// 5. Schedule timeout callback ()


	// Questions??
	//  - how to handle priorities
	//  - report failers?
	//  - structure for setting QNH from GPS



	for (int uorb_index = 0; uorb_index < _subscription_count; uorb_index++) {
		sensor_baro_s baro_report{};

		if (_sensor_sub[uorb_index].update(&baro_report)) {
			float vect[3] = {baro_report.pressure, baro_report.temperature, 0.f};
			_voter.put(uorb_index, baro_report.timestamp, vect, baro_report.error_count, _priority[uorb_index]);
		}
	}



	int best_index;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		//airdata = _last_airdata[best_index];

		if (_last_best_vote != best_index) {
			_last_best_vote = (uint8_t)best_index;
		}

		// if (_selection.baro_device_id != _baro_device_id[best_index]) {
		// 	_selection_changed = true;
		// 	_selection.baro_device_id = _baro_device_id[best_index];
		// }


		_selected_sensor_sub_index = best_index;
		//
	}





	// update corrections first to set _selected_sensor
	bool sensor_select_update = false;
	SensorCorrectionsUpdate(sensor_select_update);
	ParametersUpdate();

	if (_sensor_sub[_selected_sensor_sub_index].updated() || sensor_select_update) {
		sensor_baro_s sensor_data;

		if (_sensor_sub[_selected_sensor_sub_index].copy(&sensor_data)) {


			_selected_sensor_device_id = sensor_data.device_id;

			// get the sensor data and correct for thermal errors (apply offsets and scale)

			// Convert from millibar to Pa and apply temperature compensation
			float corrected_pressure = (100.0f * sensor_data.pressure - _offset) * _scale;

			// publish
			vehicle_air_data_s out;

			//out.timestamp_sample = sensor_data.timestamp_sample;
			out.baro_device_id = sensor_data.device_id;
			out.baro_pressure_pa = corrected_pressure;



			// calculate altitude using the hypsometric equation

			static constexpr float T1 = 15.0f - CONSTANTS_ABSOLUTE_NULL_CELSIUS;	/* temperature at base height in Kelvin */
			static constexpr float a  = -6.5f / 1000.0f;	/* temperature gradient in degrees per metre */

			/* current pressure at MSL in kPa (QNH in hPa)*/
			const float p1 = _param_sens_baro_qnh.get() * 0.1f;

			/* measured pressure in kPa */
			const float p = out.baro_pressure_pa * 0.001f;

			/*
			 * Solve:
			 *
			 *     /        -(aR / g)     \
			 *    | (p / p1)          . T1 | - T1
			 *     \                      /
			 * h = -------------------------------  + h1
			 *                   a
			 */
			out.baro_alt_meter = (((powf((p / p1), (-(a * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * T1) - T1) / a;


			// calculate air density
			// estimate air density assuming typical 20degC ambient temperature
			// TODO: use air temperature if available (differential pressure sensors)
			static constexpr float pressure_to_density = 1.0f / (CONSTANTS_AIR_GAS_CONST * (20.0f -
					CONSTANTS_ABSOLUTE_NULL_CELSIUS));
			out.rho = pressure_to_density * out.baro_pressure_pa;



			out.timestamp = hrt_absolute_time();

			_vehicle_air_data_pub.publish(out);
		}
	}



	// TODO: make sure we're subscribed to primary baro

	// TODO: need ScheduleDelayed() timeout


	// TODO: add checkFailover() equivalent
	// mavlink_log_emergency(&_mavlink_log_pub, "%s #%i fail: %s%s%s%s%s!",
	// 			sensor_name,
	// 			failover_index,
	// 			((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
	// 			((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
	// 			((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
	// 			((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
	// 			((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));


	ScheduleDelayed(10_ms);

}

void VehicleAirData::PrintStatus()
{
	PX4_INFO("selected sensor: %d (%d)", _selected_sensor_device_id, _selected_sensor_sub_index);
	PX4_INFO("offset: %.3f", (double)_offset);
	PX4_INFO("scale: %.3f", (double)_scale);

	_voter.print();
}
