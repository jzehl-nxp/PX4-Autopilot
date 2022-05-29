/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "VehicleOpticalFlow.hpp"

#include <px4_platform_common/log.h>

namespace sensors
{

using namespace matrix;
using namespace time_literals;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleOpticalFlow::VehicleOpticalFlow() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_optical_flow_pub.advertise();

	_gyro_integrator.set_reset_samples(1);
}

VehicleOpticalFlow::~VehicleOpticalFlow()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleOpticalFlow::Start()
{
	_sensor_flow_sub.registerCallback();

	_sensor_gyro_sub.registerCallback();
	_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH);

	ScheduleNow();
	return true;
}

void VehicleOpticalFlow::Stop()
{
	Deinit();

	// clear all registered callbacks
	_sensor_flow_sub.unregisterCallback();
	_sensor_gyro_sub.unregisterCallback();
}

void VehicleOpticalFlow::ParametersUpdate()
{
	// Check if parameters have changed
	if (_params_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();
	}
}

void VehicleOpticalFlow::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	while (_sensor_gyro_sub.updated()) {
		const unsigned last_generation = _sensor_gyro_sub.get_last_generation();

		sensor_gyro_s sensor_gyro;

		if (_sensor_gyro_sub.update(&sensor_gyro)) {

			if (_sensor_gyro_sub.get_last_generation() != last_generation + 1) {
				PX4_ERR("sensor_gyro lost, generation %u -> %u", last_generation, _sensor_gyro_sub.get_last_generation());
			}

			_gyro_calibration.set_device_id(sensor_gyro.device_id);
			_gyro_calibration.SensorCorrectionsUpdate();

			const float dt_s = (sensor_gyro.timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;
			_gyro_timestamp_sample_last = sensor_gyro.timestamp_sample;

			gyroSample gyro_sample;
			gyro_sample.time_us = sensor_gyro.timestamp_sample;
			gyro_sample.data = _gyro_calibration.Correct(Vector3f{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z});
			gyro_sample.dt = dt_s;

			_gryo_buffer.push(gyro_sample);
		}
	}

	if (_distance_sensor_sub.updated()) {
		distance_sensor_s distance_sensor;

		if (_distance_sensor_sub.copy(&distance_sensor)) {
			if ((distance_sensor.current_distance > distance_sensor.min_distance)
			    && (distance_sensor.current_distance < distance_sensor.max_distance)) {
				_range = distance_sensor.current_distance;
			}
		}
	}


	sensor_optical_flow_s sensor_optical_flow;

	if (_sensor_flow_sub.update(&sensor_optical_flow)) {

		// only process valid samples
		if ((sensor_optical_flow.dt > 1000) && (sensor_optical_flow.dt < 60'000)
		    && (sensor_optical_flow.quality > 0)
		    && PX4_ISFINITE(sensor_optical_flow.pixel_flow[0])
		    && PX4_ISFINITE(sensor_optical_flow.pixel_flow[1])) {

			gyroSample gyro_sample;
			const hrt_abstime timestamp_oldest = sensor_optical_flow.timestamp_sample - lroundf(sensor_optical_flow.dt);
			const hrt_abstime timestamp_newest = sensor_optical_flow.timestamp;

			while (_gryo_buffer.pop_oldest(timestamp_oldest, timestamp_newest, &gyro_sample)) {

				_gyro_integrator.put(gyro_sample.data, gyro_sample.dt);

				float min_interval_s = (sensor_optical_flow.dt * 1e-6f) - (0.5f * gyro_sample.dt);

				if (_gyro_integrator.integral_dt() >= min_interval_s) {
					//PX4_INFO("integral dt: %.6f, min interval: %.6f", (double)_gyro_integrator.integral_dt(),(double) min_interval_s);
					break;
				}
			}


			vehicle_optical_flow_s vehicle_optical_flow{};

			vehicle_optical_flow.timestamp_sample = sensor_optical_flow.timestamp_sample;
			vehicle_optical_flow.device_id = sensor_optical_flow.device_id;
			vehicle_optical_flow.pixel_flow_x_integral = sensor_optical_flow.pixel_flow[0];
			vehicle_optical_flow.pixel_flow_y_integral = sensor_optical_flow.pixel_flow[1];
			vehicle_optical_flow.integration_timespan = sensor_optical_flow.dt;

			//vehicle_optical_flow.ground_distance_m = sensor_optical_flow.ground_distance_m;

			vehicle_optical_flow.quality = sensor_optical_flow.quality;

			if (sensor_optical_flow.delta_angle_available
			    && PX4_ISFINITE(sensor_optical_flow.delta_angle[0])
			    && PX4_ISFINITE(sensor_optical_flow.delta_angle[1])
			    && PX4_ISFINITE(sensor_optical_flow.delta_angle[2])
			   ) {
				// passthrough integrated gyro if available
				vehicle_optical_flow.gyro_x_rate_integral = sensor_optical_flow.delta_angle[0];
				vehicle_optical_flow.gyro_y_rate_integral = sensor_optical_flow.delta_angle[1];
				vehicle_optical_flow.gyro_z_rate_integral = sensor_optical_flow.delta_angle[2];

			} else {
				Vector3f delta_angle;
				uint16_t delta_angle_dt;

				if (_gyro_integrator.reset(delta_angle, delta_angle_dt)) {

					vehicle_optical_flow.gyro_x_rate_integral = delta_angle(0);
					vehicle_optical_flow.gyro_y_rate_integral = delta_angle(1);
					vehicle_optical_flow.gyro_z_rate_integral = delta_angle(2);

					vehicle_optical_flow.integration_timespan_gyro = delta_angle_dt;

					{
						// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
						// is produced by a RH rotation of the image about the sensor axis.
						Vector2f flow_xy_rad{-sensor_optical_flow.pixel_flow[0], -sensor_optical_flow.pixel_flow[1]};

						Vector2f flow_compensated_XY_rad = flow_xy_rad - Vector2f(-delta_angle(0), -delta_angle(1));

						float range = _range;

						const float flow_dt = sensor_optical_flow.dt * 1e-6f;

						Vector2f vel_optflow_body;
						vel_optflow_body(0) = - range * flow_compensated_XY_rad(1) / flow_dt;
						vel_optflow_body(1) =   range * flow_compensated_XY_rad(0) / flow_dt;

						vel_optflow_body.copyTo(vehicle_optical_flow.flow_vel_body);
						vehicle_optical_flow.height = range;
					}

				} else {
					vehicle_optical_flow.gyro_x_rate_integral = NAN;
					vehicle_optical_flow.gyro_y_rate_integral = NAN;
					vehicle_optical_flow.gyro_z_rate_integral = NAN;

					// force integrator reset
					_gyro_integrator.reset();
				}
			}

			vehicle_optical_flow.max_flow_rate = sensor_optical_flow.max_flow_rate;
			vehicle_optical_flow.min_ground_distance = sensor_optical_flow.min_ground_distance;
			vehicle_optical_flow.max_ground_distance = sensor_optical_flow.max_ground_distance;

			vehicle_optical_flow.timestamp = hrt_absolute_time();

			_vehicle_optical_flow_pub.publish(vehicle_optical_flow);

		}
	}

	// reschedule backup
	ScheduleDelayed(10_ms);

	perf_end(_cycle_perf);
}

void VehicleOpticalFlow::PrintStatus()
{

}

}; // namespace sensors
