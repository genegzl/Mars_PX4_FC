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

#pragma once

#include <sensor_calibration/SensorCalibration.hpp>

#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>

namespace sensors
{

class VehicleAngularVelocity : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleAngularVelocity();
	~VehicleAngularVelocity() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void CheckFilters();
	void ParametersUpdate(bool force = false);
	void SensorBiasUpdate(bool force = false);
	bool SensorSelectionUpdate(bool force = false);

	static constexpr int MAX_SENSOR_COUNT = 3;

	uORB::Publication<vehicle_angular_acceleration_s> _vehicle_angular_acceleration_pub{ORB_ID(vehicle_angular_acceleration)};
	uORB::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _params_sub{ORB_ID(parameter_update)};

	uORB::SubscriptionCallbackWorkItem _sensor_selection_sub{this, ORB_ID(sensor_selection)};
	uORB::SubscriptionCallbackWorkItem _sensor_sub[MAX_SENSOR_COUNT] {
		{this, ORB_ID(sensor_gyro), 0},
		{this, ORB_ID(sensor_gyro), 1},
		{this, ORB_ID(sensor_gyro), 2}
	};

	SensorCalibration _calibration{SensorCalibration::SensorType::Gyroscope};

	matrix::Vector3f _bias{0.f, 0.f, 0.f};

	matrix::Vector3f _angular_acceleration_prev{0.f, 0.f, 0.f};
	matrix::Vector3f _angular_velocity_prev{0.f, 0.f, 0.f};
	hrt_abstime _timestamp_sample_prev{0};

	hrt_abstime _last_publish{0};
	static constexpr const float kInitialRateHz{1000.0f}; /**< sensor update rate used for initialization */
	float _update_rate_hz{kInitialRateHz}; /**< current rate-controller loop update rate in [Hz] */

	uint8_t _required_sample_updates{0}; /**< number or sensor publications required for configured rate */

	// angular velocity filters
	math::LowPassFilter2pVector3f _lp_filter_velocity{kInitialRateHz, 30.0f};
	math::NotchFilter<matrix::Vector3f> _notch_filter_velocity{};
	math::NotchFilter<matrix::Vector3f> _notch_filter_velocity_2nd{};
	math::NotchFilter<matrix::Vector3f> _notch_filter_velocity_3rd{};
	math::NotchFilter<matrix::Vector3f> _notch_filter_velocity_4rd{};


	// angular acceleration filter
	math::LowPassFilter2pVector3f _lp_filter_acceleration{kInitialRateHz, 30.0f};

	float _filter_sample_rate{kInitialRateHz};

	uint32_t _selected_sensor_device_id{0};
	uint8_t _selected_sensor_sub_index{0};

	hrt_abstime _timestamp_sample_last{0};
	float _interval_sum{0.f};
	float _interval_count{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::IMU_GYRO_CUTOFF>) _param_imu_gyro_cutoff,
		(ParamFloat<px4::params::IMU_GYRO_NF_FREQ>) _param_imu_gyro_nf_freq,
		(ParamFloat<px4::params::IMU_GYRO_NF_BW>) _param_imu_gyro_nf_bw,
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_rate_max,

		(ParamFloat<px4::params::IMU_DGYRO_CUTOFF>) _param_imu_dgyro_cutoff
	)
};

} // namespace sensors
