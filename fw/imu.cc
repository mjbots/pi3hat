// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fw/imu.h"

namespace fw {

void Imu::DoImu() {
  const auto start = timer_->read_us();

  if (reset_estimator_.load()) {
    reset_estimator_.store(false);
    if (bmi088_) {
      ConfigureBmi088();
    } else {
      ConfigureIcm42688();
    }
    attitude_reference_.emplace();
  }


  const auto unrotated_data =
      (!!bmi088_) ? bmi088_->read_data() : icm42688_->read_data();

  auto data = unrotated_data;
  data.rate_dps = mounting_.Rotate(data.rate_dps);
  data.accel_mps2 = mounting_.Rotate(data.accel_mps2);

  auto& imu_data = [&]() -> ImuData& {
    for (auto& item : imu_data_buffer_) {
      if (item.active.load() == false) {
        // Nothing is using it, and we're the only one who can set
        // it to true, so this won't change.
        return item;
      }
    }
    mbed_die();
  }();

  imu_data.imu = ImuRegister{data};

  attitude_reference_->ProcessMeasurement(
      period_s_,
      (static_cast<float>(M_PI) / 180.0f) * data.rate_dps,
      data.accel_mps2);

  AttitudeRegister& my_att = imu_data.attitude;
  my_att.present = 1;
  const Quaternion att = attitude_reference_->attitude();
  my_att.w = att.w();
  my_att.x = att.x();
  my_att.y = att.y();
  my_att.z = att.z();
  const Point3D rate_dps = (180.0f / static_cast<float>(M_PI)) *
                           attitude_reference_->rate_rps();
  my_att.x_dps = rate_dps.x();
  my_att.y_dps = rate_dps.y();
  my_att.z_dps = rate_dps.z();
  const Point3D a_mps2 = attitude_reference_->acceleration_mps2();
  my_att.a_x_mps2 = a_mps2.x();
  my_att.a_y_mps2 = a_mps2.y();
  my_att.a_z_mps2 = a_mps2.z();
  const Point3D bias_dps = (180.0f / static_cast<float>(M_PI)) *
                           attitude_reference_->bias_rps();
  my_att.bias_x_dps = bias_dps.x();
  my_att.bias_y_dps = bias_dps.y();
  my_att.bias_z_dps = bias_dps.z();
  const Eigen::Vector4f attitude_uncertainty =
      attitude_reference_->attitude_uncertainty();
  my_att.uncertainty_w = attitude_uncertainty(0);
  my_att.uncertainty_x = attitude_uncertainty(1);
  my_att.uncertainty_y = attitude_uncertainty(2);
  my_att.uncertainty_z = attitude_uncertainty(3);
  const Eigen::Vector3f bias_uncertainty_dps =
      (180.0f / static_cast<float>(M_PI)) *
      attitude_reference_->bias_uncertainty_rps();
  my_att.uncertainty_bias_x_dps = bias_uncertainty_dps.x();
  my_att.uncertainty_bias_y_dps = bias_uncertainty_dps.y();
  my_att.uncertainty_bias_z_dps = bias_uncertainty_dps.z();

  const auto end = timer_->read_us();
  my_att.update_time_10us = std::min<decltype(end)>(255, (end - start) / 10);

  // Now we need to let the ISR know about this.
  irq_.write(1);
  imu_data.active.store(true);
  auto* old_imu_data = imu_to_isr_.exchange(&imu_data);
  // If we got something, that means the ISR hadn't claimed it yet.
  // Put it back to unused.
  if (old_imu_data) {
    old_imu_data->active.store(false);
  }

  const auto this_error = attitude_reference_->error();
  if (this_error != AttitudeReference::Error::kNone) {
    my_att.last_error = static_cast<uint32_t>(this_error);
    my_att.error_count++;
    reset_estimator_.store(true);
  }
}

}
