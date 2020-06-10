// Copyright 2014-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#pragma once

#include "fw/euler.h"
#include "fw/point3d.h"
#include "fw/quaternion.h"
#include "fw/ukf_filter.h"

namespace fw {

/// Filters IMU readings to produce an estimate of attitude, rates,
/// and accelerations.
///
/// The following frame conventions are used:
///  +x is positive forward
///  +y is positive right
///  +z is positive down
///
/// Accelerometer measurements are assumed such that at rest, +1g is
/// measured in the UP (-z) direction.
class AttitudeReference {
 public:
  // Note: This is not ideal, as it treats the quaternion as 4
  // independent states when in fact there are only 3 degrees of
  // freedom.
  enum {
    kNumStates = 7,
  };

  using Filter = UkfFilter<float, kNumStates>;
  using Error = Filter::Error;

  struct Options {
    float process_noise_gyro_rps = (5e-4f * 5e-4f);
    float process_noise_bias_rps = (7e-5f * 7e-5f);
    float measurement_noise_accel = (0.5f * 0.5f);
    float initial_noise_attitude = (1.0f * 1.0f);
    float initial_noise_bias_rps = (0.1f * 0.1f);
    float accelerometer_reject_mps2 = 2.0f;
    float zero_yaw_noise_rps = (10.0f * 10.0f);

    Options() {}
  };

  AttitudeReference(const Options& options = Options())
      : options_(options) ,
        ukf_{
    (Filter::State() <<
     1.0f, 0.0f, 0.0f, 0.0f,
     0.0f, 0.0f, 0.0f).finished(),
        Eigen::DiagonalMatrix<float, 7, 7>(
            (Filter::State() <<
             options.initial_noise_attitude,
             options.initial_noise_attitude,
             options.initial_noise_attitude,
             options.initial_noise_attitude,
             options.initial_noise_bias_rps,
             options.initial_noise_bias_rps,
             options.initial_noise_bias_rps).finished()),
        Eigen::DiagonalMatrix<float, 7, 7>(
            (Filter::State() <<
             options.process_noise_gyro_rps,
             options.process_noise_gyro_rps,
             options.process_noise_gyro_rps,
             options.process_noise_gyro_rps,
             options.process_noise_bias_rps,
             options.process_noise_bias_rps,
             options.process_noise_bias_rps).finished())} {
  }

  Filter::Error error() const { return ukf_.error(); }

  void ProcessMeasurement(float delta_s,
                          const Point3D& rate_B_rps,
                          const Point3D& accel_mps2) {
    current_gyro_rps_ = rate_B_rps;
    current_accel_mps2_ = accel_mps2;

    const Point3D norm_g = accel_mps2.normalized();

    if (!initialized_) {
      initialized_ = true;
      const Quaternion start = AccelToOrientation(norm_g);
      ukf_.state()(0) = start.w();
      ukf_.state()(1) = start.x();
      ukf_.state()(2) = start.y();
      ukf_.state()(3) = start.z();
    }

    ukf_.UpdateState(
        delta_s,
        [this](const auto& _1, const auto& _2) {
          return this->ProcessFunction(_1, _2);
        });

    const float accel_norm_mps2 = accel_mps2.norm();
    const float accel_err_mps2 = std::abs(accel_norm_mps2 - 9.81f);

    // Only accept the acceleration measurement if we're somewhat
    // close to 1g.  Otherwise, we are likely in some dynamic maneuver
    // and it won't add much value.
    if (accel_err_mps2 < options_.accelerometer_reject_mps2) {
      ukf_.UpdateMeasurement(
          &MeasureAccel,
          norm_g,
          (Eigen::DiagonalMatrix<float, 3, 3>(
              (Eigen::Vector3f() <<
               options_.measurement_noise_accel,
               options_.measurement_noise_accel,
               options_.measurement_noise_accel).finished())));
    }

    // We have no observability into the yaw measurement.  Lets
    // artificially constrain it to return to 0 velocity there just so
    // things don't grow without bound.
    ukf_.UpdateMeasurement(
        [this](const auto& _1) { return this->MeasureYawRate(_1); },
        Eigen::Matrix<float, 1, 1>(0.f),
        (Eigen::Matrix<float, 1, 1>(options_.zero_yaw_noise_rps)));
  }

  Quaternion attitude() const {
    return Quaternion(ukf_.state()(0),
                      ukf_.state()(1),
                      ukf_.state()(2),
                      ukf_.state()(3));
  }

  Point3D rate_rps() const {
    return current_gyro_rps_ + bias_rps();
  }

  Point3D acceleration_mps2() const {
    return current_accel_mps2_ - 9.81f * OrientationToAccel(attitude());
  }

  Point3D bias_rps() const {
    return ukf_.state().tail(3);
  }

  Eigen::Vector4f attitude_uncertainty() const {
    return ukf_.covariance().diagonal().head(4).cwiseSqrt();
  }

  Eigen::Vector3f bias_uncertainty_rps() const {
    return ukf_.covariance().diagonal().tail(3).cwiseSqrt();
  }

 private:
  Filter::State ProcessFunction(
      const Filter::State& state, float dt_s) const {
    const Quaternion this_attitude = Quaternion(
        state(0), state(1), state(2), state(3)).normalized();

    Quaternion delta = Quaternion::IntegrateRotationRate(
        current_gyro_rps_ + Point3D(state(4), state(5), state(6)),
        dt_s);

    Quaternion next_attitude = (this_attitude * delta).normalized();

    return (Filter::State() <<
            next_attitude.w(),
            next_attitude.x(),
            next_attitude.y(),
            next_attitude.z(),
            state(4),
            state(5),
            state(6)).finished();
  }

  static Eigen::Matrix<float, 3, 1> OrientationToAccel(
      const Quaternion& attitude) {
    const Point3D gravity(0.f, 0.f, -1.f);
    return attitude.conjugated().Rotate(gravity);
  }

  static Eigen::Matrix<float, 3, 1> MeasureAccel(const Filter::State& s) {
    return OrientationToAccel(
        Quaternion(s(0), s(1), s(2), s(3)).normalized());
  }

  Eigen::Matrix<float, 1, 1> MeasureYawRate(const Filter::State& s) const {
    const Eigen::Vector3f rate_rps = current_gyro_rps_ + s.tail(3);
    const Quaternion attitude(s(0), s(1), s(2), s(3));
    return Eigen::Matrix<float, 1, 1>(attitude.conjugated().Rotate(rate_rps).z());
  }

  static Quaternion AccelToOrientation(const Point3D& n_inv) {
    const Point3D n = -1.0 * n_inv;
    Euler euler_rad;
    euler_rad.roll = std::atan2(-n.x(), n.z());
    euler_rad.pitch = std::atan2(n.y(), std::sqrt(n.x() * n.x() +
                                                  n.z() * n.z()));

    return Quaternion::FromEuler(euler_rad);
  }

  const Options options_;
  Filter ukf_;
  bool initialized_ = false;
  Point3D current_gyro_rps_;
  Point3D current_accel_mps2_;
};

}
