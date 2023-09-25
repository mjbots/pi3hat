// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <Eigen/Core>

#include "PinNames.h"

#include "mjlib/micro/static_ptr.h"

#include "fw/imu_data.h"
#include "fw/millisecond_timer.h"

namespace fw {

// Operate the BMI088 IMU via SPI.
class Bmi088 {
 public:
  struct Options {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName acc_cs = NC;
    PinName gyro_cs = NC;
    PinName acc_int = NC;
    PinName gyro_int = NC;

    uint16_t rate_hz = 800;
    uint16_t gyro_max_dps = 1000;
    uint16_t accel_max_g = 6;

    Options() {}
  };

  Bmi088(MillisecondTimer*, const Options& = Options());
  ~Bmi088();

  struct SetupData : ImuSetupData {
    uint8_t gyro_id = 0;
  };

  const SetupData& setup_data() const;

  using Data = ImuData;

  Data read_data();

 private:
  class Impl;
  mjlib::micro::StaticPtr<Impl, 1024> impl_;
};

}
