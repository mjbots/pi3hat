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

#pragma once

#include "PinNames.h"

#include "mjlib/micro/pool_ptr.h"

#include "fw/quaternion.h"

namespace fw {

// Operate the BMI088 IMU via SPI.
class Bmi088 {
 public:
  struct Config {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName imu_cs = NC;
    PinName gyro_cs = NC;
    PinName imu_int = NC;
    PinName gyro_int = NC;

    uint16_t rate_hz = 1000;
    uint16_t gyro_max_dps = 1000;
    uint16_t accel_max_g = 6;
    Quaternion offset;

    Config() {}
  };

  Bmi088(mjlib::micro::Pool*, const Config& = Config());
  ~Bmi088();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
