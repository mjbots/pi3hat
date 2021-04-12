// Copyright 2020-2021 Josh Pieper, jjp@pobox.com.
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

// Operate the ICM-42688-P IMU via SPI.
class Icm42688 {
 public:
  struct Options {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    PinName irq = NC;
    PinName clkin = NC;

    uint16_t rate_hz = 800;
    uint16_t gyro_max_dps = 1000;
    uint16_t accel_max_g = 8;

    Options() {}
  };

  Icm42688(MillisecondTimer*, const Options& = Options());
  ~Icm42688();

  struct SetupData : ImuSetupData {};

  const SetupData& setup_data() const;

  using Data = ImuData;
  Data read_data();

  // This polls the interrupt digital input, so is quick.
  bool data_ready();

 private:
  class Impl;
  mjlib::micro::StaticPtr<Impl, 1024> impl_;
};

}
