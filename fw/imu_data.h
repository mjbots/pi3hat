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

namespace fw {
struct ImuData {
  Eigen::Vector3f rate_dps;
  Eigen::Vector3f accel_mps2;
};

struct ImuSetupData {
  int error = 0;
  uint8_t whoami = 0;
  uint16_t rate_hz = 0;
  uint16_t gyro_max_dps = 0;
  uint16_t accel_max_g = 0;
};

}
