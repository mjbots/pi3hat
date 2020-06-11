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

#include <string_view>

#include "fw/register_spi_slave.h"

namespace fw {

class CpuMeter {
 public:
  void Poll() {
    cycles_since_ms_++;
  }

  void PollMillisecond() {
    ms_cycle_count_[ms_cycle_count_pos_] = cycles_since_ms_;
    cycles_since_ms_ = 0;
    ms_cycle_count_pos_ = (ms_cycle_count_pos_ + 1) % 20;

    result_.min_cycle_per_ms = ms_cycle_count_[0];
    uint32_t total = 0;
    for (auto value : ms_cycle_count_) {
      total += value;
      if (value < result_.min_cycle_per_ms) {
        result_.min_cycle_per_ms = value;
      }
    }
    result_.average_cycle_per_ms = total / 20;
  }

  static bool IsSpiAddress(uint16_t address) {
    return address == 100;
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 100) {
      // This is a race condition.  I don't really care.
      return {
        std::string_view(
            reinterpret_cast<const char*>(&result_),
            sizeof(result_)),
        {}
      };
    }

    return {{}, {}};
  }

  void ISR_End(uint16_t address, int bytes) {
  }

 private:
  uint32_t cycles_since_ms_ = 0;
  uint32_t ms_cycle_count_[20] = {};
  uint8_t ms_cycle_count_pos_ = 0;

  struct Result {
    uint32_t average_cycle_per_ms = 0;
    uint32_t min_cycle_per_ms = 0;
  } __attribute__((packed));

  Result result_;
};

}
