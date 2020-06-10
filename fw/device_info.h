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

#include "fw/git_info.h"

namespace fw {

/// Exposes device info through the following SPI register
///
/// 97: Device Information
///  bytes0-19: git sha256 hash
///  byte20: dirty flag
///  byte21-32: serial number
class DeviceInfo {
 public:
  DeviceInfo() {
    GitInfo git_info;
    std::memcpy(&device_info_.sha256hash[0], &git_info.hash[0],
                sizeof(git_info.hash));
    device_info_.dirty = git_info.dirty ? 1 : 0;
    std::memcpy(&device_info_.serial_number[0],
                reinterpret_cast<const char*>(UID_BASE),
                sizeof(device_info_.serial_number));
  }

  static bool IsSpiAddress(uint16_t address) {
    return address == 97;
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 97) {
      return {
        std::string_view(
            reinterpret_cast<const char*>(&device_info_),
            sizeof(device_info_)),
        {}
      };
    }

    return {{}, {}};
  }

  void ISR_End(uint16_t address, int bytes) {
  }

 private:
  struct DeviceInfoData {
    uint8_t sha256hash[20] = {};
    uint8_t dirty = 0;
    uint8_t serial_number[12] = {};
  } __attribute__((packed));

  DeviceInfoData device_info_;
};

}
