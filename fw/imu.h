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

#include <atomic>
#include <cstdint>
#include <string_view>

#include "mjlib/micro/pool_ptr.h"

#include "fw/attitude_reference.h"
#include "fw/bmi088.h"
#include "fw/millisecond_timer.h"
#include "fw/quaternion.h"
#include "fw/register_spi_slave.h"

namespace fw {

/// Exposes an IMU through the following SPI registers.
///
///  32: Protocol version:
///     byte 0: the constant value 0x20
///  33: Raw IMU data
///     bytes: The contents of the 'ImuRegister' structure
///  34: Attitude data
///     bytes: The contents of the 'AttitudeRegister' structure
class Imu {
 public:
  struct ImuRegister {
    uint16_t present = 0;
    float gx_dps = 0;
    float gy_dps = 0;
    float gz_dps = 0;
    float ax_mps2 = 0;
    float ay_mps2 = 0;
    float az_mps2 = 0;

    ImuRegister() {}
    ImuRegister(const Bmi088::Data& data) {
      present = 1;
      gx_dps = data.rate_dps.x();
      gy_dps = data.rate_dps.y();
      gz_dps = data.rate_dps.z();
      ax_mps2 = data.accel_mps2.x();
      ay_mps2 = data.accel_mps2.y();
      az_mps2 = data.accel_mps2.z();
    }
  } __attribute__((packed));

  struct AttitudeRegister {
    uint8_t present = 0;
    uint8_t update_time_10us = 0;
    float w = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    float x_dps = 0;
    float y_dps = 0;
    float z_dps = 0;
    float a_x_mps2 = 0;
    float a_y_mps2 = 0;
    float a_z_mps2 = 0;
    float bias_x_dps = 0;
    float bias_y_dps = 0;
    float bias_z_dps = 0;
    float uncertainty_w = 0;
    float uncertainty_x = 0;
    float uncertainty_y = 0;
    float uncertainty_z = 0;
    float uncertainty_bias_x_dps = 0;
    float uncertainty_bias_y_dps = 0;
    float uncertainty_bias_z_dps = 0;
  } __attribute__((packed));

  Imu(mjlib::micro::Pool* pool, fw::MillisecondTimer* timer)
      : pool_(pool),
        timer_(timer) {
    // We do this here after everything has been initialized.
    next_imu_sample_ = timer_->read_us();

    setup_data_ = imu_.setup_data();
  }

  void Poll() {
    const auto now = timer_->read_us();
    if (now - next_imu_sample_ < 0x80000000) {
      // We have gone past.
      next_imu_sample_ += us_step_;
      DoImu();
    }
  }

  void PollMillisecond() {}

  bool data_present() const {
    return imu_to_isr_.load() != nullptr;
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 32) {
      return {
        std::string_view("\x20", 1),
        {},
      };
    }
    if (address == 33) {
      const int bit = 1;
      if (imu_isr_bitmask_ & bit) {
        ISR_GetImuData();
      }
      imu_isr_bitmask_ |= bit;
      if (imu_in_isr_) {
        return {
          std::string_view(reinterpret_cast<const char*>(&imu_in_isr_->imu),
                           sizeof(imu_in_isr_->imu)),
          {},
        };
      } else {
        return {{}, {}};
      }
    }
    if (address == 34) {
      const int bit = 2;
      if (imu_isr_bitmask_ & bit) {
        ISR_GetImuData();
      }
      imu_isr_bitmask_ |= bit;
      if (imu_in_isr_) {
        return {
          std::string_view(
              reinterpret_cast<const char*>(&imu_in_isr_->attitude),
              sizeof(imu_in_isr_->attitude)),
          {},
        };
      } else {
        return {{}, {}};
      }
    }
    return {};
  }

  void ISR_End(uint16_t address, int bytes) {
  }

 private:
  void ISR_GetImuData() {
    // If we have something, release it.
    if (imu_in_isr_) {
      imu_in_isr_->active.store(false);
    }
    imu_in_isr_ = nullptr;
    // Now try to get the next value.
    imu_in_isr_ = imu_to_isr_.exchange(nullptr);

    imu_isr_bitmask_ = 0;
  }

  void DoImu();

  mjlib::micro::Pool* const pool_;
  MillisecondTimer* const timer_;

  Bmi088 imu_{pool_, timer_, []() {
      Bmi088::Options options;
      options.mosi = PB_15;
      options.miso = PB_14;
      options.sck = PB_13;
      options.acc_cs = PA_9;
      options.gyro_cs = PB_12;
      options.acc_int = PA_10;
      options.gyro_int = PA_8;

      options.rate_hz = 400;
      options.gyro_max_dps = 1000;
      options.accel_max_g = 6;

      return options;
    }()
  };
  const float period_s_ = 1.0f / static_cast<float>(imu_.setup_data().rate_hz);
  const uint32_t us_step_ = 1000000 / imu_.setup_data().rate_hz;
  uint32_t next_imu_sample_ = 0;

  Quaternion mounting_ = Quaternion::FromEuler(0, M_PI_2, -M_PI_2);

  struct ImuData {
    ImuRegister imu;
    AttitudeRegister attitude;
    std::atomic<bool> active{false};
  };

  // We need one here for the ISR to work from, one to be queued up
  // for it to use next, and one for the main routine to fill in to
  // replace that.
  ImuData imu_data_buffer_[3] = {};
  // This contains the value that we want to give to the ISR.
  std::atomic<ImuData*> imu_to_isr_{nullptr};

  // And this contains the value the ISR is currently using.
  ImuData* imu_in_isr_{nullptr};
  // Used to keep track of when we need to grab new data in the ISR.
  uint32_t imu_isr_bitmask_ = 3;

  Bmi088::SetupData setup_data_;
  AttitudeReference attitude_reference_;
};

}
