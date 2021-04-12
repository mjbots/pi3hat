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
#include "fw/icm42688.h"
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
///  35: Read configuration
///     bytes: The contents of the 'Configuration' structure
///  36: Write configuration
///     bytes: The contents of the 'Configuration' structure
///  37: Error status
///     byte 0-3: A bitmask of error flags
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
    ImuRegister(const ImuData& data) {
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
    uint32_t error_count = 0;
    uint32_t last_error = 0;
  } __attribute__((packed));

  struct Configuration {
    float roll_deg = 0;
    float pitch_deg = 0;
    float yaw_deg = 0;
    uint32_t rate_hz = 400;

    Quaternion quaternion() {
      return Quaternion::FromEuler(
          Radians(roll_deg),
          Radians(pitch_deg),
          Radians(yaw_deg));
    }

    bool operator==(const Configuration& rhs) const {
      return yaw_deg == rhs.yaw_deg &&
          pitch_deg == rhs.pitch_deg &&
          roll_deg == rhs.roll_deg &&
          rate_hz == rhs.rate_hz;
    }

    bool operator!=(const Configuration& rhs) const {
      return !(*this == rhs);
    }
  } __attribute__((packed));

  Imu(mjlib::micro::Pool* pool, fw::MillisecondTimer* timer, PinName irq_name)
      : pool_(pool),
        timer_(timer),
        irq_(irq_name, 0) {
    // Detect if we have a BMI088/BMI090L or the ICM-42688-P
    ConfigureBmi088();
    if (bmi088_->setup_data().error) {
      bmi088_.reset();
      ConfigureIcm42688();
      if (icm42688_->setup_data().error) {
        // No IMU found!
        error_flags_ |= 1;
        return;
      }
    }
    attitude_reference_.emplace();

    // We do this here after everything has been initialized.
    next_imu_sample_ = timer_->read_us();

    setup_data_ = bmi088_->setup_data();
  }

  void Poll() {
    if (icm42688_) {
      // For the ICM-42688-P, we rely on its interrupt to tell us that
      // we should read data.
      if (icm42688_->data_ready()) {
        DoImu();
      }
    } else {
      // Otherwise we do it at a fixed interval.
      const auto now = timer_->read_us();
      if (now - next_imu_sample_ < 0x80000000) {
        // We have gone past.
        next_imu_sample_ += us_step_;
        DoImu();
      }
    }
  }

  void PollMillisecond() {}

  bool data_present() const {
    return imu_to_isr_.load() != nullptr;
  }

  static bool IsSpiAddress(uint16_t address) {
    return address >= 32 && address <= 37;
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
    if (address == 35) {
      return {
        std::string_view(
            reinterpret_cast<const char*>(&spi_config_),
            sizeof(spi_config_)),
        {},
      };
    }
    if (address == 36) {
      return {
        {},
        mjlib::base::string_span(
            reinterpret_cast<char*>(&spi_config_shadow_),
            sizeof(spi_config_shadow_)),
      };
    }
    if (address == 37) {
      return {
        {},
        mjlib::base::string_span(
            reinterpret_cast<char*>(&error_flags_),
            sizeof(error_flags_)),
      };
    }
    return {};
  }

  void ISR_End(uint16_t address, int bytes) {
    if (address == 36 &&
        bytes == sizeof(spi_config_shadow_)) {
      if (spi_config_ != spi_config_shadow_) {
        spi_config_ = spi_config_shadow_;
        // This is a data race, but we're about to throw away our entire
        // estimator, so who cares?
        mounting_ = spi_config_.quaternion();
        reset_estimator_.store(true);
      }
    }
  }

 private:
  void ConfigureBmi088() {
    bmi088_.emplace(timer_, [&]() {
        Bmi088::Options options;
        options.mosi = PB_15;
        options.miso = PB_14;
        options.sck = PB_13;
        options.acc_cs = PA_9;
        options.gyro_cs = PB_12;
        options.acc_int = PA_10;
        options.gyro_int = PA_8;

        options.rate_hz = spi_config_.rate_hz;
        options.gyro_max_dps = 1000;
        options.accel_max_g = 6;

        return options;
      }());
    period_s_ = 1.0f / static_cast<float>(bmi088_->setup_data().rate_hz);
    us_step_ = 1000000 / bmi088_->setup_data().rate_hz;
  };

  void ConfigureIcm42688() {
    icm42688_.emplace(timer_, [&]() {
        Icm42688::Options options;
        options.mosi = PB_15;
        options.miso = PB_14;
        options.sck = PB_13;
        options.cs = PB_11;
        options.clkin = PB_10;
        options.irq = PA_8;

        options.rate_hz = spi_config_.rate_hz;
        options.gyro_max_dps = 1000;
        options.accel_max_g = 8;

        return options;
      }());
    period_s_ = 1.0f / static_cast<float>(icm42688_->setup_data().rate_hz);
    us_step_ = 1000000 / icm42688_->setup_data().rate_hz;
  }

  void ISR_GetImuData() {
    // If we have something, release it.
    if (imu_in_isr_) {
      imu_in_isr_->active.store(false);
    }
    imu_in_isr_ = nullptr;
    // Now try to get the next value.
    imu_in_isr_ = imu_to_isr_.exchange(nullptr);
    irq_.write(0);

    imu_isr_bitmask_ = 0;
  }

  void DoImu();

  mjlib::micro::Pool* const pool_;
  MillisecondTimer* const timer_;
  DigitalOut irq_;

  std::optional<Bmi088> bmi088_;
  std::optional<Icm42688> icm42688_;
  float period_s_ = 0.0;
  uint32_t us_step_ = 0;
  uint32_t next_imu_sample_ = 0;

  Configuration spi_config_{0, 90, -90, 400};
  Configuration spi_config_shadow_;
  Quaternion mounting_ = spi_config_.quaternion();
  uint32_t error_flags_ = 0;

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
  std::optional<AttitudeReference> attitude_reference_;

  std::atomic<bool> reset_estimator_{false};
};

}
