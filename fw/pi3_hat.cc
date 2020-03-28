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

#include <atomic>
#include <cstring>

#include "mbed.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/string_span.h"

#include "fw/attitude_reference.h"
#include "fw/bmi088.h"
#include "fw/can_bridge.h"
#include "fw/fdcan.h"
#include "fw/millisecond_timer.h"
#include "fw/register_spi_slave.h"
#include "fw/slot_rf_protocol.h"

namespace fw {
namespace {

class CanApplication {
 public:
  CanApplication(fw::MillisecondTimer* timer)
      : timer_(timer) {}

  void Poll() {
    bridge_.Poll();
  }

  void PollMillisecond() {
  }

  fw::MillisecondTimer* const timer_;

  fw::FDCan can1_{[]() {
      fw::FDCan::Options o;
      o.td = PB_4;
      o.rd = PB_3;
      o.slow_bitrate = 1000000;
      o.fast_bitrate = 5000000;
      o.fdcan_frame = true;
      o.bitrate_switch = true;
      o.automatic_retransmission = true;
      return o;
    }()
  };
  fw::FDCan can2_{[]() {
      fw::FDCan::Options o;
      o.td = PA_12;
      o.rd = PA_11;
      o.slow_bitrate = 1000000;
      o.fast_bitrate = 5000000;
      o.fdcan_frame = true;
      o.bitrate_switch = true;
      o.automatic_retransmission = true;
      return o;
    }()
  };

  CanBridge bridge_{timer_, &can1_, &can2_, {}, {}};
};


/// This is used on the auxiliary processor.  It exposes the following
/// functions:
///
/// A. A CAN bridge using an identical SPI mapping as the
/// CanApplication/CanBridge above.
///
/// B. An IMU with the following SPI registers:
///
///  32: Protocol version:
///     byte 0: the constant value 0x20
///  33: Raw IMU data
///     bytes: The contents of the 'ImuRegister' structure
///  34: Attitude data
///     bytes: The contents of the 'AttitudeRegister' structure
///
/// C. A spread spectrum RF transceiver with the following SPI
/// registers:
///
///  48: Protocol version
class AuxApplication {
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

  AuxApplication(mjlib::micro::Pool* pool, fw::MillisecondTimer* timer)
      : pool_(pool), timer_(timer) {
    setup_data_ = imu_.setup_data();
    // We do this here after everything has been initialized.
    next_imu_sample_ = timer_->read_us();

    SetupRf();
  }

  void Poll() {
    bridge_.Poll();
    const auto now = timer_->read_us();
    if (now - next_imu_sample_ < 0x80000000) {
      // We have gone past.
      next_imu_sample_ += us_step_;
      DoImu();
    }
    rf_->Poll();
  }

  void PollMillisecond() {
    rf_->PollMillisecond();
  }

 private:
  void SetupRf() {
    rf_.emplace(timer_, []() {
        Nrf24l01::Pins pins;
        pins.mosi = PB_5_ALT0;
        pins.miso = PB_4_ALT0;
        pins.sck = PB_3_ALT0;
        pins.cs = PA_15;
        pins.irq = PB_7;
        pins.ce = PB_6;

        SlotRfProtocol::Options options;
        options.id = 0x3045;
        options.ptx = false;
        options.pins = pins;

        return options;
      }());
    rf_->Start();
  }

  void DoImu() {
    const auto start = timer_->read_us();

    const auto unrotated_data = imu_.read_data();
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

    attitude_reference_.ProcessMeasurement(
        period_s_,
        (M_PI / 180.0f) * data.rate_dps,
        data.accel_mps2);

    AttitudeRegister& my_att = imu_data.attitude;
    my_att.present = 1;
    const Quaternion att = attitude_reference_.attitude();
    my_att.w = att.w();
    my_att.x = att.x();
    my_att.y = att.y();
    my_att.z = att.z();
    const Point3D rate_dps = (180.0f / M_PI) * attitude_reference_.rate_rps();
    my_att.x_dps = rate_dps.x();
    my_att.y_dps = rate_dps.y();
    my_att.z_dps = rate_dps.z();
    const Point3D a_mps2 = attitude_reference_.acceleration_mps2();
    my_att.a_x_mps2 = a_mps2.x();
    my_att.a_y_mps2 = a_mps2.y();
    my_att.a_z_mps2 = a_mps2.z();
    const Point3D bias_dps = (180.0f / M_PI) * attitude_reference_.bias_rps();
    my_att.bias_x_dps = bias_dps.x();
    my_att.bias_y_dps = bias_dps.y();
    my_att.bias_z_dps = bias_dps.z();
    const Eigen::Vector4f attitude_uncertainty =
        attitude_reference_.attitude_uncertainty();
    my_att.uncertainty_w = attitude_uncertainty(0);
    my_att.uncertainty_x = attitude_uncertainty(1);
    my_att.uncertainty_y = attitude_uncertainty(2);
    my_att.uncertainty_z = attitude_uncertainty(3);
    const Eigen::Vector3f bias_uncertainty_dps =
        (180.0f / M_PI) * attitude_reference_.bias_uncertainty_rps();
    my_att.uncertainty_bias_x_dps = bias_uncertainty_dps.x();
    my_att.uncertainty_bias_y_dps = bias_uncertainty_dps.y();
    my_att.uncertainty_bias_z_dps = bias_uncertainty_dps.z();

    const auto end = timer_->read_us();
    my_att.update_time_10us = std::min<decltype(end)>(255, (end - start) / 10);

    // Now we need to let the ISR know about this.
    imu_data.active.store(true);
    auto* old_imu_data = imu_to_isr_.exchange(&imu_data);
    // If we got something, that means the ISR hadn't claimed it yet.
    // Put it back to unused.
    if (old_imu_data) {
      old_imu_data->active.store(false);
    }
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
    if (address == 48) {
    }
    return {};
  }

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

  void ISR_End(uint16_t address, int bytes) {
  }

  mjlib::micro::Pool* const pool_;
  MillisecondTimer* const timer_;

  fw::FDCan can1_{[]() {
      fw::FDCan::Options o;
      o.td = PA_12;
      o.rd = PA_11;
      o.slow_bitrate = 125000;
      o.fast_bitrate = 125000;
      o.fdcan_frame = false;
      o.bitrate_switch = false;
      o.automatic_retransmission = true;
      return o;
    }()
  };

  CanBridge bridge_{
    timer_, &can1_, nullptr,
        [this](uint16_t address) {
      return this->ISR_Start(address);
    },
        [this](uint16_t address, int bytes) {
          this->ISR_End(address, bytes);
        }
  };

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

  std::optional<SlotRfProtocol> rf_;

  char nrf_registers_[32] = {};
};

void SetupClock() {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 170 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  //  85 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  //  85 MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    mbed_die();
  }

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      mbed_die();
    }
  }
}

}
}

int main(void) {
  DigitalIn id_select1(PC_14);
  DigitalIn id_select2(PC_15);

  fw::SetupClock();

  mjlib::micro::SizedPool<16384> pool;
  fw::MillisecondTimer timer;

  auto run = [&](auto& app) {
    uint32_t last_ms = 0;
    while (true) {
      app.Poll();
      const auto now = timer.read_ms();
      if (now != last_ms) {
        app.PollMillisecond();
        last_ms = now;
      }
    }
  };

  // See what thing we're going to do.
  if (id_select1.read() == 1) {
    // We are the AUX processor.
    fw::AuxApplication application{&pool, &timer};

    run(application);
  } else {
    fw::CanApplication application{&timer};

    run(application);
  }
}

extern "C" {
void abort() {
  mbed_die();
}

void mbed_die(void) {
  // Flash an LED that exists.
  gpio_t led;
  gpio_init_out(&led, PF_0);

  for (;;) {
    gpio_write(&led, 0);
    wait_ms(200);
    gpio_write(&led, 1);
    wait_ms(200);
  }
}
}
