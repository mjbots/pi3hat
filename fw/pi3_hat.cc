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
#include "fw/imu.h"
#include "fw/microphone.h"
#include "fw/millisecond_timer.h"
#include "fw/register_spi_slave.h"
#include "fw/rf_transceiver.h"
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

  CanBridge bridge_{timer_, &can1_, &can2_, PB_0, {}, {}};
};


/// This is used on the auxiliary processor.  It exposes the following
/// functions:
///
/// A. A CAN bridge using an identical SPI mapping as the
/// CanApplication/CanBridge.
///
/// B. An IMU with the SPI mapping as per the Imu class.
///
/// C. A spread spectrum RF transceiver as per TODO.
///
/// D. A microphone.
class AuxApplication {
 public:
  AuxApplication(mjlib::micro::Pool* pool, fw::MillisecondTimer* timer)
      : pool_(pool), timer_(timer) {
  }

  void Poll() {
    bridge_.Poll();
    imu_.Poll();
    rf_.Poll();
  }

  void PollMillisecond() {
    imu_.PollMillisecond();
    rf_.PollMillisecond();
    microphone_.PollMillisecond();
  }

 private:
  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address >=32 && address < 48) {
      return imu_.ISR_Start(address);
    }
    if (address >= 48 && address < 80) {
      return rf_.ISR_Start(address);
    }
    if (address >= 80 && address < 96) {
      return microphone_.ISR_Start(address);
    }
    if (address == 96) {
      // Until we have IRQs, this will be a multiplexing register
      // which allows clients to determine what is ready to read.
      read_buf_[0] = bridge_.queue_size();
      read_buf_[1] = imu_.data_present() ? 1 : 0;
      const auto bitfield = rf_.bitfield();
      std::memcpy(&read_buf_[2], &bitfield, sizeof(bitfield));
      return {
        std::string_view(&read_buf_[0], 6),
        {},
      };
    }
    return {};
  }

  void ISR_End(uint16_t address, int bytes) {
    if (address >= 32 && address < 48) {
      imu_.ISR_End(address, bytes);
    }
    if (address >= 48 && address < 80) {
      rf_.ISR_End(address, bytes);
    }
    if (address >= 80 && address < 96) {
      microphone_.ISR_End(address, bytes);
    }
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
    timer_, &can1_, nullptr, PB_0,
        [this](uint16_t address) {
          return this->ISR_Start(address);
        },
        [this](uint16_t address, int bytes) {
          this->ISR_End(address, bytes);
        }
  };

  DigitalOut can_shdn_{PC_6, 0};

  Imu imu_{pool_, timer_, PB_1};
  RfTransceiver rf_{timer_, PB_2};
  Microphone microphone_{timer_, PB_10};
  char read_buf_[6] = {};
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
