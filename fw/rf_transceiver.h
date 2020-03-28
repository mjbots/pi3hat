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

#include <optional>

#include "fw/millisecond_timer.h"
#include "fw/register_spi_slave.h"
#include "fw/slot_rf_protocol.h"

namespace fw {

/// Exposes the RF transceiver through the following SPI registers.
///  48: Protocol version:
///   byte0: 0x10
///  49: Read ID
///   byte0-3: ID
///  50: Write ID
///   byte0-3: ID
///  52: Write slot data
///   byte0: slot number
///   byte1: slot size (<= 15)
///   byte2-: slot data
///  53: Write slot priority
///   byte0: slot number
///   byte1-5: slot priority
///  56: Read count
///   byte0-1: uint16 that increments on each received data
///  57: Slot age
///   * 15x of the following 2 byte structure
///     * byte-k+0,1: count - most recent "count" associated with this slot
///  64-79: Read slot data
///   byte0-3: uint32t age in ms
///   byte4: size
///   byte5-20: data
class RfTransceiver {
 public:
  RfTransceiver(fw::MillisecondTimer* timer)
      : timer_(timer) {
  }

  void Poll() {
    rf_->Poll();
  }

  void PollMillisecond() {
    rf_->PollMillisecond();
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    return {};
  }

  void ISR_End(uint16_t address, int bytes) {
  }

 private:
  void SetupRf() {
    rf_.emplace(timer_, [&]() {
        Nrf24l01::Pins pins;
        pins.mosi = PB_5_ALT0;
        pins.miso = PB_4_ALT0;
        pins.sck = PB_3_ALT0;
        pins.cs = PA_15;
        pins.irq = PB_7;
        pins.ce = PB_6;

        SlotRfProtocol::Options options;
        options.id = id_;
        options.ptx = false;
        options.pins = pins;

        return options;
      }());
    rf_->Start();
  }

  MillisecondTimer* const timer_;
  std::optional<SlotRfProtocol> rf_;
  uint32_t id_ = 0x3045;
};

}
