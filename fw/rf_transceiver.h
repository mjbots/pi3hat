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
///  51: Write slot data
///   byte0: slot number
///   byte1-4: priority (little endian)
///   byte5+: slot data
///  52: Read status
///   byte0-3: uint32 with 2 bits per field
///   byte4-7: uint32 ms since last lock from tx
///  64-79: Read slot data
///   byte0-3: uint32 age in ms (little endian)
///   byte4: size
///   byte5-20: data
class RfTransceiver {
 public:
  static inline constexpr int kTimeoutMs = 1000;

  RfTransceiver(fw::MillisecondTimer* timer, PinName irq_name)
      : timer_(timer),
        irq_(irq_name) {
    SetupRf();
  }

  void Poll() {
    rf_->Poll();
    const auto this_bitfield = rf_->remote()->slot_bitfield();
    if (this_bitfield != last_bitfield_) {
      irq_.write(1);
      last_bitfield_ = this_bitfield;
    }
  }

  void PollMillisecond() {
    if (reset_.load()) {
      reset_.store(false);
      SetupRf();
    }

    auto old_timeout = remaining_timeout_ms_.load();
    if (old_timeout == 0) {
      // do nothing
    } else {
      if (old_timeout == 1) {
        Disable();
      }
      const auto new_timeout = old_timeout - 1;
      remaining_timeout_ms_.compare_exchange_strong(old_timeout, new_timeout);
    }
    rf_->PollMillisecond();

    if (rf_->locked()) {
      lock_age_ms_ = 0;
    } else {
      if (lock_age_ms_ != std::numeric_limits<uint32_t>::max()) {
        lock_age_ms_++;
      }
    }
  }

  uint32_t bitfield() {
    irq_.write(0);
    return rf_->remote()->slot_bitfield();
  }

  static bool IsSpiAddress(uint16_t address) {
    return address >= 48 && address <= 79;
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 48) {
      return {
        std::string_view("\x10", 1),
        {},
      };
    }
    if (address == 49) {
      return {
        std::string_view(reinterpret_cast<const char*>(&staged_id_),
                         sizeof(staged_id_)),
        {},
      };
    }
    if (address == 50) {
      return {
        {},
        mjlib::base::string_span(
            reinterpret_cast<char*>(&staged_id_), sizeof(staged_id_)),
      };
    }
    if (address == 51) {
      return {
        {},
        mjlib::base::string_span(
            reinterpret_cast<char*>(write_buf_), sizeof(write_buf_)),
      };
    }
    if (address == 52) {
      const auto result = bitfield();
      std::memcpy(&read_buf_[0], reinterpret_cast<const char*>(&result),
                  sizeof(result));
      const uint32_t lock_age_ms = lock_age_ms_.load();
      std::memcpy(&read_buf_[4], reinterpret_cast<const char*>(&lock_age_ms),
                  sizeof(lock_age_ms));
      return {
        std::string_view(&read_buf_[0], sizeof(result) + sizeof(lock_age_ms)),
        {},
      };
    }
    if (address >= 64 && address <= 79) {
      const int slot_num = address - 64;
      const auto slot = rf_->remote()->rx_slot(slot_num);
      std::memcpy(&read_buf_[0], reinterpret_cast<const char*>(&slot.age), 4);
      read_buf_[4] = slot.size;
      std::memcpy(&read_buf_[5], slot.data, slot.size);
      return {
        std::string_view(&read_buf_[0], slot.size + 5),
        {},
      };
    }
    return {};
  }

  void ISR_End(uint16_t address, int bytes) {
    if (address == 50 && bytes == sizeof(staged_id_)) {
      id_.store(staged_id_);
      reset_.store(true);
    }
    if (address == 51 && bytes > 5) {
      const auto slot_num = std::max(0, std::min<int>(15, write_buf_[0]));
      SlotRfProtocol::Slot slot;
      slot.priority =
          (write_buf_[1] << 0) |
          (write_buf_[2] << 8) |
          (write_buf_[3] << 16) |
          (write_buf_[4] << 24);
      slot.size = bytes - 5;
      std::memcpy(slot.data, &write_buf_[5], bytes - 5);
      rf_->remote()->tx_slot(slot_num, slot);
      remaining_timeout_ms_.store(kTimeoutMs);
    }
  }

 private:
  void SetupRf() {
    __disable_irq();
    rf_.emplace(timer_, [&]() {
        Nrf24l01::Pins pins;
        pins.mosi = PB_5_ALT0;
        pins.miso = PB_4_ALT0;
        pins.sck = PB_3_ALT0;
        pins.cs = PA_15;
        pins.irq = PB_7;
        pins.ce = PB_6;

        SlotRfProtocol::Options options;
        options.ids[0] = id_;
        options.ptx = false;
        options.pins = pins;

        return options;
      }());
    rf_->Start();
    __enable_irq();
  }

  void Disable() {
    for (int slot_index = 0;
         slot_index < SlotRfProtocol::kNumSlots;
         slot_index++) {
      auto slot = rf_->remote()->tx_slot(slot_index);
      slot.priority = 0;
      rf_->remote()->tx_slot(slot_index, slot);
    }
  }

  MillisecondTimer* const timer_;
  DigitalOut irq_;
  std::optional<SlotRfProtocol> rf_;
  std::atomic<uint32_t> id_{0x3045};
  uint32_t staged_id_ = id_.load();
  char write_buf_[64] = {};
  char read_buf_[24] = {};
  std::atomic<int32_t> remaining_timeout_ms_ = 0;
  std::atomic<bool> reset_{false};
  std::atomic<uint32_t> lock_age_ms_ = 0;
  uint32_t last_bitfield_ = 0;
};

}
