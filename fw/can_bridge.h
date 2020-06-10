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

#include "fw/fdcan.h"
#include "fw/millisecond_timer.h"
#include "fw/register_spi_slave.h"

#pragma once

namespace fw {

/// This presents the following SPI interface.
///
/// Each "register" points to a discontiguous block of data that
/// cannot be read or written piecemeal.
///
/// 0: Protocol version
///    byte 0: the constant value 2
/// 1: Interface type
///    byte 0: the constant value 1
/// 2: Receive status
///    byte 0-6 - Size of up to 6 received frames.  0 means no frame
///      is available.  this size includes the header, so is the total
///      number of bytes that must be read from register 3 to get a
///      complete frame
/// 3: Received frame: reading this register returns exactly one frame
///    from the received queue.
///   byte 0 (0x00 means no data)
///     bit 7 - CAN bus
///     bits 6-0 - size of payload + 1 (1-65)
///   byte 1,2,3,4 - CAN ID, MSB first
///   byte 5+ payload
/// 4: Send a CAN frame w/ 4 byte ID
///   byte 0
///     bit 7 - CAN bus
///     bit 6-0 - size of payload
///   byte 1,2,3,4 - CAN ID, MSB first
///   byte 5+ payload
/// 5: Send a CAN frame w/ 2 byte ID
///   byte 0
///     bit 7 - CAN bus
///     byte 6-0 - size of payload
///   byte 1, 2 - CAN ID
///   byte 3+ payload

class CanBridge {
 public:
  static constexpr int kMaxSpiFrameSize = 70;
  static constexpr int kBufferItems = 6;

  struct Pins {
    PinName irq_name = NC;
  };

  CanBridge(fw::MillisecondTimer* timer,
            fw::FDCan* can1, fw::FDCan* can2,
            const Pins& pins)
      : timer_{timer},
        pins_{pins},
        can1_{can1},
        can2_{can2},
        irq_{pins.irq_name, 0} {}

  void Poll() {
    auto can_poll = [&](int bus_id, auto* can) {
      const bool rx = can->Poll(&can_header_, rx_buffer_);
      if (!rx) {
        return;
      }

      auto* this_buf = [&]() -> CanReceiveBuf* {
        for (auto& buf : can_buf_) {
          if (!buf.active) { return &buf; }
        }
        return nullptr;
      }();

      if (this_buf == nullptr) {
        // Record an error.
        return;
      }

      this_buf->active = true;

      const int size = ParseDlc(can_header_.DataLength);

      this_buf->data[0] = ((bus_id == 2) ? 0x80 : 0x00) | (size + 1);
      const auto id = can_header_.Identifier;
      this_buf->data[1] = (id >> 24) & 0xff;
      this_buf->data[2] = (id >> 16) & 0xff;
      this_buf->data[3] = (id >> 8) & 0xff;
      this_buf->data[4] = (id >> 0) & 0xff;
      this_buf->size = size;
      std::memcpy(&this_buf->data[5], rx_buffer_, size);

      // Insert this item into the receive queue.
      __disable_irq();
      [&]() {
        for (auto& item : can_rx_queue_) {
          if (item == nullptr) {
            item = this_buf;
            return;
          }
        }
        // Hmm, we couldn't.  Just throw it away.
        this_buf->active = false;
      }();
      irq_.write(1);
      __enable_irq();
    };

    if (can1_) {
      can_poll(1, can1_);
    }
    if (can2_) {
      can_poll(2, can2_);
    }

    // Look to see if we have anything to send.
    for (auto& spi_buf : spi_buf_) {
      if (spi_buf.ready_to_send) {
        // Let's send this out.
        const auto send_result = SendCan(&spi_buf);
        if (send_result == fw::FDCan::kNoSpace) {
          // Just finish this poll and try again later.
          return;
        }

        spi_buf.ready_to_send = false;
        spi_buf.active = false;
      }
    }
  }

  void PollMillisecond() {
  }

  uint8_t queue_size() const {
    return can_rx_queue_[0] ? 1 : 0;
  }

  static bool IsSpiAddress(uint16_t address) {
    return address <= 5;
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 0) {
      return {
        std::string_view("\x02", 1),
        {},
      };
    }
    if (address == 1) {
      return {
        std::string_view("\x01", 1),
        {},
      };
    }
    if (address == 2) {
      for (size_t i = 0; i < kBufferItems; i++) {
        const auto item = can_rx_queue_[i];
        address16_buf_[i] = (item == nullptr) ? 0 : (item->size + 5);
      }
      return {
        address16_buf_,
        {},
      };
    }
    if (address == 3) {
      if (!can_rx_queue_[0]) {
        current_can_buf_ = nullptr;
        return { {}, {} };
      }
      current_can_buf_ = can_rx_queue_[0];

      // Shift everything over.
      for (size_t i = 1; i < kBufferItems; i++) {
        can_rx_queue_[i - 1] = can_rx_queue_[i];
      }
      can_rx_queue_[kBufferItems - 1] = nullptr;
      if (can_rx_queue_[0] == nullptr) {
        irq_.write(0);
      }

      return {
        std::string_view(current_can_buf_->data, current_can_buf_->size + 5),
        {},
      };
    }
    if (address == 4 || address == 5) {
      current_spi_buf_ = [&]() -> SpiReceiveBuf* {
        for (auto& buf : spi_buf_) {
          if (!buf.active) {
            buf.address = address;
            return &buf;
          }
        }
        // All our buffers are full.  There must be something wrong on the CAN bus.
        //
        // TODO: Log an error.
        return nullptr;
      }();

      if (!current_spi_buf_) {
        return { {}, {} };
      } else {
        current_spi_buf_->active = true;

        return {
          {},
          mjlib::base::string_span(current_spi_buf_->data, kMaxSpiFrameSize),
        };
      }
    }

    return { {}, {} };
  }

  void ISR_End(uint16_t address, int bytes) {
    if (current_can_buf_) {
      current_can_buf_->active = false;
      current_can_buf_ = nullptr;
    }

    if (current_spi_buf_) {
      current_spi_buf_->size = bytes;
      current_spi_buf_->ready_to_send = true;
      current_spi_buf_ = nullptr;
    }
  }

 private:
  struct SpiReceiveBuf {
    char data[kMaxSpiFrameSize] = {};
    int size = 0;
    int address = 0;
    std::atomic<bool> active{false};
    std::atomic<bool> ready_to_send{false};
  };

  struct CanReceiveBuf {
    char data[kMaxSpiFrameSize] = {};
    int size = 0;
    std::atomic<bool> active{false};
  };

  fw::FDCan::SendResult SendCan(const SpiReceiveBuf* spi) {
    const int min_size = (spi->address == 4) ? 5 : 3;
    if (spi->size < min_size) {
      // This is not big enough for a header.
      // TODO record an error.
      return fw::FDCan::kSuccess;
    }
    const int can_bus = (spi->data[0] & 0x80) ? 1 : 0;
    const int size = (spi->data[0] & 0x7f);
    uint32_t id = 0;
    if (spi->address == 4) {
      // 4 byte ID
      id = (u8(spi->data[1]) << 24) |
          (u8(spi->data[2]) << 16) |
          (u8(spi->data[3]) << 8) |
          (u8(spi->data[4]));
    } else {
      // 2 byte ID
      id = (u8(spi->data[1]) << 8) |
          (u8(spi->data[2]));
    }
    if (size + min_size > spi->size) {
      // There is not enough data.  TODO Record an error.
      return fw::FDCan::kSuccess;
    }

    auto* const can = (can_bus == 0) ?
        can1_ :
        can2_;

    if (can) {
      return can->Send(id, std::string_view(&spi->data[min_size], size));
    } else {
      return fw::FDCan::kSuccess;
    }
  }

  static int ParseDlc(uint32_t dlc_code) {
    if (dlc_code == FDCAN_DLC_BYTES_0) { return 0; }
    if (dlc_code == FDCAN_DLC_BYTES_1) { return 1; }
    if (dlc_code == FDCAN_DLC_BYTES_2) { return 2; }
    if (dlc_code == FDCAN_DLC_BYTES_3) { return 3; }
    if (dlc_code == FDCAN_DLC_BYTES_4) { return 4; }
    if (dlc_code == FDCAN_DLC_BYTES_5) { return 5; }
    if (dlc_code == FDCAN_DLC_BYTES_6) { return 6; }
    if (dlc_code == FDCAN_DLC_BYTES_7) { return 7; }
    if (dlc_code == FDCAN_DLC_BYTES_8) { return 8; }
    if (dlc_code == FDCAN_DLC_BYTES_12) { return 12; }
    if (dlc_code == FDCAN_DLC_BYTES_16) { return 16; }
    if (dlc_code == FDCAN_DLC_BYTES_20) { return 20; }
    if (dlc_code == FDCAN_DLC_BYTES_24) { return 24; }
    if (dlc_code == FDCAN_DLC_BYTES_32) { return 32; }
    if (dlc_code == FDCAN_DLC_BYTES_48) { return 48; }
    if (dlc_code == FDCAN_DLC_BYTES_64) { return 64; }
    mbed_die();
    return 0;
  }

  template <typename T>
  static uint8_t u8(T value) {
    return static_cast<uint8_t>(value);
  }

  fw::MillisecondTimer* const timer_;
  const Pins pins_;

  fw::FDCan* can1_ = nullptr;
  fw::FDCan* can2_ = nullptr;

  DigitalOut irq_;

  SpiReceiveBuf spi_buf_[kBufferItems] = {};
  SpiReceiveBuf* current_spi_buf_ = nullptr;

  CanReceiveBuf can_buf_[kBufferItems] = {};
  CanReceiveBuf* can_rx_queue_[kBufferItems] = {};
  CanReceiveBuf* current_can_buf_ = nullptr;

  FDCAN_RxHeaderTypeDef can_header_ = {};
  char rx_buffer_[64] = {};

  char address16_buf_[kBufferItems] = {};
};


}
