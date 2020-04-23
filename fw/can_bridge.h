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
///    byte 0: the constant value 1
/// 1: Interface type
///    byte 0: the constant value 1
/// 16: Receive status
///    byte 0-6 - Size of up to 6 received frames.  0 means no frame
///      is available.  this size includes the header, so is the total
///      number of bytes that must be read from register 17 to get a
///      complete frame
/// 17: Received frame: reading this register returns exactly one frame
///    from the received queue.
///   byte 0 - CAN bus (0/1/2) - 0 means this frame is not present
///   byte 1,2,3,4 - CAN ID, MSB first
///   byte 5 - size of remaining payload
///   byte 6+ payload
/// 18: Send a CAN frame
///   byte 0 - CAN bus (1/2)
///   byte 1,2,3,4 - CAN ID, MSB first
///   byte 5 - size of remaining payload
///   byte 6+ payload

class CanBridge {
 public:
  static constexpr int kMaxSpiFrameSize = 70;
  static constexpr int kBufferItems = 6;

  CanBridge(fw::MillisecondTimer* timer,
            fw::FDCan* can1, fw::FDCan* can2,
            PinName irq_name,
            RegisterSPISlave::StartHandler start_handler,
            RegisterSPISlave::EndHandler end_handler)
      : timer_{timer},
        can1_{can1},
        can2_{can2},
        irq_{irq_name, 0},
        start_handler_(start_handler),
        end_handler_(end_handler) {}

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

      this_buf->data[0] = bus_id;
      const auto id = can_header_.Identifier;
      this_buf->data[1] = (id >> 24) & 0xff;
      this_buf->data[2] = (id >> 16) & 0xff;
      this_buf->data[3] = (id >> 8) & 0xff;
      this_buf->data[4] = (id >> 0) & 0xff;
      const int size = ParseDlc(can_header_.DataLength);
      this_buf->data[5] = size;
      this_buf->size = size;
      std::memcpy(&this_buf->data[6], rx_buffer_, size);

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

  uint8_t queue_size() const {
    return can_rx_queue_[0] ? 1 : 0;
  }

 private:
  struct SpiReceiveBuf {
    char data[kMaxSpiFrameSize] = {};
    int size = 0;
    std::atomic<bool> active{false};
    std::atomic<bool> ready_to_send{false};
  };

  struct CanReceiveBuf {
    char data[kMaxSpiFrameSize] = {};
    int size = 0;
    std::atomic<bool> active{false};
  };

  fw::FDCan::SendResult SendCan(const SpiReceiveBuf* spi) {
    if (spi->size < 6) {
      // This is not big enough for a header.
      // TODO record an error.
      return fw::FDCan::kSuccess;
    }

    const int can_bus = spi->data[0];
    if (can_bus != 1 && can_bus != 2) {
      // TODO Record an error of some sort.
      return fw::FDCan::kSuccess;
    }
    const uint32_t id =
        (u8(spi->data[1]) << 24) |
        (u8(spi->data[2]) << 16) |
        (u8(spi->data[3]) << 8) |
        (u8(spi->data[4]));
    const int size = u8(spi->data[5]);
    if (size + 6 > spi->size) {
      // There is not enough data.  TODO Record an error.
      return fw::FDCan::kSuccess;
    }

    auto* const can = (can_bus == 1) ?
        can1_ :
        can2_;

    if (can) {
      return can->Send(id, std::string_view(&spi->data[6], size));
    } else {
      return fw::FDCan::kSuccess;
    }
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 0) {
      return {
        std::string_view("\x01", 1),
        {},
      };
    }
    if (address == 1) {
      return {
        std::string_view("\x01", 1),
        {},
      };
    }
    if (address == 16) {
      for (size_t i = 0; i < kBufferItems; i++) {
        const auto item = can_rx_queue_[i];
        address16_buf_[i] = (item == nullptr) ? 0 : (item->size + 6);
      }
      return {
        address16_buf_,
        {},
      };
    }
    if (address == 17) {
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
        std::string_view(current_can_buf_->data, current_can_buf_->size + 6),
        {},
      };
    }
    if (address == 18) {
      current_spi_buf_ = [&]() -> SpiReceiveBuf* {
        for (auto& buf : spi_buf_) {
          if (!buf.active) { return &buf; }
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

    if (start_handler_) {
      return start_handler_(address);
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

    if (address != 0 && address != 1 &&
        address != 16 && address != 17 && address != 18) {
      if (end_handler_) {
        end_handler_(address, bytes);
      }
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
  RegisterSPISlave spi_{
    timer_,
    PA_7, PA_6, PA_5, PA_4,
        [this](uint16_t address) {
          return this->ISR_Start(address);
        },
        [this](uint16_t address, int bytes) {
          return this->ISR_End(address, bytes);
        }};

  fw::FDCan* can1_ = nullptr;
  fw::FDCan* can2_ = nullptr;

  DigitalOut irq_;

  RegisterSPISlave::StartHandler start_handler_;
  RegisterSPISlave::EndHandler end_handler_;

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
