// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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
#include "fw/fifo.h"
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
///    byte 0: the constant value 4
/// 1: Interface type
///    byte 0: the constant value 1
/// 2: Receive status
///    byte 0-5 - Size of up to 6 received frames.  0 means no frame
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
/// 6: Error status
///   byte 0 - CAN1
///    bit 7-6 activity
///    bit 5-3 data last error code
///    bit 2-0 last error code
///   byte 1 - CAN1
///    bit 3 protocol exception
///    bit 2 bus_off
///    bit 1 warning status
///    bit 0 error passive
///   byte 2 - CAN1 rx error count
///   byte 3 - CAN1 tx error count
///   byte 4 - CAN1 reset count
///   byte 5 - reserved
///   byte 6 - CAN2
///    bit 7-6 activity
///    bit 5-3 data last error code
///    bit 2-0 last error code
///   byte 7 - CAN2
///    bit 3 protocol exception
///    bit 2 bus_off
///    bit 1 warning status
///    bit 0 error passive
///   byte 8 - CAN2 rx error count
///   byte 9 - CAN2 tx error count
///   byte 10 - CAN2 reset count
///   byte 11 - reserved
/// 7: Read configuration 1
///   bytes: The contents of the 'Configuration' structure
/// 8: Read configuration 2
///   bytes: The contents of the 'Configuration' structure
/// 9: Write configuration 1
///   bytes: The contents of the 'Configuration' structure
/// 10: Write configuration 2
///   bytes: The contents of the 'Configuration' structure


// Protocol history:
//
// Version 3: Added register 7-10 for configuration.
// Version 4: Stop getting borked if no-one acks our packets.

class CanBridge {
 public:
  static constexpr int kMaxSpiFrameSize = 70;
  static constexpr int kBufferItems = 32;
  static constexpr int kReceiveStatusCount = 6;

  struct Pins {
    PinName irq_name = NC;
  };

  struct Rate {
    int8_t prescaler = -1;
    int8_t sync_jump_width = -1;
    int8_t time_seg1 = -1;
    int8_t time_seg2 = -1;

    bool operator==(const Rate& rhs) const {
      return prescaler == rhs.prescaler &&
          sync_jump_width == rhs.sync_jump_width &&
          time_seg1 == rhs.time_seg1 &&
          time_seg2 == rhs.time_seg2;
    }

    bool operator!=(const Rate& rhs) {
      return !(*this == rhs);
    }
  } __attribute__((packed));

  struct Configuration {
    int32_t slow_bitrate = 1000000;
    int32_t fast_bitrate = 5000000;
    int8_t fdcan_frame = 1;
    int8_t bitrate_switch = 1;
    int8_t automatic_retransmission = 1;
    int8_t restricted_mode = 0;
    int8_t bus_monitor = 0;

    // If any members of either 'rate' structure are non-negative, use
    // them instead of the 'bitrate' fields above.  Each rate applies
    // to a base clock rate of 85MHz.
    Rate std_rate;
    Rate fd_rate;

    // If no progress has been made sending frames in the last N ms,
    // cancel the entire hardware CAN queue.  This can keep the
    // hardware from getting stuck if a frame is sent that nothing
    // will ever acknowledge.  If zero, then never cancel frames.
    uint32_t cancel_all_ms = 50;

    bool operator==(const Configuration& rhs) const {
      return slow_bitrate == rhs.slow_bitrate &&
          fast_bitrate == rhs.fast_bitrate &&
          fdcan_frame == rhs.fdcan_frame &&
          bitrate_switch == rhs.bitrate_switch &&
          automatic_retransmission == rhs.automatic_retransmission &&
          restricted_mode == rhs.restricted_mode &&
          bus_monitor == rhs.bus_monitor &&
          std_rate == rhs.std_rate &&
          fd_rate == rhs.fd_rate &&
          cancel_all_ms == rhs.cancel_all_ms;
    }

    bool operator!=(const Configuration& rhs) const {
      return !(*this == rhs);
    }
  } __attribute__((packed));

  static FDCan::Options MakeCanOptions(
      const FDCan::Options& source,
      const Configuration& config) {

    FDCan::Options result;
    result.td = source.td;
    result.rd = source.rd;

    result.slow_bitrate = config.slow_bitrate;
    result.fast_bitrate = config.fast_bitrate;
    result.fdcan_frame = config.fdcan_frame;
    result.bitrate_switch = config.bitrate_switch;
    result.automatic_retransmission = config.automatic_retransmission;
    result.restricted_mode = config.restricted_mode;
    result.bus_monitor = config.bus_monitor;

    result.rate_override.prescaler = config.std_rate.prescaler;
    result.rate_override.sync_jump_width = config.std_rate.sync_jump_width;
    result.rate_override.time_seg1 = config.std_rate.time_seg1;
    result.rate_override.time_seg2 = config.std_rate.time_seg2;

    result.fdrate_override.prescaler = config.fd_rate.prescaler;
    result.fdrate_override.sync_jump_width = config.fd_rate.sync_jump_width;
    result.fdrate_override.time_seg1 = config.fd_rate.time_seg1;
    result.fdrate_override.time_seg2 = config.fd_rate.time_seg2;

    return result;
  }

  static Configuration GetCanConfig(const FDCan::Options& source) {
    Configuration result;
    result.slow_bitrate = source.slow_bitrate;
    result.fast_bitrate = source.fast_bitrate;
    result.fdcan_frame = source.fdcan_frame;
    result.bitrate_switch = source.bitrate_switch;
    result.automatic_retransmission = source.automatic_retransmission;
    result.restricted_mode = source.restricted_mode;
    result.bus_monitor = source.bus_monitor;

    result.std_rate.prescaler = source.rate_override.prescaler;
    result.std_rate.sync_jump_width = source.rate_override.sync_jump_width;
    result.std_rate.time_seg1 = source.rate_override.time_seg1;
    result.std_rate.time_seg2 = source.rate_override.time_seg2;

    result.fd_rate.prescaler = source.fdrate_override.prescaler;
    result.fd_rate.sync_jump_width = source.fdrate_override.sync_jump_width;
    result.fd_rate.time_seg1 = source.fdrate_override.time_seg1;
    result.fd_rate.time_seg2 = source.fdrate_override.time_seg2;

    return result;
  }

  CanBridge(fw::MillisecondTimer* timer,
            fw::FDCan* can1, fw::FDCan* can2,
            const Pins& pins)
      : timer_{timer},
        pins_{pins},
        can1_{can1},
        can2_{can2},
        irq_{pins.irq_name, 0} {
    if (can1_) {
      can_config1_ = GetCanConfig(can1_->options());
      can_config1_shadow_ = can_config1_;
    }
    if (can2_) {
      can_config2_ = GetCanConfig(can2_->options());
      can_config2_shadow_ = can_config2_;
    }
  }

  void Poll() {
    if (reset_can_.load()) {
      reset_can_.store(false);
      if (can1_) {
        can1_->Reset(MakeCanOptions(can1_->options(), can_config1_));
      }
      if (can2_) {
        can2_->Reset(MakeCanOptions(can2_->options(), can_config2_));
      }
    }

    auto can_poll = [&](int bus_id, auto* can, uint8_t* reset_count) {
      const auto status = can->status();
      uint8_t* const can_status = &status_buf_[(bus_id - 1) * 6];
      can_status[0] = (
          (status.LastErrorCode << 0) |
          (status.DataLastErrorCode << 3) |
          (status.Activity << 6) |
          0);
      can_status[1] = (
          (status.ErrorPassive << 0) |
          (status.Warning << 1) |
          (status.BusOff << 2) |
          (status.ProtocolException << 3) |
          0);

      const auto error_counters = can->error_counters();
      can_status[2] = error_counters.RxErrorCnt;
      can_status[3] = error_counters.TxErrorCnt;
      can_status[4] = *reset_count;

      if (status.BusOff) {
        // We need to reset.
        (*reset_count)++;
        can->RecoverBusOff();
      }

      // If our queue is full, don't even bother polling.
      if (rx_buf_.full()) { return; }

      const bool rx = can->Poll(&can_header_, rx_buffer_);
      if (!rx) {
        return;
      }

      auto* const this_buf = rx_buf_.prepare_push();

      if (this_buf == nullptr) {
        // This shouldn't be possible, as we checked for fullness
        // before trying to Poll and nothing else should have been
        // able to add to the queue in the meantime.
        mbed_die();
        return;
      }

      const int size = ParseDlc(can_header_.DataLength);

      this_buf->data[0] = ((bus_id == 2) ? 0x80 : 0x00) | (size + 1);
      const auto id = can_header_.Identifier;
      this_buf->data[1] = (id >> 24) & 0xff;
      this_buf->data[2] = (id >> 16) & 0xff;
      this_buf->data[3] = (id >> 8) & 0xff;
      this_buf->data[4] = (id >> 0) & 0xff;
      this_buf->size = size;
      std::memcpy(&this_buf->data[5], rx_buffer_, size);

      // Finalize this, now that all the data is in place.
      rx_buf_.push();
      irq_.write(1);
    };

    if (can1_) {
      can_poll(1, can1_, &can1_reset_count_);
    }
    if (can2_) {
      can_poll(2, can2_, &can2_reset_count_);
    }

    // Look to see if we have anything to send.
    if (!tx_buf_.empty()) {
      // Let's send this out.
      const auto* to_send = tx_buf_.prepare_get();
      const auto send_result = SendCan(to_send);
      if (send_result == fw::FDCan::kNoSpace) {
        // The CAN interface is busy.  Just keep waiting around.
      } else {
        // We successfully handed this off to the CAN layer.
        tx_buf_.get();
      }
    }
  }

  void PollMillisecond() {
    can1_cancel_all_count_++;
    can2_cancel_all_count_++;

    // 50ms after the last thing was added to the TQ queue, just
    // cancel everything in it.  This is relatively simple, and
    // ensures we make at least some forward progress even when frames
    // are accidentally sent that nothing acknowledges while automatic
    // retransmission is enabled.
    if (can1_ &&
        can1_cancel_all_count_ > can_config1_.cancel_all_ms &&
        can_config1_.cancel_all_ms > 0) {
      can1_->CancelAll();
      can1_cancel_all_count_ = 0;
    }
    if (can2_ &&
        can2_cancel_all_count_ > can_config2_.cancel_all_ms &&
        can_config2_.cancel_all_ms > 0) {
      can2_->CancelAll();
      can2_cancel_all_count_ = 0;
    }
  }

  uint8_t queue_size() const {
    return rx_buf_.empty() ? 0 : 1;
  }

  static bool IsSpiAddress(uint16_t address) {
    return address <= 10;
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 0) {
      return {
        std::string_view("\x04", 1),
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
      address16_buf_[kReceiveStatusCount] = 0;
      for (size_t i = 0; i < kReceiveStatusCount; i++) {
        const auto item = rx_buf_(i);
        address16_buf_[i] = (item == nullptr) ? 0 : (item->size + 5);
        address16_buf_[kReceiveStatusCount] += address16_buf_[i];
      }
      address16_buf_[kReceiveStatusCount] ^= 0xff;

      return {
        std::string_view(reinterpret_cast<const char*>(&address16_buf_[0]),
                         sizeof(address16_buf_)),
        {},
      };
    }
    if (address == 3) {
      if (rx_buf_.empty()) {
        current_can_buf_ = nullptr;
        return { {}, {} };
      }

      current_can_buf_ = rx_buf_.prepare_get();

      return {
        std::string_view(current_can_buf_->data, current_can_buf_->size + 5),
        {},
      };
    }
    if (address == 4 || address == 5) {
      if (tx_buf_.full()) {
        return { {}, {} };
      }

      current_spi_buf_ = tx_buf_.prepare_push();
      current_spi_buf_->address = address;

      return {
        {},
        mjlib::base::string_span(current_spi_buf_->data, kMaxSpiFrameSize),
      };
    }
    if (address == 6) {
      return {
        std::string_view(reinterpret_cast<const char*>(&status_buf_[0]),
                         sizeof(status_buf_)),
        {},
      };
    }
    if (address == 7) {
      return {
        std::string_view(
            reinterpret_cast<const char*>(&can_config1_),
            sizeof(can_config1_)),
        {},
      };
    }
    if (address == 8) {
      return {
        std::string_view(
            reinterpret_cast<const char*>(&can_config2_),
            sizeof(can_config2_)),
        {},
      };
    }
    if (address == 9) {
      return {
        {},
        mjlib::base::string_span(
            reinterpret_cast<char*>(&can_config1_shadow_),
            sizeof(can_config1_shadow_)),
      };
    }
    if (address == 10) {
      return {
        {},
        mjlib::base::string_span(
            reinterpret_cast<char*>(&can_config2_shadow_),
            sizeof(can_config2_shadow_)),
      };
    }

    return { {}, {} };
  }

  void ISR_End(uint16_t address, int bytes) {
    if ((address == 9 || address == 10) &&
        bytes == sizeof(can_config1_shadow_)) {
      if (can_config1_ != can_config1_shadow_) {
        can_config1_ = can_config1_shadow_;
        reset_can_.store(true);
      }
      if (can_config2_ != can_config2_shadow_) {
        can_config2_ = can_config2_shadow_;
        reset_can_.store(true);
      }
    }

    if (current_can_buf_) {
      current_can_buf_ = nullptr;
      if (!rx_buf_.empty()) {
        rx_buf_.get();
      }
      if (rx_buf_.empty()) {
        irq_.write(0);
      }
    }

    if (current_spi_buf_) {
      current_spi_buf_->size = bytes;
      current_spi_buf_ = nullptr;

      // Mark this as ready to go out over CAN.
      if (!tx_buf_.full()) {
        tx_buf_.push();
      }
    }
  }

 private:
  struct SpiReceiveBuf {
    char data[kMaxSpiFrameSize] = {};
    int size = 0;
    int address = 0;
  };

  struct CanReceiveBuf {
    char data[kMaxSpiFrameSize] = {};
    int size = 0;
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
      const auto result =
          can->Send(id, std::string_view(&spi->data[min_size], size));
      if (result == fw::FDCan::kSuccess) {
        // Reset our "cancel everything" counters, because we
        // successfully enqueued something.
        if (can == can1_) {
          can1_cancel_all_count_ = 0;
        } else if (can == can2_) {
          can2_cancel_all_count_ = 0;
        } else {
          mbed_die();
        }
      }
      return result;
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

  uint32_t can1_cancel_all_count_ = 0;
  uint32_t can2_cancel_all_count_ = 0;

  DigitalOut irq_;

  AtomicFifo<SpiReceiveBuf, kBufferItems> tx_buf_;
  SpiReceiveBuf* current_spi_buf_ = nullptr;

  AtomicFifo<CanReceiveBuf, kBufferItems> rx_buf_;
  CanReceiveBuf* current_can_buf_ = nullptr;

  FDCAN_RxHeaderTypeDef can_header_ = {};
  char rx_buffer_[64] = {};

  uint8_t can1_reset_count_ = 0;
  uint8_t can2_reset_count_ = 0;


  uint8_t address16_buf_[kReceiveStatusCount + 1] = {};
  uint8_t status_buf_[12] = {};

  Configuration can_config1_;
  Configuration can_config1_shadow_;
  Configuration can_config2_;
  Configuration can_config2_shadow_;

  std::atomic<bool> reset_can_{false};
};


}
