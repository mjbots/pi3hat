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

#include "fw/slot_rf_protocol.h"

#include <optional>

#include "mjlib/base/visitor.h"
#include "mjlib/micro/async_types.h"
#include "mjlib/micro/static_vector.h"

namespace micro = mjlib::micro;

namespace fw {

namespace {
constexpr int kSlotPeriodMs = 20;
constexpr int kNumChannels = 23;

}  // namespace

class SlotRfProtocol::Impl {
 public:
  Impl(fw::MillisecondTimer* timer,
       PinName irq_name,
       const Options& options)
      : options_(options),
        timer_(timer),
        irq_(irq_name, 0) {
  }

  void Start() {
    Restart();
  }

  void Poll() {
    MJ_ASSERT(!!nrf_);
    nrf_->Poll();

    if (!nrf_->is_data_ready()) {
      return;
    }

    rx_packet_.size = 0;
    nrf_->Read(&rx_packet_);

    // If we are a receiver, we need to mark ourselves as now locked
    // and update slot_timer_ to be ready for the next reception
    // cycle.
    if (!options_.ptx) {
      receive_mode_ = kLocked;
      slot_timer_ = kSlotPeriodMs;
      rx_miss_count_ = 0;
    }

    // Update our receive ages:
    for (auto& slot : rx_slots_) { slot.age++; }
    ParsePacket();
  }

  void PollMillisecond() {
    MJ_ASSERT(!!nrf_);
    nrf_->PollMillisecond();

    slot_timer_--;

    if (options_.ptx) {
      if (slot_timer_ == 0) {
        TransmitCycle();
        slot_timer_ = kSlotPeriodMs;
      } else if (slot_timer_ == (kSlotPeriodMs / 4)) {
        // Switch to the next channel.
        SwitchChannel();
      }
    } else {
      if (slot_timer_ == 0) {
        slot_timer_ = kSlotPeriodMs;
        rx_miss_count_++;

        if (receive_mode_ == kSynchronizing) {
          if (rx_miss_count_ > 20) {
            // Move on to the next channel.
            SwitchChannel();
            rx_miss_count_ = 0;
          }
        } else {
          if (rx_miss_count_ > 5) {
            // Whoops, count us as now needing to synchronize.
            receive_mode_ = kSynchronizing;
          }
        }
      } else if (slot_timer_ == (kSlotPeriodMs / 2) && receive_mode_ == kLocked) {
        // When receiving, we switch to the next channel halfway through
        // our time window.
        SwitchChannel();
        ReplyCycle();
      }
    }
  }

  uint32_t slot_bitfield() {
    irq_.write(0);
    return slot_bitfield_;
  }

  void tx_slot(int slot_idx, const Slot& slot) {
    tx_slots_[slot_idx] = slot;
  }

  const Slot& tx_slot(int slot_idx) const {
    return tx_slots_[slot_idx];
  }

  const Slot& rx_slot(int slot_idx) const {
    return rx_slots_[slot_idx];
  }

  uint8_t channel() const {
    return channels_[channel_];
  }

  uint32_t error() const {
    return nrf_->error();
  }

 private:
  void SwitchChannel() {
    channel_ = (channel_ + 1) % kNumChannels;
    nrf_->SelectRfChannel(channels_[channel_]);
  }

  void ParsePacket() {
    char* pos = rx_packet_.data;
    auto remaining = rx_packet_.size;

    while (remaining) {
      uint8_t header = static_cast<uint8_t>(*pos);
      uint8_t slot_index = header >> 4;
      uint8_t slot_size = header & 0x0f;
      remaining--;
      pos++;
      if (slot_size > remaining) {
        // TODO: Record this as malformed.
        return;
      }

      if (slot_index == 15) {
        // This is reserved for special codes and can only appear at
        // the end.
        break;
      }

      auto& slot = rx_slots_[slot_index];
      slot.age = 0;
      slot.size = slot_size;
      std::memcpy(slot.data, pos, slot_size);

      uint32_t cur_bitfield = (slot_bitfield_ >> (slot_index * 2)) & 0x03;
      cur_bitfield = (cur_bitfield + 1) % 4;
      slot_bitfield_ = (slot_bitfield_ & ~(0x03 << (slot_index * 2))) |
          (cur_bitfield << (slot_index * 2));

      irq_.write(1);

      pos += slot_size;
      remaining -= slot_size;
    }
  }

  void TransmitCycle() {
    PrepareTxPacket();

    // Now we send out our frame, whether or not it has anything in it
    // (that gives the receiver a chance to reply).
    nrf_->Transmit(&tx_packet_);
  }

  void ReplyCycle() {
    PrepareTxPacket();

    nrf_->QueueAck(&tx_packet_);
  }

  void PrepareTxPacket() {
    // Increment the ages for all slots.
    for (auto& slot : tx_slots_) {
      slot.age++;
    }

    // Pick a set of slots to send out.

    // For all slots which are enabled for this priority window, fill
    // our transmission buffer with those with the oldest age.
    auto enabled_slots = FindEnabledSlots(priority_count_);
    std::sort(enabled_slots.begin(), enabled_slots.end(),
              [&](auto lhs, auto rhs) {
                return tx_slots_[lhs].age > tx_slots_[rhs].age;
              });

    tx_packet_.size = 0;
    // Now loop through by age filling up whatever we can.
    for (auto slot_idx : enabled_slots) {
      const int remaining_size = 32 - tx_packet_.size;
      if ((tx_slots_[slot_idx].size + 1) < remaining_size) {
        EmitSlot(slot_idx);
      }
    }

    // The NRF won't send anything if there are no bytes at all.
    // Thus, use a placeholder if that is the case.  We need to send
    // something in order to give the receiver a chance to ack.
    if (tx_packet_.size == 0) {
      tx_packet_.data[0] = 0xff;
      tx_packet_.size = 1;
    }

    priority_count_ = (priority_count_ + 1) % 32;
  }

  micro::StaticVector<uint8_t, kNumSlots>
  FindEnabledSlots(uint8_t current_priority) const {
    micro::StaticVector<uint8_t, kNumSlots> result;
    uint32_t mask = 1 << current_priority;
    for (int i = 0; i < kNumSlots; i++) {
      if (tx_slots_[i].priority & mask) { result.push_back(i); }
    }
    return result;
  }

  void EmitSlot(int slot_index) {
    auto& size = tx_packet_.size;

    const int remaining = 32 - size;
    auto& slot = tx_slots_[slot_index];

    MJ_ASSERT((slot.size + 1) < remaining);

    tx_packet_.data[size] = (slot_index << 4) | slot.size;
    size++;
    std::memcpy(&tx_packet_.data[size], slot.data, slot.size);
    size += slot.size;
    slot.age = 0;
  }

  void Restart() {
    nrf_.emplace(
        timer_,
        [&]() {
          Nrf24l01::Options options;
          options.pins = options_.pins;

          options.ptx = options_.ptx;
          options.address_length = 5;
          options.id = SelectShockburstId();
          options.dynamic_payload_length = true;
          options.enable_crc = true;
          options.crc_length = 2;
          options.auto_retransmit_count = options_.auto_retransmit_count;
          options.auto_retransmit_delay_us = 1000;
          options.automatic_acknowledgment = true;
          options.initial_channel = 0;
          options.data_rate = options_.data_rate;
          options.output_power = options_.output_power;

          return options;
        }());

    // Generate our channel switching table.
    GenerateChannelTable();
  }

  uint64_t SelectShockburstId() const {
    const auto slot_id = options_.id;

    const auto byte_lsb = 0xc0 | (slot_id & 0x0f);

    const auto make_byte = [&](int shift) -> uint64_t {
      const auto shifted = slot_id >> shift;
      return (shifted & 0xfe) | (((shifted >> 1) & 0x01) ^ 0x01);
    };

    const auto byte1 = make_byte(4);
    const auto byte2 = make_byte(11);
    const auto byte3 = make_byte(18);
    const auto byte4 = make_byte(25);

    return (static_cast<uint64_t>(0)
            | byte_lsb
            | (byte1 << 8)
            | (byte2 << 16)
            | (byte3 << 24)
            | (byte4 << 32));
  }

  void GenerateChannelTable() {
    uint32_t prn = options_.id;
    int channel_count = 0;

    while (channel_count < kNumChannels) {
      prn = (prn * 0x0019660D) + 0x3c6ef35f;

      const uint8_t possible_channel = prn % 125;

      // See if this channel is usable.
      if (!EvaluatePossibleChannel(possible_channel, channel_count)) {
        continue;
      }

      // It is, add it to our list.
      channels_[channel_count] = possible_channel;
      channel_count++;
    }
  }

  bool EvaluatePossibleChannel(uint8_t possible_channel, int channel_count) {
    // If this channel has already been selected, then we discard it.
    for (int i = 0; i < channel_count; i++) {
      if (channels_[i] == possible_channel) { return false; }
    }

    // Evaluate our band limits.
    int band_count[4] = {};
    constexpr int band_channels[4] = { 31, 63, 95, 125 };
    constexpr int band_max[4] = {6, 6, 6, 5};

    const auto get_band = [&](int channel) {
      for (int band = 0; band < 4; band++) {
        if (channel > band_channels[band]) { continue; }
        return band;
      }
      return 0;
    };

    for (int i = 0; i < channel_count; i++) {
      const auto this_channel = channels_[i];
      const int band = get_band(this_channel);
      band_count[band]++;
    }

    const int this_band = get_band(possible_channel);
    if (band_count[this_band] >= band_max[this_band]) {
      return false;
    }

    return true;
  }

  const Options options_;
  fw::MillisecondTimer* const timer_;
  DigitalOut irq_;

  std::optional<Nrf24l01> nrf_;

  bool write_outstanding_ = false;
  micro::VoidCallback done_callback_;

  uint8_t channels_[kNumChannels] = {};
  uint8_t channel_ = 0;

  /// For transmitters, this is the canonical source of the system
  /// time.  For receivers, we attempt to synchronize this to
  /// transmitters.
  int slot_timer_ = kSlotPeriodMs;
  int priority_count_ = 0;
  uint32_t rx_miss_count_ = 0;

  uint32_t slot_bitfield_ = 0;
  Slot tx_slots_[kNumSlots] = {};
  Slot rx_slots_[kNumSlots] = {};

  Nrf24l01::Packet rx_packet_;
  Nrf24l01::Packet tx_packet_;

  enum ReceiveMode {
    kSynchronizing,
    kLocked,
  };

  ReceiveMode receive_mode_ = kSynchronizing;
};

SlotRfProtocol::SlotRfProtocol(MillisecondTimer* timer,
                               PinName irq_name,
                               const Options& options)
    : impl_(timer, irq_name, options) {}

SlotRfProtocol::~SlotRfProtocol() {}

void SlotRfProtocol::Poll() {
  impl_->Poll();
}

void SlotRfProtocol::PollMillisecond() {
  impl_->PollMillisecond();
}

void SlotRfProtocol::Start() {
  impl_->Start();
}

uint32_t SlotRfProtocol::slot_bitfield() {
  return impl_->slot_bitfield();
}

void SlotRfProtocol::tx_slot(int slot_idx, const Slot& slot) {
  impl_->tx_slot(slot_idx, slot);
}

const SlotRfProtocol::Slot& SlotRfProtocol::tx_slot(int slot_idx) const {
  return impl_->tx_slot(slot_idx);
}

const SlotRfProtocol::Slot& SlotRfProtocol::rx_slot(int slot_idx) const {
  return impl_->rx_slot(slot_idx);
}

uint8_t SlotRfProtocol::channel() const {
  return impl_->channel();
}

uint32_t SlotRfProtocol::error() const {
  return impl_->error();
}

}
