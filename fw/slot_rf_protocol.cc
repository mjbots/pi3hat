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
#include "mjlib/micro/static_vector.h"

namespace micro = mjlib::micro;

namespace fw {

namespace {
constexpr int kSlotPeriodMs = 20;
constexpr int kNumChannels = 23;

uint64_t SelectShockburstId(uint32_t slot_id) {
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

}  // namespace

class SlotRfProtocol::Impl {
 public:
  Impl(fw::MillisecondTimer* timer,
       const Options& options)
      : options_(options),
        timer_(timer) {
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

    remotes_[last_transmit_remote_index_].ParsePacket(rx_packet_);
  }

  void PollMillisecond() {
    MJ_ASSERT(!!nrf_);
    nrf_->PollMillisecond();

    slot_timer_--;

    if (!options_.ptx) {
      PollMillisecondReceive();
    } else {
      PollMillisecondTransmit();
    }
  }

  void PollMillisecondTransmit() {
    // We split up our total slot period into regions based on how
    // many remotes we have.
    constexpr int per_slot_count = kSlotPeriodMs / kNumRemotes;
    const bool final_cycle = (slot_timer_ == 0);

    remote_index_ = std::min(kNumRemotes - 1, slot_timer_ / per_slot_count);
    const int remote_timer = slot_timer_ - remote_index_ * per_slot_count;

    auto* remote = &remotes_[remote_index_];

    if (remote->enabled()) {
      if (remote_timer == 0) {
        last_transmit_remote_index_ = remote_index_;
        TransmitCycle();
      } else if (remote_timer == 2) {
        // Switch to the next remote 2ms before we transmit.
        nrf_->SelectId(remote->shockburst_id());
        nrf_->SelectRfChannel(remote->channel(channel_index_));
      }
    }

    if (final_cycle) {
      SwitchChannel();
      slot_timer_ = kSlotPeriodMs;
    }
  }

  void PollMillisecondReceive() {
    auto* remote = &remotes_.front();

    if (slot_timer_ == 0) {
      slot_timer_ = kSlotPeriodMs;
      rx_miss_count_++;

      if (receive_mode_ == kSynchronizing) {
        if (rx_miss_count_ > 20) {
          // Move on to the next channel.
          SwitchChannel();
          nrf_->SelectRfChannel(remote->channel(channel_index_));
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
      nrf_->SelectRfChannel(remote->channel(channel_index_));
      ReplyCycle();
    }
  }

  Remote* remote(int index) {
    MJ_ASSERT(index >= 0 && index < static_cast<int>(remotes_.size()));
    return &remotes_[index];
  }

  uint8_t channel() const {
    return remotes_[remote_index_].channel(channel_index_);
  }

  uint32_t error() const {
    return nrf_->error();
  }

  bool locked() const {
    return options_.ptx == 1 || receive_mode_ == kLocked;
  }

 private:
  class ConcreteRemote : public Remote {
   public:
    uint32_t slot_bitfield() const override {
      return slot_bitfield_;
    }

    void tx_slot(int slot_idx, const Slot& slot) override {
      tx_slots_[slot_idx] = slot;
    }

    const Slot& tx_slot(int slot_idx) const override {
      return tx_slots_[slot_idx];
    }

    const Slot& rx_slot(int slot_idx) const override {
      return rx_slots_[slot_idx];
    }

    bool enabled() const {
      return enabled_;
    }

    void SetId(uint32_t id) {
      enabled_ = id != 0;
      if (!enabled_) { return; }

      shockburst_id_ = SelectShockburstId(id);

      uint32_t prn = id;
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

    uint64_t shockburst_id() const {
      return shockburst_id_;
    }

    uint32_t channel(int index) const {
      return channels_[index];
    }

    void ParsePacket(const Nrf24l01::Packet& packet) {
      // Update our receive ages:
      for (auto& slot : rx_slots_) { slot.age++; }

      const char* pos = packet.data;
      auto remaining = packet.size;

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

        pos += slot_size;
        remaining -= slot_size;
      }
    }

    void PrepareTxPacket(Nrf24l01::Packet* packet) {
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

      packet->size = 0;
      // Now loop through by age filling up whatever we can.
      for (auto slot_idx : enabled_slots) {
        const int remaining_size = 32 - packet->size;
        if ((tx_slots_[slot_idx].size + 1) < remaining_size) {
          EmitSlot(packet, slot_idx);
        }
      }

      // The NRF won't send anything if there are no bytes at all.
      // Thus, use a placeholder if that is the case.  We need to send
      // something in order to give the receiver a chance to ack.
      if (packet->size == 0) {
        packet->data[0] = 0xff;
        packet->size = 1;
      }

      priority_count_ = (priority_count_ + 1) % 32;
    }

   private:
    micro::StaticVector<uint8_t, kNumSlots>
    FindEnabledSlots(uint8_t current_priority) const {
      micro::StaticVector<uint8_t, kNumSlots> result;
      uint32_t mask = 1 << current_priority;
      for (int i = 0; i < kNumSlots; i++) {
        if (tx_slots_[i].priority & mask) { result.push_back(i); }
      }
      return result;
    }

    void EmitSlot(Nrf24l01::Packet* packet, int slot_index) {
      auto& size = packet->size;

      const int remaining = 32 - size;
      auto& slot = tx_slots_[slot_index];

      MJ_ASSERT((slot.size + 1) < remaining);

      packet->data[size] = (slot_index << 4) | slot.size;
      size++;
      std::memcpy(&packet->data[size], slot.data, slot.size);
      size += slot.size;
      slot.age = 0;
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

    bool enabled_ = false;
    int priority_count_ = 0;
    uint64_t shockburst_id_ = 0;
    uint8_t channels_[kNumChannels] = {};
    uint32_t slot_bitfield_ = 0;
    Slot tx_slots_[kNumSlots] = {};
    Slot rx_slots_[kNumSlots] = {};
  };

  void SwitchChannel() {
    channel_index_ = (channel_index_ + 1) % kNumChannels;
  }

  void TransmitCycle() {
    remotes_[remote_index_].PrepareTxPacket(&tx_packet_);

    // Now we send out our frame, whether or not it has anything in it
    // (that gives the receiver a chance to reply).
    nrf_->Transmit(&tx_packet_);
  }

  void ReplyCycle() {
    remotes_[remote_index_].PrepareTxPacket(&tx_packet_);
    nrf_->QueueAck(&tx_packet_);
  }

  void Restart() {
    MJ_ASSERT(options_.ids.size() == remotes_.size());
    for (size_t i = 0; i < remotes_.size(); i++) {
      remotes_[i].SetId(options_.ids[i]);
    }
    remote_index_ = 0;

    nrf_.emplace(
        timer_,
        [&]() {
          Nrf24l01::Options options;
          options.pins = options_.pins;

          options.ptx = options_.ptx;
          options.address_length = 5;
          options.id = remotes_.front().shockburst_id();
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

  }

  const Options options_;
  fw::MillisecondTimer* const timer_;

  std::optional<Nrf24l01> nrf_;

  std::array<ConcreteRemote, kNumRemotes> remotes_;
  uint8_t channel_index_ = 0;
  uint8_t remote_index_ = 0;
  uint8_t last_transmit_remote_index_ = 0;

  /// For transmitters, this is the canonical source of the system
  /// time.  For receivers, we attempt to synchronize this to
  /// transmitters.
  int slot_timer_ = kSlotPeriodMs;
  uint32_t rx_miss_count_ = 0;

  Nrf24l01::Packet rx_packet_;
  Nrf24l01::Packet tx_packet_;

  enum ReceiveMode {
    kSynchronizing,
    kLocked,
  };

  ReceiveMode receive_mode_ = kSynchronizing;
};

SlotRfProtocol::SlotRfProtocol(MillisecondTimer* timer,
                               const Options& options)
    : impl_(timer, options) {}

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

SlotRfProtocol::Remote* SlotRfProtocol::remote(int index) {
  return impl_->remote(index);
}

uint8_t SlotRfProtocol::channel() const {
  return impl_->channel();
}

uint32_t SlotRfProtocol::error() const {
  return impl_->error();
}

bool SlotRfProtocol::locked() const {
  return impl_->locked();
}

}
