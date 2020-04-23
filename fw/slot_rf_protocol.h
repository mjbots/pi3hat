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

#include "mjlib/micro/static_ptr.h"

#include "fw/millisecond_timer.h"
#include "fw/nrf24l01.h"

namespace fw {

class SlotRfProtocol {
 public:
  static constexpr int kNumSlots = 15;

  struct Options {
    bool ptx = true;
    uint32_t id = 0x3045;
    int32_t data_rate = 1000000;
    int32_t output_power = 0;
    int32_t auto_retransmit_count = 0;

    Nrf24l01::Pins pins;
  };

  SlotRfProtocol(MillisecondTimer*,
                 PinName irq_name,
                 const Options& options);
  ~SlotRfProtocol();

  void Poll();
  void PollMillisecond();
  void Start();

  struct Slot {
    uint32_t priority = 0;
    uint8_t size = 0;
    uint32_t age = 0;
    uint8_t data[16] = {};
  };

  /// Return a bitfield with 2 bits per slot.  The 2 bit number
  /// increments upon each receipt of that slot.  This can be used to
  /// efficiently poll to see if any slots have been received and
  /// which ones have been received.
  uint32_t slot_bitfield();

  /// Queue the given slot to be transmitted.
  void tx_slot(int slot_idx, const Slot&);

  /// Return the current value of the given tx slot.
  const Slot& tx_slot(int slot_idx) const;

  /// Return the current value of the given receive slot.
  const Slot& rx_slot(int slot_idx) const;

  /// Return the current rf channel.
  uint8_t channel() const;

  /// Return any error flags.
  uint32_t error() const;

 private:
  class Impl;
  mjlib::micro::StaticPtr<Impl, 2048> impl_;
};

}
