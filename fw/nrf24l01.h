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

#include <string_view>

#include "PinNames.h"

#include "mjlib/base/string_span.h"
#include "mjlib/micro/pool_ptr.h"

#include "fw/millisecond_timer.h"

namespace fw {

class Nrf24l01 {
 public:
  struct Pins {
    ////////////////////
    // Pin configuration
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    PinName irq = NC;
    PinName ce = NC;
  };

  struct Options {
    Pins pins;

    ///////////////////////
    // Device configuration

    bool ptx = true;  // if false, then PRX mode
    int address_length = 5;
    uint64_t id = 0;
    bool dynamic_payload_length = true;
    bool enable_crc = true;
    int crc_length = 2;
    int auto_retransmit_count = 0;
    int auto_retransmit_delay_us = 1000;
    bool automatic_acknowledgment = false;
    int initial_channel = 2;

    // Can be one of 250000, 1000000, or 2000000;
    int data_rate = 1000000;

    // Can be one of -18, -12, -6, 0.
    int output_power = 0;

    Options() {}
  };

  Nrf24l01(MillisecondTimer*, const Options& = Options());
  ~Nrf24l01();

  void Poll();

  void PollMillisecond();

  /// Return true if the device has completed its power on cycle.
  bool ready() const;

  /// Switch to one of the possible 125 channels (0-124)
  void SelectRfChannel(uint8_t);

  /// Return true if there is data available to read.
  bool is_data_ready();

  struct Status {
    uint8_t status_reg = 0;
    uint32_t retransmit_exceeded = 0;
  };
  Status status();

  struct Packet {
    size_t size = 0;
    char data[32] = {};
  };

  /// Read the next available data packet.  @return false if no data
  /// was available.
  bool Read(Packet*);

  /// Transmit a packet.
  void Transmit(const Packet*);

  /// Queue the given packet to be sent as the next auto
  /// acknowledgement.  This can only be called if Options::ptx == false
  void QueueAck(const Packet*);

  uint8_t ReadRegister(uint8_t);
  void ReadRegister(uint8_t, mjlib::base::string_span);

  void WriteRegister(uint8_t, std::string_view);

  uint32_t error() const { return error_; }

 private:
  void ReadPacket();
  void VerifyRegister(uint8_t address, std::string_view);
  void VerifyRegister(uint8_t address, uint8_t value);

  void WriteConfig();
  void Configure();
  uint8_t GetConfig() const;

  MillisecondTimer* const timer_;
  const Options options_;

  SPI spi_;

  class SpiMaster {
   public:
    SpiMaster(SPI*, PinName cs, MillisecondTimer*);
    uint8_t Command(uint8_t command,
                    std::string_view,
                    mjlib::base::string_span);
    uint8_t WriteRegister(uint8_t address, std::string_view);
    uint8_t WriteRegister(uint8_t address, uint8_t data);

    uint8_t ReadRegister(uint8_t address, mjlib::base::string_span);
    uint8_t ReadRegister(uint8_t address);
    bool VerifyRegister(uint8_t address, std::string_view);
    bool VerifyRegister(uint8_t address, uint8_t value);

   private:
    SPI* const spi_;
    DigitalOut cs_;
    MillisecondTimer* const timer_;
    char buf_[16] = {};
  };

  SpiMaster nrf_;

  DigitalIn irq_;
  DigitalOut ce_;

  enum ConfigureState {
    kPowerOnReset,
    kEnteringStandby,
    kStandby,
  };
  ConfigureState configure_state_ = kPowerOnReset;
  uint32_t start_entering_standby_ = 0;

  bool is_data_ready_ = false;
  bool rx_overflow_ = false;

  uint32_t retransmit_exceeded_ = 0;
  Packet rx_packet_;

  uint32_t error_ = 0;

};

}
