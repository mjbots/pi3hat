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

#include "fw/nrf24l01.h"

#include "mjlib/base/string_span.h"

namespace fw {

Nrf24l01::SpiMaster::SpiMaster(SPI* spi, PinName cs, MillisecondTimer* timer)
    : spi_(spi), cs_(cs, 1), timer_(timer) {
}

uint8_t Nrf24l01::SpiMaster::Command(
    uint8_t command,
    std::string_view data_in,
    mjlib::base::string_span data_out) {
  cs_.write(0);

  // The nrf24l01 has a 38ns CS setup time.  8 nops should get us
  // that for any stm32 frequency.
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");

  const uint8_t status = spi_->write(command);

  const auto to_transfer =
      std::max<std::size_t>(data_in.size(), data_out.size());
  for (size_t i = 0; i < to_transfer; i++) {
    const uint8_t out_byte = spi_->write(
        (i < data_in.size()) ? data_in[i] : 0);
    if (i < static_cast<std::size_t>(data_out.size())) {
      data_out[i] = out_byte;
    }
  }

  cs_.write(1);

  return status;
}

uint8_t Nrf24l01::SpiMaster::WriteRegister(uint8_t address, std::string_view data) {
  return Command(0x20 + address, data, {});
}

uint8_t Nrf24l01::SpiMaster::WriteRegister(uint8_t address, uint8_t data) {
  return WriteRegister(address, {reinterpret_cast<const char*>(&data), 1});
}

uint8_t Nrf24l01::SpiMaster::ReadRegister(
    uint8_t address, mjlib::base::string_span data) {
  return Command(0x00 + address, {}, data);
}

uint8_t Nrf24l01::SpiMaster::ReadRegister(uint8_t address) {
  uint8_t result = 0;
  ReadRegister(address, {reinterpret_cast<char*>(&result), 1});
  return result;
}

bool Nrf24l01::SpiMaster::VerifyRegister(uint8_t address, std::string_view data) {
  constexpr int kMaxRetries = 2;

  for (int i = 0; i < kMaxRetries; i++) {
    WriteRegister(address, data);
    ReadRegister(address, {buf_, static_cast<ssize_t>(data.size())});
    if (data == std::string_view{buf_, data.size()}) {
      return true;
    }
  }

  // Well, we failed.  Report an error.
  return false;
}

bool Nrf24l01::SpiMaster::VerifyRegister(uint8_t address, uint8_t value) {
  return VerifyRegister(address, {reinterpret_cast<const char*>(&value), 1});
}

Nrf24l01::Nrf24l01(MillisecondTimer* timer, const Options& options)
    : timer_(timer),
      options_(options),
      spi_(options.pins.mosi, options.pins.miso, options.pins.sck),
      nrf_(&spi_, options.pins.cs, timer),
      irq_(options.pins.irq),
      ce_(options.pins.ce, 0) {
  spi_.frequency(10000000);
}

Nrf24l01::~Nrf24l01() {}

void Nrf24l01::Poll() {
  if (irq_.read() == 0) {
    // We have some interrupt to deal with.  Read the status.
    const uint8_t status = nrf_.Command(0xff, {}, {});

    if (status & (1 << 6)) {
      ReadPacket();

      if (is_data_ready_) { rx_overflow_ = true; }
      is_data_ready_ = true;
    }
    if (status & (1 << 4)) {
      // Retransmit count exceeded!
      retransmit_exceeded_++;

      // Flush our TX FIFO.
      nrf_.Command(0xe1, {}, {});
    }

    const uint8_t maybe_to_clear = status & 0x70;
    if (maybe_to_clear) {
      // Yes!
      nrf_.WriteRegister(0x07, maybe_to_clear);
    }
  }
}

void Nrf24l01::PollMillisecond() {
  const auto now = timer_->read_ms();
  // The NRF isn't turned on for 100ms after power up.
  switch (configure_state_) {
    case kPowerOnReset: {
      // While we are in power on reset, leave CE off.
      ce_.write(0);

      // This check can be absolute, because the device only has to
      // do power on reset once.
      if (now < 150) { return; }

      WriteConfig();
      configure_state_ = kEnteringStandby;
      start_entering_standby_ = now;
      return;
    }
    case kEnteringStandby: {
      if ((now - start_entering_standby_) < 2) { return; }

      Configure();
      configure_state_ = kStandby;
      return;
    }
    case kStandby: {
      break;
    }
  }
}

bool Nrf24l01::ready() const {
  return configure_state_ == kStandby;
}

void Nrf24l01::SelectRfChannel(uint8_t channel) {
  MJ_ASSERT(channel < 125);
  if (options_.ptx == 0) {
    // To reliably change the frequency, the receiver needs to be
    // disabled.  It seems to kinda work only for a few limited
    // channels without doing this.
    ce_.write(0);
  }
  VerifyRegister(0x05, channel & 0x7f);  // RF_CH
  if (options_.ptx == 0) {
    ce_.write(1);
  }
}

bool Nrf24l01::is_data_ready() {
  return is_data_ready_;
}

bool Nrf24l01::Read(Packet* packet)  {
  if (!is_data_ready_) {
    packet->size = 0;
    return false;
  }
  *packet = rx_packet_;
  rx_packet_.size = 0;

  // Check to see if there is more remaining.
  const auto status_reg = nrf_.Command(0xff, {}, {});
  is_data_ready_ = ((status_reg >> 1) & 0x07) != 0x07;
  if (is_data_ready_) {
    ReadPacket();
  }

  return true;
}

void Nrf24l01::Transmit(const Packet* packet) {
  MJ_ASSERT(options_.ptx == 1);
  nrf_.Command(0xa0, {&packet->data[0], packet->size}, {});
  // Strobe CE to start this transmit.
  ce_.write(1);
  timer_->wait_us(10);
  ce_.write(0);
}

void Nrf24l01::QueueAck(const Packet* packet) {
  // We always use PPP == 0
  nrf_.Command(0xa8, {&packet->data[0], packet->size}, {});
}

void Nrf24l01::ReadPacket() {
  uint8_t payload_width = 0;
  nrf_.Command(0x60,  // R_RX_PL_WID
               {},
               {reinterpret_cast<char*>(&payload_width), 1});

  rx_packet_.size = payload_width;
  if (payload_width) {
    nrf_.Command(0x61, {}, {&rx_packet_.data[0],
            static_cast<ssize_t>(payload_width)});
  }
}

void Nrf24l01::VerifyRegister(uint8_t address, std::string_view data) {
  if (!nrf_.VerifyRegister(address, data) && error_ == 0) {
    // Just report the first error.
    error_ = 0x100 | address;
  }
}

void Nrf24l01::VerifyRegister(uint8_t address, uint8_t value) {
  if (!nrf_.VerifyRegister(address, value) && error_ == 0) {
    error_ = 0x100 | address;
  }
}

void Nrf24l01::WriteConfig() {
  nrf_.WriteRegister(0x00, GetConfig());  // CONFIG
  // Now we need to wait another 1.5ms to enter standby mode for this
  // to take effect.
}

void Nrf24l01::Configure() {
  VerifyRegister(0x00, GetConfig());

  VerifyRegister(
      0x01, // EN_AA - enable auto-acknowledge per rx channel
      (options_.automatic_acknowledgment ? 0x01 : 0x00));
  VerifyRegister(
      0x02, // EN_RXADDR
      (options_.ptx == 0 || options_.automatic_acknowledgment) ?
      0x01 : 0);  // EN_RXADDR enable 0
  VerifyRegister(
      0x03,  // SETUP_AW
      [&]() {
        if (options_.address_length == 3) { return 1; }
        if (options_.address_length == 4) { return 2; }
        if (options_.address_length == 5) { return 3; }
        // default to length 5 address
        return 3;
      }());
  VerifyRegister(
      0x04,  // SETUP_RETR
      std::min(15, options_.auto_retransmit_delay_us / 250) << 4 |
      std::min(15, options_.auto_retransmit_count));

  SelectRfChannel(options_.initial_channel);

  VerifyRegister(
      0x06,  // RF_SETUP
      [&]() {
        if (options_.data_rate == 250000) {
          return (1 << 5);
        } else if (options_.data_rate == 1000000) {
          return (0 << 5) | (0 << 3);
        } else if (options_.data_rate == 2000000) {
          return (0 << 5) | (1 << 3);
        }
        // default to 250kbps
        return (1 << 5);
      }() |
      [&]() {
        if (options_.output_power == -18) {
          return 0;
        } else if (options_.output_power == -12) {
          return 2;
        } else if (options_.output_power == -6) {
          return 4;
        } else if (options_.output_power == 0) {
          return 6;
        }
        // default to 0dB output power
        return 6;
      }());

  uint8_t id_buf[5] = {};
  for (int i = 0; i < options_.address_length; i++) {
    id_buf[i] = (options_.id >> (i * 8)) & 0xff;
  }
  std::string_view id_view{
    reinterpret_cast<const char*>(&id_buf[0]),
        static_cast<size_t>(options_.address_length)};
  VerifyRegister(0x0a,  id_view); // RX_ADDR_P0
  VerifyRegister(0x10,  id_view); // TX_ADDR
  VerifyRegister(
      0x1c,
      (options_.dynamic_payload_length  ||
       options_.automatic_acknowledgment) ? 1 : 0);  // DYNPD
  VerifyRegister(
      0x1d,  0 // FEATURE
      | (((options_.dynamic_payload_length ||
           options_.automatic_acknowledgment) ? 1 : 0) << 2) // EN_DPL
      | ((options_.automatic_acknowledgment ? 1 : 0) << 1) // EN_ACK_PAY
      | ((options_.automatic_acknowledgment ? 1 : 0 ) << 0) // EN_DYN_ACK
  );

  // In read mode, we leave CE high.
  if (options_.ptx == 0) {
    ce_.write(1);
  }
}

uint8_t Nrf24l01::GetConfig() const {
  return 0
      | (0 << 6) // MASK_RX_DR - enable RX_DR interrupt
      | (0 << 5) // MASK_TX_DS - enable TX_DS interrupt
      | (0 << 4) // MASK_MAX_RT - enable MAX_RT interrupt
      | ((options_.enable_crc ? 1 : 0) << 3) // EN_CRC
      | (((options_.crc_length == 2) ? 1 : 0) << 2) // CRCO (0=1 byte, 1=2 bytes)
      | (1 << 1) // PWR_UP
      | ((options_.ptx ? 0 : 1) << 0) // PRIM_RX
      ;
}

Nrf24l01::Status Nrf24l01::status() {
  Status result;
  result.status_reg = nrf_.Command(0xff, {}, {});
  result.retransmit_exceeded = retransmit_exceeded_;
  return result;
}

uint8_t Nrf24l01::ReadRegister(uint8_t reg) {
  return nrf_.ReadRegister(reg);
}

void Nrf24l01::ReadRegister(uint8_t reg, mjlib::base::string_span buffer) {
  nrf_.ReadRegister(reg, buffer);
}

void Nrf24l01::WriteRegister(uint8_t reg, std::string_view buffer) {
  nrf_.WriteRegister(reg, buffer);
}

}  // namespace fw
