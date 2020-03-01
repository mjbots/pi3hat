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

namespace {
class SpiMaster {
 public:
  SpiMaster(SPI* spi, PinName cs, MillisecondTimer* timer)
      : spi_(spi), cs_(cs, 1), timer_(timer) {
  }

  uint8_t Command(uint8_t command,
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

  uint8_t Write(uint8_t address, std::string_view data) {
    return Command(0x20 + address, data, {});
  }

  uint8_t Read(uint8_t address, mjlib::base::string_span data) {
    return Command(0x00 + address, {}, data);
  }

  template <typename T>
  T ReadScalar(uint8_t address) {
    T result = {};
    Read(address, {reinterpret_cast<char*>(&result), sizeof(result)});
    return result;
  }

 private:
  SPI* const spi_;
  DigitalOut cs_;
  MillisecondTimer* const timer_;
};

}

class Nrf24l01::Impl {
 public:
  Impl(MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        options_(options),
        spi_{options.mosi, options.miso, options.sck},
        nrf_{&spi_, options.cs, timer} {
  }

  void PollMillisecond() {
    // The NRF isn't turned on for 100ms after power up.
    if (!power_on_ && timer_->read_ms() < 100) { return; }

    if (!power_on_) {
      power_on_ = true;

      for (size_t i = 0; i < sizeof(register_map_); i++) {
        register_map_[i] = nrf_.ReadScalar<uint8_t>(i);
      }
    }
  }

  std::string_view register_map() const {
    return {register_map_, sizeof(register_map_)};
  }

 private:
  MillisecondTimer* const timer_;
  const Options options_;

  SPI spi_;
  SpiMaster nrf_;

  bool power_on_ = false;
  char register_map_[32] = {};
};

Nrf24l01::Nrf24l01(mjlib::micro::Pool* pool, MillisecondTimer* timer,
                   const Options& options)
    : impl_(pool, timer, options) {}

Nrf24l01::~Nrf24l01() {}

void Nrf24l01::PollMillisecond() {
  impl_->PollMillisecond();
}

std::string_view Nrf24l01::register_map() const {
  return impl_->register_map();
}

}
