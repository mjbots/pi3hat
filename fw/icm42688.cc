// Copyright 2020-2021 Josh Pieper, jjp@pobox.com.
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

#include "fw/icm42688.h"

#include <atomic>
#include <utility>
#include <string_view>

#include "mbed.h"

#include "mjlib/base/assert.h"
#include "mjlib/base/string_span.h"

#include "fw/stm32_pwm_out.h"

namespace fw {

namespace {
const uint8_t ICM_INT_CONFIG = 0x14;
const uint8_t ICM_ACCEL_DATA_X1 = 0x1f;
const uint8_t ICM_ACCEL_DATA_X0 = 0x20;
const uint8_t ICM_ACCEL_DATA_Y1 = 0x21;
const uint8_t ICM_ACCEL_DATA_Y0 = 0x22;
const uint8_t ICM_ACCEL_DATA_Z1 = 0x23;
const uint8_t ICM_ACCEL_DATA_Z0 = 0x24;
const uint8_t ICM_GYRO_DATA_X1 = 0x25;
const uint8_t ICM_GYRO_DATA_X0 = 0x26;
const uint8_t ICM_GYRO_DATA_Y1 = 0x27;
const uint8_t ICM_GYRO_DATA_Y0 = 0x28;
const uint8_t ICM_GYRO_DATA_Z1 = 0x29;
const uint8_t ICM_GYRO_DATA_Z0 = 0x2a;
const uint8_t ICM_INTF_CONFIG0 = 0x4c;
const uint8_t ICM_INTF_CONFIG1 = 0x4d;
const uint8_t ICM_PWR_MGMT0 = 0x4e;
const uint8_t ICM_GYRO_CONFIG0 = 0x4f;
const uint8_t ICM_ACCEL_CONFIG0 = 0x50;
const uint8_t ICM_INT_CONFIG0 = 0x63;
const uint8_t ICM_INT_SOURCE0 = 0x65;
const uint8_t ICM_REG_BANK_SEL = 0x76;
const uint8_t ICM_WHOAMI = 0x75;

// Bank 1

const uint8_t ICM_INTF_CONFIG5 = 0x7b;

}

namespace {
void wait_until_us(MillisecondTimer* timer, uint32_t absolute_us) {
  const auto now = timer->read_us();
  if (now > absolute_us) { return; }
  timer->wait_us(absolute_us - now);
}

class SpiMaster {
 public:
  struct Options {
  };
  SpiMaster(SPI* spi, PinName cs, MillisecondTimer* timer, const Options& options)
      : spi_(spi), cs_(cs, 1), timer_(timer), options_(options) {
  }

  void Write(uint8_t address, std::string_view data) {
    cs_.write(0);
    // The ICM-42688 datasheet documents a 39ns CS setup time on the
    // chip select.  That's only about 8 instructions at the maximum
    // clock rate for any STM32 device.
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");

    spi_->write(address);
    for (auto c : data) {
      spi_->write(c);
    }

    timer_->wait_us(1);

    cs_.write(1);
  }

  void Read(uint8_t address, mjlib::base::string_span data) {
    cs_.write(0);

    // Our 8 cycle setup time.
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");

    spi_->write(0x80 | address);
    for (auto& c : data) {
      c = spi_->write(0);
    }
    cs_.write(1);
  }

  template <typename T>
  T ReadScalar(uint8_t address) {
    T value = {};
    Read(address, mjlib::base::string_span(
             reinterpret_cast<char*>(&value), sizeof(value)));
    return value;
  }

  template <typename T>
  void WriteScalar(uint8_t address, T value) {
    Write(address, std::string_view(
              reinterpret_cast<const char*>(&value), sizeof(value)));
  }

  template <typename T>
  void VerifyScalar(uint8_t address, T value) {
    WriteScalar(address, value);
    const T result = ReadScalar<T>(0x80 | address);
    MJ_ASSERT(result == value);
  }

 private:
  SPI* const spi_;
  DigitalOut cs_;
  MillisecondTimer* const timer_;
  const Options options_;
};
}

class Icm42688::Impl {
 public:
  Impl(MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        options_(options),
        clkin_(options.clkin, [&]() {
          Stm32PwmOut::Options pwm_opt;
          pwm_opt.prescaler = 1;
          pwm_opt.period = 2 * HAL_RCC_GetPCLK1Freq() / 32000;
          pwm_opt.on_counts = pwm_opt.period / 2;
          return pwm_opt;
        }()),
        spi_{options.mosi, options.miso, options.sck},
        master_{&spi_, options.cs, timer, {}},
        irq_{options.irq, PullUp} {
    spi_.frequency(5000000);

    // Lets give the part some time after power-on to get ready.  We
    // use this formulation to avoid having to wait for
    // re-initializations after boot.  This assumes that the counter
    // hasn't wrapped, but if it has, we'll pay a bit of a penalty.
    wait_until_us(timer, 1000);

    // Start out in bank 0 and turn off the sensors.
    master_.WriteScalar<uint8_t>(ICM_REG_BANK_SEL, 0);
    master_.WriteScalar<uint8_t>(ICM_PWR_MGMT0, 0x00);

    // Give things a bit to turn off.
    timer_->wait_us(20);

    setup_data_.whoami = master_.ReadScalar<uint8_t>(ICM_WHOAMI);
    if (setup_data_.whoami != 0x47) {
      setup_data_.error = 1;
      return;
    }

    const uint8_t odr = [&]() {
      if (options_.rate_hz <= 13) { return 11; }
      if (options_.rate_hz <= 25) { return 10; }
      if (options_.rate_hz <= 50) { return 9; }
      if (options_.rate_hz <= 100) { return 8; }
      if (options_.rate_hz <= 200) { return 7; }
      if (options_.rate_hz <= 1000) { return 6; }
      if (options_.rate_hz <= 2000) { return 5; }
      if (options_.rate_hz <= 4000) { return 4; }
      if (options_.rate_hz <= 8000) { return 3; }
      if (options_.rate_hz <= 16000) { return 2; }
      return 1;  // 32000
    }();

    const uint8_t gyro_range =
        [&]() {
          if (options_.gyro_max_dps <= 16) { return 7; }
          if (options_.gyro_max_dps <= 32) { return 6; }
          if (options_.gyro_max_dps <= 63) { return 5; }
          if (options_.gyro_max_dps <= 125) { return 4; }
          if (options_.gyro_max_dps <= 250) { return 3; }
          if (options_.gyro_max_dps <= 500) { return 2; }
          if (options_.gyro_max_dps <= 1000) { return 1; }
          return 0;  // 2000
        }();

    master_.WriteScalar<uint8_t>(ICM_GYRO_CONFIG0, (gyro_range << 5) | odr);

    const uint8_t accel_range = [&]() {
      if (options_.accel_max_g <= 2) { return 3; }
      if (options_.accel_max_g <= 4) { return 2; }
      if (options_.accel_max_g <= 8) { return 1; }
      return 0;  // 16;
    }();

    master_.WriteScalar<uint8_t>(ICM_ACCEL_CONFIG0, (accel_range << 5) | odr);

    // Maybe eventually we could use the gyro/accel anti-alias filters
    // or the gyro notch filter?

    setup_data_.rate_hz = [&]() {
      if (odr == 11) { return 13; }
      if (odr == 10) { return 25; }
      if (odr == 9) { return 50; }
      if (odr == 8) { return 100; }
      if (odr == 7) { return 200; }
      if (odr == 6) { return 1000; }
      if (odr == 5) { return 2000; }
      if (odr == 4) { return 4000; }
      if (odr == 3) { return 8000; }
      if (odr == 2) { return 16000; }
      if (odr == 1) { return 32000; }
      return 32000;
    }();

    setup_data_.gyro_max_dps = [&]() {
      if (gyro_range == 0) { return 2000; }
      if (gyro_range == 1) { return 1000; }
      if (gyro_range == 2) { return 500; }
      if (gyro_range == 3) { return 250; }
      if (gyro_range == 4) { return 125; }
      if (gyro_range == 5) { return 63; }
      if (gyro_range == 6) { return 32; }
      if (gyro_range == 7) { return 16; }
      return 2000;
    }();

    setup_data_.accel_max_g = [&]() {
      if (accel_range == 0) { return 16; }
      if (accel_range == 1) { return 8; }
      if (accel_range == 2) { return 4; }
      if (accel_range == 3) { return 2; }
      return 1;
    }();

    constexpr float kG = 9.81;

    accel_scale_ = kG * static_cast<float>(setup_data_.accel_max_g) / 32767.0f;
    gyro_scale_ = static_cast<float>(setup_data_.gyro_max_dps) / 32767.0f;

    // Configure our interrupts.

    // UI_DRDY_INT_CLEAR = 2 (clear on sensor register read)
    master_.WriteScalar<uint8_t>(ICM_INT_CONFIG0, 0x20);

    // UI_DRDY_INT1_EN = 1 (data ready routed to INT1)
    master_.WriteScalar<uint8_t>(ICM_INT_SOURCE0, 0x08);

    // INT1_MODE = 1 (latched mode)
    // INT1_DRIVE_CIRCUIT = 0 (open drain)
    // INT1_POLARITY = 0 (active low)
    master_.WriteScalar<uint8_t>(ICM_INT_CONFIG, 0x04);


    // Configure our clocks.

    // ACCEL_LP_CLK_SEL = 1
    // RTC_MODE = 1
    // CLKSEL = 1 (select PLL when available, else RC)
    master_.WriteScalar<uint8_t>(ICM_INTF_CONFIG1, 0x95);

    ////////////////////
    // Move to bank 1.
    master_.WriteScalar<uint8_t>(ICM_REG_BANK_SEL, 1);

    // PIN9_FUNCTION = 2 (CLKIN)
    master_.WriteScalar<uint8_t>(ICM_INTF_CONFIG5, 0x04);

    ////////////////////
    // And finally back to bank 0 to turn things on.
    master_.WriteScalar<uint8_t>(ICM_REG_BANK_SEL, 0);


    // Place both gyro and accel in low noise mode.
    master_.WriteScalar<uint8_t>(ICM_PWR_MGMT0, 0x0f);

    // When transitioning, we must not issue any writes for 200us.
    // Just don't do anything for that long to be sure.
    timer_->wait_us(200);
  }

  Data read_data() {
    Data result;

    if (setup_data_.error) {
      return result;
    }

    last_read_ = timer_->read_us();

    uint8_t buf[12] = {};
    master_.Read(ICM_ACCEL_DATA_X1,
                 {reinterpret_cast<char*>(buf), sizeof(buf)});

    auto read_int16 = [&](int offset) -> int16_t {
      const auto u16 = static_cast<uint16_t>(
          buf[offset] * 256 + buf[offset + 1]);
      return u16 > 32767 ? (u16 - 65536) : u16;
    };

    result.accel_mps2.x() = read_int16(0) * accel_scale_;
    result.accel_mps2.y() = read_int16(2) * accel_scale_;
    result.accel_mps2.z() = read_int16(4) * accel_scale_;

    result.rate_dps.x() = read_int16(6) * gyro_scale_;
    result.rate_dps.y() = read_int16(8) * gyro_scale_;
    result.rate_dps.z() = read_int16(10) * gyro_scale_;

    return result;
  }

  MillisecondTimer* const timer_;
  const Options options_;
  Stm32PwmOut clkin_;
  SPI spi_;
  SpiMaster master_;
  DigitalIn irq_;

  SetupData setup_data_;
  float accel_scale_ = 0.0f;
  float gyro_scale_ = 0.0f;

  uint32_t last_read_ = 0;
};

Icm42688::Icm42688(MillisecondTimer* timer, const Options& options)
    : impl_(timer, options) {}

Icm42688::~Icm42688() {}

const Icm42688::SetupData& Icm42688::setup_data() const {
  return impl_->setup_data_;
}

Icm42688::Data Icm42688::read_data() {
  return impl_->read_data();
}

bool Icm42688::data_ready() {
  return impl_->irq_.read() == 0;
}

}
