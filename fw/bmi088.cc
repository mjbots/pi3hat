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

#include "fw/bmi088.h"

#include <atomic>
#include <utility>
#include <string_view>

#include "mbed.h"

#include "mjlib/base/assert.h"
#include "mjlib/base/string_span.h"

namespace fw {

namespace {
void wait_until_us(MillisecondTimer* timer, uint32_t absolute_us) {
  const auto now = timer->read_us();
  if (now > absolute_us) { return; }
  timer->wait_us(absolute_us - now);
}

class SpiMaster {
 public:
  struct Options {
    bool post_address_dummy_read = false;
  };
  SpiMaster(SPI* spi, PinName cs, MillisecondTimer* timer, const Options& options)
      : spi_(spi), cs_(cs, 1), timer_(timer), options_(options) {
    // TODO: Set SPI frequency.
  }

  void Write(uint8_t address, std::string_view data) {
    cs_.write(0);
    // The BMI088 datasheet documents a 40ns CS setup time on the chip
    // select.  That's only about 8 instructions at the maximum clock
    // rate for any STM32 device.
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

    // The BMI088 datasheet says there is at least a 2us wait time
    // between writes.
    timer_->wait_us(3);
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

    spi_->write(address);
    if (options_.post_address_dummy_read) {
      spi_->write(0);
    }
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

class Bmi088::Impl {
 public:
  Impl(MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        options_(options),
        spi_{options.mosi, options.miso, options.sck},
        acc_{&spi_, options.acc_cs, timer, []() {
            SpiMaster::Options options;
            options.post_address_dummy_read = true;
            return options;
          }()},
        gyro_{&spi_, options.gyro_cs, timer, {}},
        acc_int_{options.acc_int},
        gyro_int_{options.gyro_int, PullUp} {
    spi_.frequency(5000000);
    // Before the accelerometer can be be initialized, we have to wait
    // 1ms after power on.  We'll just assume our timer hasn't wrapped
    // around and we can use the timer as a per-boot clock.  That way,
    // if we are constructed after power on, we won't pay this
    // penalty.
    wait_until_us(timer, 1000);

    if (!setup_data_.error) {
      // First do the accelerometer.
      ConfigureAccelerometer();
    }

    if (!setup_data_.error) {
      // Then the gyro.
      ConfigureGyro();
    }
  }

  Data read_data() {
    Data result;

    if (setup_data_.error) {
      return result;
    }

    last_read_ = timer_->read_us();

    uint8_t buf[6] = {};
    acc_.Read(0x80 | 0x12,  // ACC_X_LSB
              {reinterpret_cast<char*>(buf), sizeof(buf)});

    auto read_int16 = [&](int offset) -> int16_t {
      const auto u16 = static_cast<uint16_t>(
          buf[offset + 1] * 256 + buf[offset]);
      return u16 > 32767 ? (u16 - 65536) : u16;
    };

    result.accel_mps2.x() = read_int16(0) * accel_scale_;
    result.accel_mps2.y() = read_int16(2) * accel_scale_;
    result.accel_mps2.z() = read_int16(4) * accel_scale_;

    std::memset(&buf[0], 0, sizeof(buf));

    gyro_.Read(0x80 | 0x02,  // RATE_X_LSB
               {reinterpret_cast<char*>(buf), sizeof(buf)});

    result.rate_dps.x() = read_int16(0) * gyro_scale_;
    result.rate_dps.y() = read_int16(2) * gyro_scale_;
    result.rate_dps.z() = read_int16(4) * gyro_scale_;

    return result;
  }

  void ConfigureAccelerometer() {
    // Switch the accelerometer into SPI mode.  Any CS toggle will do
    // this, we'll just make a dummy read.
    acc_.ReadScalar<uint8_t>(0x80);

    // Verify we have an actual accelerometer present.
    setup_data_.whoami = acc_.ReadScalar<uint8_t>(0x80);
    if (setup_data_.whoami != 0x1e &&  // BMI088
        setup_data_.whoami != 0x1a) {   // BMI090L
      setup_data_.error = 1;
      return;
    }

    // Power on the sensor.
    acc_.WriteScalar<uint8_t>(0x7d, 0x04);  // ACC_PWR_CTRL - o n

    // Wait long enough for it to power up.
    timer_->wait_us(50000);

    // Configure interrupts.
    acc_.VerifyScalar<uint8_t>(0x53, 0x08);  // INT1_IO_CONF - INT1 as output
    acc_.VerifyScalar<uint8_t>(0x58, 0x04);  // INT1_INT2_DATA_MAP - drd -> int1

    acc_.VerifyScalar(
        0x40,  // ACC_CONF
        (0x0a << 4) | // acc_bwp - normal bandwidth
        [&]() {
          if (options_.rate_hz <= 13) { return 0x05; }
          if (options_.rate_hz <= 25) { return 0x06; }
          if (options_.rate_hz <= 50) { return 0x07; }
          if (options_.rate_hz <= 100) { return 0x08; }
          if (options_.rate_hz <= 200) { return 0x09; }
          if (options_.rate_hz <= 400) { return 0x0a; }
          if (options_.rate_hz <= 800) { return 0x0b; }
          return 0x0c;  // 1600
        }());

    constexpr float kG = 9.81;
    auto [accel_range_conf, accel_scale] = [&]() {
      if (options_.accel_max_g <= 3) {
        return std::make_pair(0x00, kG / 10920.f);
      }
      if (options_.accel_max_g <= 6) {
        return std::make_pair(0x01, kG / 5460.f);
      }
      if (options_.accel_max_g <= 12) {
        return std::make_pair(0x02, kG / 2730.f);
      }
      return std::make_pair(0x03, kG / 1365.f);  // 24g
    }();
    // ACC_RANGE
    acc_.VerifyScalar(0x41, accel_range_conf);
    accel_scale_ = accel_scale;
  }

  void ConfigureGyro() {
    setup_data_.gyro_id = gyro_.ReadScalar<uint8_t>(0x80);
    if (setup_data_.gyro_id != 0x0f) {
      setup_data_.error = 2;
      return;
    }

    gyro_.VerifyScalar<uint8_t>(0x15, 0x80);  // GYRO_INT_CTRL - enable interrupts
    gyro_.VerifyScalar<uint8_t>(0x16, 0x00);  // push pull active high
    gyro_.VerifyScalar<uint8_t>(0x18, 0x01);  // data ready mapped to int3

    auto [gyro_conf, gyro_scale] = [&]() {
      if (options_.gyro_max_dps <= 125) {
        return std::make_pair(0x04, 0.0038f);
      }
      if (options_.gyro_max_dps <= 250) {
        return std::make_pair(0x03, 0.0076f);
      }
      if (options_.gyro_max_dps <= 500) {
        return std::make_pair(0x02, 0.0153f);
      }
      if (options_.gyro_max_dps <= 1000) {
        return std::make_pair(0x01, 0.0305f);
      }
      return std::make_pair(0x00, 0.061f);
    }();

    // GYRO_RANGE
    gyro_.VerifyScalar<uint8_t>(0x0f, gyro_conf);
    gyro_scale_ = gyro_scale;

    auto [rate_bits, rate_hz] = [&]() -> std::pair<uint8_t, uint16_t> {
      if (options_.rate_hz <= 100) { return {0x07, 100}; }
      if (options_.rate_hz <= 200) { return {0x06, 200}; }
      // We don't support the lower filter bandwidth versions of
      // 100 and 200Hz ODR.
      if (options_.rate_hz <= 400) { return {0x03, 400}; }
      if (options_.rate_hz <= 1000) { return {0x02, 1000}; }
      return {0x00, 2000};  // 2000 dps 532Hz BW
    }();

    setup_data_.rate_hz = rate_hz;

    gyro_.VerifyScalar<uint8_t>(
        0x10,  // GYRO_BANDWIDTH
        0x80 | rate_bits);
  }

  MillisecondTimer* const timer_;
  const Options options_;
  SPI spi_;
  SpiMaster acc_;
  SpiMaster gyro_;
  DigitalIn acc_int_;

  DigitalIn gyro_int_;

  SetupData setup_data_;
  float accel_scale_ = 0.0f;
  float gyro_scale_ = 0.0f;

  uint32_t last_read_ = 0;
};

Bmi088::Bmi088(MillisecondTimer* timer, const Options& options)
    : impl_(timer, options) {}

Bmi088::~Bmi088() {}

const Bmi088::SetupData& Bmi088::setup_data() const {
  return impl_->setup_data_;
}

Bmi088::Data Bmi088::read_data() {
  return impl_->read_data();
}

}
