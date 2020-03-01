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

#include <string_view>

#include "mbed.h"

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
        gyro_int_{options.gyro_int} {
    // Before the accelerometer can be be initialized, we have to wait
    // 1ms after power on.  We'll just assume our timer hasn't wrapped
    // around and we can use the timer as a per-boot clock.  That way,
    // if we are constructed after power on, we won't pay this
    // penalty.
    wait_until_us(timer, 1000);

    // First do the accelerometer.
    ConfigureAccelerometer();

    // Then the gyro.
    ConfigureGyro();
  }

  Data data() {
    Data result;
    result.acc_int = acc_int_.read();
    result.gyro_int = gyro_int_.read();

    uint8_t buf[6] = {};
    acc_.Read(0x80 | 0x12,  // ACC_X_LSB
              {reinterpret_cast<char*>(buf), sizeof(buf)});

    result.accelx = static_cast<int16_t>(buf[1] * 256 + buf[0]);
    result.accely = static_cast<int16_t>(buf[3] * 256 + buf[2]);
    result.accelz = static_cast<int16_t>(buf[5] * 256 + buf[4]);

    gyro_.Read(0x80 | 0x02,  // RATE_X_LSB
               {reinterpret_cast<char*>(buf), sizeof(buf)});

    result.gyrox = static_cast<int16_t>(buf[1] * 256 + buf[0]);
    result.gyroy = static_cast<int16_t>(buf[3] * 256 + buf[2]);
    result.gyroz = static_cast<int16_t>(buf[5] * 256 + buf[4]);

    return result;
  }

  void ConfigureAccelerometer() {
    // Switch the accelerometer into SPI mode.  Any CS toggle will do
    // this, we'll just make a dummy read.
    acc_.ReadScalar<uint8_t>(0);

    // Power on the sensor.
    acc_.WriteScalar<uint8_t>(0x7d, 0x04);  // ACC_PWR_CTRL - o n

    // Wait long enough for it to power up.
    timer_->wait_us(50000);

    // Get the chip id.
    setup_data_.acc_id = acc_.ReadScalar<uint8_t>(0x80);

    // Configure interrupts.
    acc_.WriteScalar<uint8_t>(0x53, 0x08);  // INT1_IO_CONF - INT1 as output
    acc_.WriteScalar<uint8_t>(0x58, 0x04);  // INT1_INT2_DATA_MAP - drd -> int1

    acc_.WriteScalar(
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
    acc_.WriteScalar(
        0x41,  // ACC_RANGE
        [&]() {
          if (options_.accel_max_g <= 3) { return 0x00; }
          if (options_.accel_max_g <= 6) { return 0x01; }
          if (options_.accel_max_g <= 12) { return 0x02; }
          return 0x03;  // 24g
        }());
  }

  void ConfigureGyro() {
    setup_data_.gyro_id = gyro_.ReadScalar<uint8_t>(0x80);

    gyro_.WriteScalar<uint8_t>(0x15, 0x80);  // GYRO_INT_CTRL - enable interrupts
    gyro_.WriteScalar<uint8_t>(0x16, 0x00);  // push pull active high
    gyro_.WriteScalar<uint8_t>(0x18, 0x01);  // data ready mapped to int3

    gyro_.WriteScalar<uint8_t>(
        0x0f,  // GYRO_RANGE
        [&]() {
          if (options_.gyro_max_dps <= 125) { return 0x04; }
          if (options_.gyro_max_dps <= 250) { return 0x03; }
          if (options_.gyro_max_dps <= 500) { return 0x02; }
          if (options_.gyro_max_dps <= 1000) { return 0x01; }
          return 0x00;  // 2000 dps
        }());
    gyro_.WriteScalar<uint8_t>(
        0x10,  // GYRO_BANDWIDTH
        [&]() {
          if (options_.rate_hz <= 100) { return 0x07; }
          if (options_.rate_hz <= 200) { return 0x06; }
          // We don't support the lower filter bandwidth versions of
          // 100 and 200Hz ODR.
          if (options_.rate_hz <= 400) { return 0x03; }
          if (options_.rate_hz <= 1000) { return 0x02; }
          return 0x00;  // 2000 dps 532Hz BW
        }());
  }

  MillisecondTimer* const timer_;
  const Options options_;
  SPI spi_;
  SpiMaster acc_;
  SpiMaster gyro_;
  DigitalIn acc_int_;
  DigitalIn gyro_int_;

  SetupData setup_data_;
};

Bmi088::Bmi088(mjlib::micro::Pool* pool, MillisecondTimer* timer,
               const Options& options)
    : impl_(pool, timer, options) {}

Bmi088::~Bmi088() {}

const Bmi088::SetupData& Bmi088::setup_data() const {
  return impl_->setup_data_;
}

Bmi088::Data Bmi088::data() {
  return impl_->data();
}

}
