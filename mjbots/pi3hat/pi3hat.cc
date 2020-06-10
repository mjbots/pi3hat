// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

// We purposefully don't use the full path here so that this file can
// be compiled in a wide range of build configurations.
#include "pi3hat.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <string>
#include <vector>

char g_data_block[4096] = {};

void copy_data(void* ptr) {
  std::memcpy(g_data_block, ptr, sizeof(g_data_block));
}

namespace mjbots {
namespace pi3hat {

namespace {
///////////////////////////////////////////////
/// Random utility functions

void write_u8(volatile uint32_t& address, uint8_t value) {
  volatile uint8_t* u8_addr = reinterpret_cast<volatile uint8_t*>(&address);
  *u8_addr = value;
}

uint8_t read_u8(volatile uint32_t& address) {
  volatile uint8_t* u8_addr = reinterpret_cast<volatile uint8_t*>(&address);
  return *u8_addr;
}

size_t RoundUpDlc(size_t value) {
  if (value == 0) { return 0; }
  if (value == 1) { return 1; }
  if (value == 2) { return 2; }
  if (value == 3) { return 3; }
  if (value == 4) { return 4; }
  if (value == 5) { return 5; }
  if (value == 6) { return 6; }
  if (value == 7) { return 7; }
  if (value == 8) { return 8; }
  if (value <= 12) { return 12; }
  if (value <= 16) { return 16; }
  if (value <= 20) { return 20; }
  if (value <= 24) { return 24; }
  if (value <= 32) { return 32; }
  if (value <= 48) { return 48; }
  if (value <= 64) { return 64; }
  return 0;
}

char g_format_buf[2048] = {};

const char* Format(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

const char* Format(const char* fmt, ...) {
  va_list args1;
  va_start(args1, fmt);
  ::vsnprintf(g_format_buf, sizeof(g_format_buf) - 1, fmt, args1);
  va_end(args1);
  return &g_format_buf[0];
}

template <typename ErrorGenerator>
void ThrowIf(bool value, ErrorGenerator error_generator) {
  if (!value) { return; }
  throw Error(error_generator());
}

char g_error_buf[2048] = {};

void ThrowIfErrno(bool value, const std::string& message = "") {
  if (!value) { return; }

  // Just to be on the safe side.
  g_error_buf[0] = 0;
  const auto result = strerror_r(errno, g_error_buf, sizeof(g_error_buf));
  // For portability.
  (void)result;

  throw Error(message + " : " + std::string(g_error_buf));
}

int64_t GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000ll +
      static_cast<int64_t>(ts.tv_nsec);
}

void BusyWaitUs(int64_t us) {
  const auto start = GetNow();
  const auto end = start + us * 1000;
  while (GetNow() <= end);
}

///////////////////////////////////////////////
/// Random utility classes

/// Manages ownership of a system file descriptor.
class SystemFd {
 public:
  SystemFd() : fd_(-1) {}
  SystemFd(int fd) : fd_(fd) {}

  SystemFd(SystemFd&& rhs) {
    fd_ = rhs.fd_;
    rhs.fd_ = -1;
  }

  SystemFd& operator=(SystemFd&& rhs) {
    fd_ = rhs.fd_;
    rhs.fd_ = -1;
    return *this;
  }

  ~SystemFd() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  SystemFd(const SystemFd&) = delete;
  SystemFd& operator=(const SystemFd&) = delete;

  operator int() { return fd_; }

 private:
  int fd_ = -1;
};

/// Manages ownership of an mmap'ed region of a given file descriptor.
class SystemMmap {
 public:
  SystemMmap() {}

  SystemMmap(int fd, size_t size, uint64_t offset) {
    ptr_ = ::mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    size_ = size;
    ThrowIfErrno(ptr_ == MAP_FAILED);
  }

  ~SystemMmap() {
    if (ptr_ != MAP_FAILED) {
      ThrowIfErrno(::munmap(ptr_, size_) < 0);
    }
  }

  SystemMmap(SystemMmap&& rhs) {
    std::swap(ptr_, rhs.ptr_);
    std::swap(size_, rhs.size_);
  }

  SystemMmap& operator=(SystemMmap&& rhs) {
    std::swap(ptr_, rhs.ptr_);
    std::swap(size_, rhs.size_);
    return *this;
  }

  SystemMmap(const SystemMmap&) = delete;
  SystemMmap& operator=(const SystemMmap&) = delete;

  void* ptr() { return ptr_; }

  // Since this is intended to be whatever, we just allow it to be
  // converted to any old pointer at will without extra hoops.
  template <typename T>
  operator T*() { return ptr_; }

  template <typename T>
  operator const T*() const { return ptr_; }

 private:
  void* ptr_ = MAP_FAILED;
  size_t size_ = 0;
};


///////////////////////////////////////////////
/// Drivers for the Raspberry Pi hardware

class Rpi3Gpio {
 public:
  static constexpr uint32_t RASPI_23_PERI_BASE = 0x3F000000;
  static constexpr uint32_t GPIO_BASE          = 0x00200000;

  // static constexpr uint32_t INPUT = 0;
  static constexpr uint32_t OUTPUT = 1;
  static constexpr uint32_t ALT_0 = 4;
  // static constexpr uint32_t ALT_1 = 5;
  // static constexpr uint32_t ALT_2 = 6;
  // static constexpr uint32_t ALT_3 = 7;
  static constexpr uint32_t ALT_4 = 3;
  // static constexpr uint32_t ALT_5 = 2;

  Rpi3Gpio(int dev_mem_fd)
      : mmap_(dev_mem_fd, 4096, RASPI_23_PERI_BASE + GPIO_BASE),
        gpio_(reinterpret_cast<volatile uint32_t*>(mmap_.ptr())) {}

  void SetGpioMode(uint32_t gpio, uint32_t function) {
    uint32_t reg_offset = gpio / 10;
    uint32_t bit = (gpio % 10) * 3;
    const auto value = gpio_[reg_offset];
    gpio_[reg_offset] = (value & ~(0x7 << bit)) | ((function & 0x7) << bit);
  }

  void SetGpioOutput(uint32_t gpio, bool value) {
    if (value) {
      const uint32_t reg_offset = gpio / 32 + 7;
      gpio_[reg_offset] = 1 << (gpio % 32);
    } else {
      const uint32_t reg_offset = gpio / 32 + 10;
      gpio_[reg_offset] = 1 << (gpio % 32);
    }
  }

  volatile uint32_t& operator[](int index) { return gpio_[index]; }
  const volatile uint32_t& operator[](int index) const { return gpio_[index]; }

  class ActiveLow {
   public:
    ActiveLow(Rpi3Gpio* parent, uint32_t gpio) : parent_(parent), gpio_(gpio) {
      parent_->SetGpioOutput(gpio_, false);
    }

    ~ActiveLow() {
      parent_->SetGpioOutput(gpio_, true);
    }

   private:
    Rpi3Gpio* const parent_;
    const uint32_t gpio_;
  };

 private:
  SystemMmap mmap_;
  volatile uint32_t* const gpio_;
};


constexpr uint32_t RASPI_23_PERI_BASE = 0x3F000000;
constexpr uint32_t kSpi0CS0 = 8;
constexpr uint32_t kSpi0CS1 = 7;
constexpr uint32_t kSpi0CS[] = {kSpi0CS0, kSpi0CS1};

constexpr uint32_t SPI_BASE = 0x204000;
constexpr uint32_t SPI_CS_TA = 1 << 7;
constexpr uint32_t SPI_CS_DONE = 1 << 16;
constexpr uint32_t SPI_CS_RXD = 1 << 17;
constexpr uint32_t SPI_CS_TXD = 1 << 18;


/// This class interacts with the SPI0 device on a raspberry pi using
/// the BCM2835/6/7's registers directly.  The kernel driver must not
/// be active (it can be loaded, as long as you're not using it), and
/// this must be run as root or otherwise have access to /dev/mem.
class PrimarySpi {
 public:
  struct Options {
    int speed_hz = 10000000;
    int cs_hold_us = 4;
    int address_hold_us = 8;

    Options() {}
  };

  PrimarySpi(const Options& options = Options()) {
    fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
    ThrowIfErrno(fd_ < 0, "pi3hat: could not open /dev/mem");

    spi_mmap_ = SystemMmap(fd_, 4096, RASPI_23_PERI_BASE + SPI_BASE);
    spi_ = reinterpret_cast<volatile Bcm2835Spi*>(
        static_cast<char*>(spi_mmap_.ptr()));

    gpio_ = std::make_unique<Rpi3Gpio>(fd_);

    gpio_->SetGpioOutput(kSpi0CS0, true);
    gpio_->SetGpioOutput(kSpi0CS1, true);

    gpio_->SetGpioMode(kSpi0CS0, Rpi3Gpio::OUTPUT); // We'll do CS in SW
    gpio_->SetGpioMode(kSpi0CS1, Rpi3Gpio::OUTPUT);
    gpio_->SetGpioMode(9, Rpi3Gpio::ALT_0);
    gpio_->SetGpioMode(10, Rpi3Gpio::ALT_0);
    gpio_->SetGpioMode(11, Rpi3Gpio::ALT_0);

    spi_->cs = (
        0
        | (0 << 25) // LEn_LONG
        | (0 << 24) // DMA_LEN
        | (0 << 23) // CSPOL2
        | (0 << 22) // CSPOL1
        | (0 << 21) // CSPOL0
        | (0 << 13) // LEN
        | (0 << 12) // REN
        | (0 << 11) // ADCS
        | (0 << 10) // INTR
        | (0 << 9) // INTD
        | (0 << 8) // DMAEN
        | (0 << 7) // TA
        | (0 << 6) // CSPOL
        | (3 << 4) // CLEAR
        | (0 << 3) // CPOL
        | (0 << 2) // CPHA
        | (0 << 0) // CS
                );

    // Configure the SPI peripheral.
    const int clkdiv =
        std::max(0, std::min(65535, 400000000 / options.speed_hz));
    spi_->clk = clkdiv;
  }

  ~PrimarySpi() {}

  PrimarySpi(const PrimarySpi&) = delete;
  PrimarySpi& operator=(const PrimarySpi&) = delete;

  void Write(int cs, int address, const char* data, size_t size) {
    BusyWaitUs(options_.cs_hold_us);
    Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi0CS[cs]);
    BusyWaitUs(options_.cs_hold_us);

    spi_->cs |= SPI_CS_TA;

    write_u8(spi_->fifo, (address & 0x00ff));

    while ((spi_->cs & SPI_CS_DONE) == 0) {
      if (spi_->cs & SPI_CS_RXD) {
        (void) spi_->fifo;
      }
    }

    spi_->cs |= SPI_CS_DONE;

    if (size != 0) {
      // Wait our address hold time.
      BusyWaitUs(options_.address_hold_us);

      size_t offset = 0;
      while (offset < size) {
        while ((spi_->cs & SPI_CS_TXD) == 0);
        write_u8(spi_->fifo, data[offset]);
        offset++;
      }

      // Wait until we are no longer busy.
      while ((spi_->cs & SPI_CS_DONE) == 0) {
        if (spi_->cs & SPI_CS_RXD) {
          (void) spi_->fifo;
        }
      }
    }

    spi_->cs &= ~SPI_CS_TA;
  }

  void Read(int cs, int address, char* data, size_t size) {
    BusyWaitUs(options_.cs_hold_us);
    Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi0CS[cs]);
    BusyWaitUs(options_.cs_hold_us);

    spi_->cs |= SPI_CS_TA;

    write_u8(spi_->fifo, (address & 0x00ff));

    while ((spi_->cs & SPI_CS_DONE) == 0) {
      if (spi_->cs & SPI_CS_RXD) {
        (void) spi_->fifo;
      }
    }

    if (size != 0) {
      // Wait our address hold time.
      BusyWaitUs(options_.address_hold_us);

      // Discard the rx fifo.
      while (spi_->cs & SPI_CS_RXD) {
        (void) spi_->fifo;
      }

      // Now we write out dummy values, reading values in.
      std::size_t remaining_read = size;
      std::size_t remaining_write = remaining_read;
      char* ptr = data;
      while (remaining_read) {
        // Make sure we don't write more than we have read spots remaining
        // so that we can never overflow the RX fifo.
        const bool can_write = (remaining_read - remaining_write) < 16;
        if (can_write &&
            remaining_write && (spi_->cs & SPI_CS_TXD) != 0) {
          write_u8(spi_->fifo, 0);
          remaining_write--;
        }

        if (remaining_read && (spi_->cs & SPI_CS_RXD) != 0) {
          *ptr = read_u8(spi_->fifo);
          ptr++;
          remaining_read--;
        }
      }
    }

    spi_->cs &= ~SPI_CS_TA;
  }

 private:
  // This is the memory layout of the SPI peripheral.
  struct Bcm2835Spi {
    uint32_t cs;
    uint32_t fifo;
    uint32_t clk;
    uint32_t dlen;
    uint32_t ltoh;
    uint32_t dc;
  };

  const Options options_;
  SystemFd fd_;
  SystemMmap spi_mmap_;
  volatile Bcm2835Spi* spi_ = nullptr;

  std::unique_ptr<Rpi3Gpio> gpio_;
};

constexpr uint32_t AUX_BASE           = 0x00215000;
constexpr uint32_t kSpi1CS0 = 18;
constexpr uint32_t kSpi1CS1 = 17;
constexpr uint32_t kSpi1CS2 = 16;
constexpr uint32_t kSpi1CS[] = {
  kSpi1CS0,
  kSpi1CS1,
  kSpi1CS2,
};

constexpr int AUXSPI_STAT_TX_FULL = 1 << 10;
constexpr int AUXSPI_STAT_TX_EMPTY = 1 << 9;
constexpr int AUXSPI_STAT_RX_EMPTY = 1 << 7;
constexpr int AUXSPI_STAT_BUSY = 1 << 6;

/// This class interacts with the AUX SPI1 device on a raspberry pi
/// using the BCM2835/6/7's registers directly.  The kernel driver
/// must not be active, and this must be run as root or otherwise have
/// access to /dev/mem.
class AuxSpi {
 public:
  struct Options {
    int speed_hz = 10000000;
    int cs_hold_us = 4;
    int address_hold_us = 8;

    Options() {}
  };

  AuxSpi(const Options& options = Options()) {
    fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
    ThrowIfErrno(fd_ < 0, "rpi3_aux_spi: could not open /dev/mem");

    spi_mmap_ = SystemMmap(fd_, 4096, RASPI_23_PERI_BASE + AUX_BASE);
    auxenb_ = reinterpret_cast<volatile uint32_t*>(
        static_cast<char*>(spi_mmap_.ptr()) + 0x04);
    spi_ = reinterpret_cast<volatile Bcm2835AuxSpi*>(
        static_cast<char*>(spi_mmap_.ptr()) + 0x80);

    gpio_ = std::make_unique<Rpi3Gpio>(fd_);

    gpio_->SetGpioOutput(kSpi1CS0, true);
    gpio_->SetGpioOutput(kSpi1CS1, true);
    gpio_->SetGpioOutput(kSpi1CS2, true);

    gpio_->SetGpioMode(kSpi1CS0, Rpi3Gpio::OUTPUT); // We'll do CS in SW
    gpio_->SetGpioMode(kSpi1CS1, Rpi3Gpio::OUTPUT);
    gpio_->SetGpioMode(kSpi1CS2, Rpi3Gpio::OUTPUT);
    gpio_->SetGpioMode(19, Rpi3Gpio::ALT_4);
    gpio_->SetGpioMode(20, Rpi3Gpio::ALT_4);
    gpio_->SetGpioMode(21, Rpi3Gpio::ALT_4);

    // Start by disabling it to try and get to a known good state.
    *auxenb_ &= ~0x02;

    BusyWaitUs(10);

    // Enable the SPI peripheral.
    *auxenb_ |= 0x02;  // SPI1 enable

    spi_->cntl1 = 0;
    spi_->cntl0 = (1 << 9); // clear fifos

    // Configure the SPI peripheral.
    const int clkdiv =
        std::max(0, std::min(4095, 250000000 / 2 / options.speed_hz - 1));
    spi_->cntl0 = (
        0
        | (clkdiv << 20)
        | (7 << 17) // chip select defaults
        | (0 << 16) // post-input mode
        | (0 << 15) // variable CS
        | (1 << 14) // variable width
        | (0 << 12) // DOUT hold time
        | (1 << 11) // enable
        | (1 << 10) // in rising?
        | (0 << 9) // clear fifos
        | (0 << 8) // out rising
        | (0 << 7) // invert SPI CLK
        | (1 << 6) // MSB first
        | (0 << 0) // shift length
                   );

    spi_->cntl1 = (
        0
        | (7 << 8) // CS high time
        | (0 << 7) // tx empty IRQ
        | (0 << 6) // done IRQ
        | (1 << 1) // shift in MS first
        | (0 << 0) // keep input
                   );
  }

  ~AuxSpi() {}

  AuxSpi(const AuxSpi&) = delete;
  AuxSpi& operator=(const AuxSpi&) = delete;

  void Write(int cs, int address, const char* data, size_t size) {
    BusyWaitUs(options_.cs_hold_us);
    Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi1CS[cs]);
    BusyWaitUs(options_.cs_hold_us);

    const uint32_t value =
        0
        | (0 << 29) // CS
        | (8 << 24) // data width
        | ((address & 0xff) << 16) // data
        ;

    if (size != 0) {
      spi_->txhold = value;
    } else {
      spi_->io = value;
    }

    while ((spi_->stat & AUXSPI_STAT_TX_EMPTY) == 0);

    if (size == 0) { return; }

    // Wait our address hold time.
    BusyWaitUs(options_.address_hold_us);

    size_t offset = 0;
    while (offset < size) {
      while (spi_->stat & AUXSPI_STAT_TX_FULL);
      const uint32_t data_value = 0
                                  | (0 << 29) // CS
                                  | (8 << 24) // data width
                                  | (data[offset] << 16) // data
                                  ;
      if (offset + 1 == size) {
        spi_->io = data_value;
      } else {
        spi_->txhold = data_value;
      }
      offset++;
    }

    // Discard anything in the RX fifo.
    while ((spi_->stat & AUXSPI_STAT_RX_EMPTY) == 0) {
      (void) spi_->io;
    }

    // Wait until we are no longer busy.
    while (spi_->stat & AUXSPI_STAT_BUSY);
  }

  void Read(int cs, int address, char* data, size_t size) {
    BusyWaitUs(options_.cs_hold_us);
    Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi1CS[cs]);
    BusyWaitUs(options_.cs_hold_us);

    const uint32_t value = 0
                           | (0 << 29) // CS
                           | (8 << 24) // data width
                           | ((address & 0xff) << 16) // data
                           ;
    if (size != 0) {
      spi_->txhold = value;
    } else {
      spi_->io = value;
    }

    while ((spi_->stat & AUXSPI_STAT_TX_EMPTY) == 0);

    if (size == 0) { return; }

    // Wait our address hold time.
    BusyWaitUs(options_.address_hold_us);

    // Discard the rx fifo.
    while ((spi_->stat & AUXSPI_STAT_RX_EMPTY) == 0) {
      (void) spi_->io;
    }

    // Now we write out dummy values, reading values in.
    std::size_t remaining_read = size;
    std::size_t remaining_write = remaining_read;
    char* ptr = data;
    while (remaining_read) {
      // Make sure we don't write more than we have read spots remaining
      // so that we can never overflow the RX fifo.
      const bool can_write = (remaining_read - remaining_write) < 3;
      if (can_write &&
          remaining_write && (spi_->stat & AUXSPI_STAT_TX_FULL) == 0) {
        const uint32_t to_write =
            0
            | (0 << 29) // CS
            | (8 << 24) // data width
            | (0) // data
            ;
        remaining_write--;
        if (remaining_write == 0) {
          spi_->io = to_write;
        } else {
          spi_->txhold = to_write;
        }
      }

      if (remaining_read && (spi_->stat & AUXSPI_STAT_RX_EMPTY) == 0) {
        *ptr = spi_->io & 0xff;
        ptr++;
        remaining_read--;
      }
    }
  }

 private:
  // This is the memory layout of the SPI peripheral.
  struct Bcm2835AuxSpi {
    uint32_t cntl0;
    uint32_t cntl1;
    uint32_t stat;
    uint32_t peek;
    uint32_t ign1[4];
    uint32_t io;
    uint32_t ign2;
    uint32_t ign3;
    uint32_t ign4;
    uint32_t txhold;
  };

  const Options options_;
  SystemFd fd_;
  SystemMmap spi_mmap_;
  volatile uint32_t* auxenb_ = nullptr;
  volatile Bcm2835AuxSpi* spi_ = nullptr;

  std::unique_ptr<Rpi3Gpio> gpio_;
};

///////////////////////////////////////////////
/// Structures exchanged with the pi3 hat over SPI

/// This is the format exported by register 34 on the hat.
struct DeviceAttitudeData {
  uint8_t present = 0;
  uint8_t update_time_10us = 0;
  float w = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  float x_dps = 0;
  float y_dps = 0;
  float z_dps = 0;
  float a_x_mps2 = 0;
  float a_y_mps2 = 0;
  float a_z_mps2 = 0;
  float bias_x_dps = 0;
  float bias_y_dps = 0;
  float bias_z_dps = 0;
  float uncertainty_w = 0;
  float uncertainty_x = 0;
  float uncertainty_y = 0;
  float uncertainty_z = 0;
  float uncertainty_bias_x_dps = 0;
  float uncertainty_bias_y_dps = 0;
  float uncertainty_bias_z_dps = 0;
  uint8_t padding[4] = {};
} __attribute__((packed));

struct DeviceMountingAngle {
  float yaw_deg = 0;
  float pitch_deg = 0;
  float roll_deg = 0;

  bool operator==(const DeviceMountingAngle& rhs) const {
    return yaw_deg == rhs.yaw_deg &&
        pitch_deg == rhs.pitch_deg &&
        roll_deg == rhs.roll_deg;
  }

  bool operator!=(const DeviceMountingAngle& rhs) const {
    return !(*this == rhs);
  }
} __attribute__((packed));


template <typename Spi>
Pi3Hat::ProcessorInfo GetProcessorInfo(Spi* spi, int cs) {
  // TODO
  return {};
}
}


class Pi3Hat::Impl {
 public:
  Impl(const Configuration& configuration)
      : config_(configuration),
        primary_spi_{[&]() {
            PrimarySpi::Options options;
            options.speed_hz = configuration.spi_speed_hz;
            return options;
    }()},
        aux_spi_{[&]() {
            AuxSpi::Options options;
            options.speed_hz = configuration.spi_speed_hz;
            return options;
          }()} {

    DeviceMountingAngle device_mounting;
    device_mounting.yaw_deg = configuration.mounting_deg.yaw;
    device_mounting.pitch_deg = configuration.mounting_deg.pitch;
    device_mounting.roll_deg = configuration.mounting_deg.roll;

    primary_spi_.Write(
        0, 36,
        reinterpret_cast<const char*>(&device_mounting),
        sizeof(device_mounting));

    // Give it some time to work.
    ::usleep(100);
    DeviceMountingAngle mounting_verify;
    primary_spi_.Read(
        0, 35,
        reinterpret_cast<char*>(&mounting_verify),
        sizeof(mounting_verify));
    ThrowIf(
        device_mounting != mounting_verify,
        [&]() {
          return Format(
              "Mounting angle not set properly (%f,%f,%f) != (%f,%f,%f)",
              device_mounting.yaw_deg,
              device_mounting.pitch_deg,
              device_mounting.roll_deg,
              mounting_verify.yaw_deg,
              mounting_verify.pitch_deg,
              mounting_verify.roll_deg);
        });

    // Configure our RF id.
    primary_spi_.Write(
        0, 50,
        reinterpret_cast<const char*>(&config_.rf_id), sizeof(uint32_t));
    ::usleep(100);
    uint32_t id_verify = 0;
    primary_spi_.Read(
        0, 49,
        reinterpret_cast<char*>(&id_verify), sizeof(id_verify));
    ThrowIf(
        config_.rf_id != id_verify,
        [&]() {
          return Format("RF Id not set properly (%08x != %08x)",
                        config_.rf_id,
                        id_verify);
        });
  }

  DeviceInfo device_info() {
    DeviceInfo result;
    // Now get the device information from all three processors.
    result.can1 = GetProcessorInfo(&aux_spi_, 0);
    result.can1 = GetProcessorInfo(&aux_spi_, 1);
    result.aux = GetProcessorInfo(&primary_spi_, 0);
    return result;
  }

  bool GetAttitude(Attitude* output, bool wait) {
    device_attitude_ = {};

    // Busy loop until we get something.
    do {
      primary_spi_.Read(
          0, 34,
          reinterpret_cast<char*>(&device_attitude_),
          sizeof(device_attitude_));
    } while (wait && ((device_attitude_.present & 0x01) == 0));

    if ((device_attitude_.present & 0x01) == 0) {
      return false;
    }

    const auto& da = device_attitude_;
    auto& o = *output;
    o.attitude = { da.w, da.x, da.y, da.z };
    o.rate_dps = { da.x_dps, da.y_dps, da.z_dps };
    o.accel_mps2 = { da.a_x_mps2, da.a_y_mps2, da.a_z_mps2 };
    o.bias_dps = { da.bias_x_dps, da.bias_y_dps, da.bias_z_dps };
    o.attitude_uncertainty = {
      da.uncertainty_w,
      da.uncertainty_x,
      da.uncertainty_y,
      da.uncertainty_z,
    };
    o.bias_uncertainty_dps = {
      da.uncertainty_bias_x_dps,
      da.uncertainty_bias_y_dps,
      da.uncertainty_bias_z_dps,
    };

    return true;
  }

  template <typename Spi>
  void SendCanPacketSpi(Spi& spi,
                        int cs, int cpu_bus,
                        const CanFrame& can_frame) {
    const auto size = RoundUpDlc(can_frame.size);
    char buf[70] = {};

    int spi_address = 0;
    int spi_size = 0;

    buf[0] = ((cpu_bus == 2) ? 0x80 : 0x00) | (can_frame.size  & 0x7f);

    if (can_frame.id <= 0xffff) {
      // We'll use the 2 byte ID formulation, cmd 5
      spi_address = 5;
      buf[1] = (can_frame.id >> 8) & 0xff;
      buf[2] = can_frame.id & 0xff;
      std::memcpy(&buf[3], can_frame.data, can_frame.size);
      for (std::size_t i = 3 + can_frame.size; i < size; i++) {
        buf[i] = 0x50;
      }
      spi_size = 3 + size;
    } else {
      // 4 byte formulation, cmd 4
      spi_address = 4;

      buf[1] = (can_frame.id >> 24) & 0xff;
      buf[2] = (can_frame.id >> 16) & 0xff;
      buf[3] = (can_frame.id >> 8) & 0xff;
      buf[4] = (can_frame.id >> 0) & 0xff;
      std::memcpy(&buf[5], can_frame.data, can_frame.size);
      for (std::size_t i = 5 + can_frame.size; i < size; i++) {
        buf[i] = 0x50;
      }
      spi_size = 5 + size;
    }

    spi.Write(cs, spi_address, buf, spi_size);
  }

  void SendCanPacket(const CanFrame& can_frame) {
    switch (can_frame.bus) {
      case 1: {
        SendCanPacketSpi(aux_spi_, 0, 0, can_frame);
        break;
      }
      case 2: {
        SendCanPacketSpi(aux_spi_, 0, 1, can_frame);
        break;
      }
      case 3: {
        SendCanPacketSpi(aux_spi_, 1, 0, can_frame);
        break;
      }
      case 4: {
        SendCanPacketSpi(aux_spi_, 1, 1, can_frame);
        break;
      }
      case 5: {
        SendCanPacketSpi(primary_spi_, 0, 0, can_frame);
        break;
      }
    }
  }

  struct ExpectedReply {
    std::array<int, 6> count = { {} };
  };

  ExpectedReply SendCan(const Input& input) {
    ExpectedReply result;

    // We try to send packets on alternating buses if possible, so we
    // can reduce the average latency before the first data goes out
    // on any bus.  Always send data on the low speed bus out last.
    for (auto& bus_packets : can_packets_) {
      bus_packets.resize(0);
    }
    for (size_t i = 0; i < input.tx_can.size(); i++) {
      const auto bus = input.tx_can[i].bus;
      can_packets_[bus].push_back(i);
      if (input.tx_can[i].expect_reply) {
        result.count[bus]++;
      }
    }

    int bus_offset[5] = {};
    while (true) {
      // We try to send out packets to buses in this order to minimize
      // latency.
      bool any_sent = false;
      for (const int bus : { 1, 3, 2, 4 }) {
        auto& offset = bus_offset[bus];
        if (offset >= can_packets_[bus].size()) { continue; }
        const auto& can_packet = input.tx_can[can_packets_[bus][offset]];
        offset++;

        SendCanPacket(can_packet);
      }

      if (!any_sent) { break; }
    }

    // Now send out all the low speed packets.
    for (const auto index : can_packets_[5]) {
      SendCanPacket(input.tx_can[index]);
    }

    return result;
  }

  template <typename Spi>
  int ReadCanFrames(Spi& spi, int cs, int bus_start,
                    const Span<CanFrame>* rx_can, Output* output) {
    // Is there any room?
    if (output->rx_can_size >= rx_can->size()) { return 0; }

    int count = 0;

    // Purposefully not initialized for speed.
    uint8_t buf[70];

    while (true) {
      // Read until no more frames are available or until the output
      // buffer is full.
      uint8_t queue_sizes[6] = {};
      spi.Read(cs, 2,
               reinterpret_cast<char*>(&queue_sizes[0]), sizeof(queue_sizes));

      bool any_read = false;

      // Read all we can until our buffer is full.
      for (int size : queue_sizes) {
        if (size == 0) { continue; }
        if (size > (64 + 5)) {
          // This is malformed.  Lets just set it to the maximum size for now.
          size = 64 + 5;
        }

        spi.Read(cs, 3, reinterpret_cast<char*>(&buf[0]), size);

        if (buf[0] == 0) {
          // Hmmm, this shouldn't happen, but indicates there isn't
          // really a frame here.
          continue;
        }

        auto& output_frame = (*rx_can)[output->rx_can_size++];
        count++;

        output_frame.bus = bus_start + ((buf[0] & 0x80) ? 1 : 0);
        output_frame.id = (buf[1] << 24) |
                          (buf[2] << 16) |
                          (buf[3] << 8) |
                          (buf[4] << 0);
        output_frame.size = size - 5;
        std::memcpy(output_frame.data, &buf[5], size - 5);
      }

      if (!any_read) {
        break;
      }
    }

    return count;
  }

  void ReadCan(const Input& input, const ExpectedReply& expected_replies,
               Output* output) {
    int bus_replies[] = {
      expected_replies.count[1] + expected_replies.count[2],
      expected_replies.count[3] + expected_replies.count[4],
      expected_replies.count[5],
    };

    const auto start_now = GetNow();

    while (true) {
      // Then check for CAN responses as necessary.
      if ((input.force_can_check & 0x06) || bus_replies[0]) {
        bus_replies[0] -=
            ReadCanFrames(aux_spi_, 0, 1, &input.rx_can, output);
      }
      if (input.force_can_check & 0x18 || bus_replies[1]) {
        bus_replies[1] -=
            ReadCanFrames(aux_spi_, 1, 3, &input.rx_can, output);
      }
      if ((input.force_can_check & 0x20) || bus_replies[2]) {
        bus_replies[2] -=
            ReadCanFrames(primary_spi_, 0, 5, &input.rx_can, output);
      }

      if (bus_replies[0] == 0 &&
          bus_replies[1] == 0 &&
          bus_replies[2] == 0) {
        // We've read all the replies we are expecting and have polled
        // everything at least once if requested.
        return;
      }

      if (output->rx_can_size >= input.rx_can.size()) {
        // Our buffer is full, so no more frames could have been
        // returned.
        return;
      }

      const auto cur_now = GetNow();
      const auto delta_ns = cur_now - start_now;
      if (delta_ns > input.timeout_ns) {
        // The timeout has expired.
        return;
      }
    }
  }

  Output Cycle(const Input& input) {
    // Send off all our CAN data to all buses.
    auto expected_replies = SendCan(input);

    Output result;

    // While those are sending, do our other work.
    if (input.tx_rf.size()) {
    }

    if (input.request_rf) {
    }

    if (input.request_attitude) {
      result.attitude_present =
          GetAttitude(input.attitude, input.wait_for_attitude);
    }

    ReadCan(input, expected_replies, &result);
    return result;
  }

  const Configuration config_;
  PrimarySpi primary_spi_;
  AuxSpi aux_spi_;

  DeviceAttitudeData device_attitude_;

  // This is a member variable purely so that in steady state we don't
  // have to allocate memory.
  //
  // It is 1 indexed to match the bus naming.
  std::vector<int> can_packets_[6];
};

Pi3Hat::Pi3Hat(const Configuration& configuration)
    : impl_(new Impl(configuration)) {}

Pi3Hat::~Pi3Hat() {
  delete impl_;
}

Pi3Hat::Output Pi3Hat::Cycle(const Input& input) {
  return impl_->Cycle(input);
}

}
}
