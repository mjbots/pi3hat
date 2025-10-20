// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

char g_data_block[4096] = {};

void copy_data(void* ptr) {
  ::memcpy(g_data_block, ptr, sizeof(g_data_block));
}

namespace mjbots {
namespace pi3hat {

namespace {
///////////////////////////////////////////////
/// Random utility functions

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
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000ll +
      static_cast<int64_t>(ts.tv_nsec);
}

void BusyWaitUs(int64_t us) {
  // We wait to ensure that setup and hold times are properly
  // enforced.  Allowing data stores and loads to be re-ordered around
  // the wait would defeat their purpose.  Thus, use barriers to force
  // a complete synchronization event on either side of our waits.
#ifdef __ARM_ARCH_ISA_A64
  asm volatile("dsb sy");
#elif __ARM_ARCH_7A__
  asm volatile("dsb");
#elif __ARM_ARCH_8A__
  asm volatile("dsb");
#elif __ARM_ARCH_6__
# error "RPI 1/2 are unsupported.  Perhaps you need '-march=native -mcpu=native -mtune=native'?"
#else
# error "Unknown architecture"
#endif

  const auto start = GetNow();
  const auto end = start + us * 1000;
  while (GetNow() <= end);

#ifdef __ARM_ARCH_ISA_A64
  asm volatile("dsb sy");
#elif __ARM_ARCH_7A__
  asm volatile("dsb");
#elif __ARM_ARCH_8A__
  asm volatile("dsb");
#elif __ARM_ARCH_6__
# error "RPI 1/2 are unsupported.  Perhaps you need '-march=native -mcpu=native -mtune=native'?"
#else
# error "Unknown architecture"
#endif
}

std::string ReadContents(const std::string& filename) {
  std::ifstream inf(filename);
  std::ostringstream ostr;
  ostr << inf.rdbuf();
  return ostr.str();
}

bool StartsWith(const std::string& value, const std::string& maybe_prefix) {
  return value.substr(0, maybe_prefix.size()) == maybe_prefix;
}

uint64_t host_get_peripheral_address() {
  FILE *fp = nullptr;
  char buf[1024] = {};

  fp = fopen("/proc/device-tree/model", "r");
  if (!fp) {
    throw Error("Unable to open /proc/device-tree/model");
  }

  if (fgets(buf, sizeof(buf), fp) == NULL) {
    fclose(fp);
    throw Error("Unable to read /proc/device-tree/model");
  }
  fclose(fp);

  const int model = [&]() {
    int version = -1;
    if (sscanf(buf, "Raspberry Pi %d ", &version) == 1) {
      return version;
    }
    if (sscanf(buf, "Raspberry Pi Compute Module %d ", &version) == 1) {
      return version;
    }
    throw Error(std::string("Unable to parse rpi version: ") + buf);
  }();

  // Return hard-coded values for known versions.
  if (model == 4) {
    return 0xfe000000UL;
  } else if (model < 4) {
    // Pi 3, 2, 1, Zero, Model A/B
    return 0x3f000000UL;
  }

  throw Error(std::string("Unsupported Raspberry Pi: ") + buf);
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
      : mmap_(dev_mem_fd, 4096, host_get_peripheral_address() + GPIO_BASE),
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
    // We actually only need hold times of around 3us.  However, the
    // linux aarch64 kernel sometimes returns up to 8us of difference
    // in consecutive calls to clock_gettime when in a tight busy loop
    // (and <1 us of wall clock time has actually passed as measured
    // by an oscilloscope).  This doesn't seem to be a problem on the
    // armv7l kernel.
    int cs_hold_us = 3;
    int address_hold_us = 3;

    Options() {}
  };

  PrimarySpi(const Options& options = Options()) {
    fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
    ThrowIfErrno(fd_ < 0, "pi3hat: could not open /dev/mem");

    spi_mmap_ = SystemMmap(
        fd_, 4096, host_get_peripheral_address() + SPI_BASE);
    spi_ = reinterpret_cast<volatile Bcm2835Spi*>(
        static_cast<char*>(spi_mmap_.ptr()));

    gpio_.reset(new Rpi3Gpio(fd_));

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
        | (0 << 4) // CLEAR
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

  Rpi3Gpio* gpio() {
    return gpio_.get();
  }

  void Write(int cs, int address, const char* data, size_t size) {
    BusyWaitUs(options_.cs_hold_us);
    Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi0CS[cs]);
    BusyWaitUs(options_.cs_hold_us);

    spi_->cs = (spi_->cs | (SPI_CS_TA | (3 << 4)));  // CLEAR

    spi_->fifo = address & 0xff;

    // We are done when we have received one byte back.
    while ((spi_->cs & SPI_CS_RXD) == 0);
    (void) spi_->fifo;

    if (size != 0) {
      // Wait our address hold time.
      BusyWaitUs(options_.address_hold_us);

      size_t offset = 0;
      while (offset < size) {
        while ((spi_->cs & SPI_CS_TXD) == 0);
        spi_->fifo =  data[offset];
        offset++;
      }

      // Wait until we are no longer busy.
      while ((spi_->cs & SPI_CS_DONE) == 0) {
        if (spi_->cs & SPI_CS_RXD) {
          (void) spi_->fifo;
        }
      }
    }

    spi_->cs = (spi_->cs & (~SPI_CS_TA));
  }

  void Read(int cs, int address, char* data, size_t size) {
    BusyWaitUs(options_.cs_hold_us);
    Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi0CS[cs]);
    BusyWaitUs(options_.cs_hold_us);

    spi_->cs = (spi_->cs | (SPI_CS_TA | (3 << 4)));  // CLEAR

    spi_->fifo = (address & 0x00ff);

    // We are done when we have received one byte back.
    while ((spi_->cs & SPI_CS_RXD) == 0);
    (void) spi_->fifo;

    if (size != 0) {
      // Wait our address hold time.
      BusyWaitUs(options_.address_hold_us);

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
          spi_->fifo = 0x00;
          remaining_write--;
        }

        if (remaining_read && (spi_->cs & SPI_CS_RXD) != 0) {
          *ptr = spi_->fifo & 0xff;
          ptr++;
          remaining_read--;
        }
      }
    }

    spi_->cs = (spi_->cs & (~SPI_CS_TA));
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
    // We actually only need hold times of around 3us, these are
    // larger for the same reasons as in PrimarySpi.
    int cs_hold_us = 3;
    int address_hold_us = 3;

    Options() {}
  };

  static constexpr int kPack = 3;

  AuxSpi(const Options& options = Options()) {
    fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
    ThrowIfErrno(fd_ < 0, "rpi3_aux_spi: could not open /dev/mem");

    spi_mmap_ = SystemMmap(
        fd_, 4096, host_get_peripheral_address() + AUX_BASE);
    auxenb_ = reinterpret_cast<volatile uint32_t*>(
        static_cast<char*>(spi_mmap_.ptr()) + 0x04);
    spi_ = reinterpret_cast<volatile Bcm2835AuxSpi*>(
        static_cast<char*>(spi_mmap_.ptr()) + 0x80);

    gpio_.reset(new Rpi3Gpio(fd_));

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
    *auxenb_ = (*auxenb_ & (~0x02));

    BusyWaitUs(10);

    // Enable the SPI peripheral.
    *auxenb_ = (*auxenb_ | 0x02);  // SPI1 enable

    spi_->cntl1 = 0;
    spi_->cntl0 = (1 << 9); // clear fifos

    // Configure the SPI peripheral.
    const int clkdiv =
        std::max(0, std::min(4095, 250000000 / 2 / options.speed_hz - 1));
    spi_->cntl0 = (
        0
        | (clkdiv << 20)
        | (0 << 17) // chip select defaults
        | (0 << 16) // post-input mode
        | (0 << 15) // variable CS
        | (1 << 14) // variable width
        | (2 << 12) // DOUT hold time
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
        | (0 << 8) // CS high time
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

      const size_t remaining = size - offset;
      const size_t to_write = std::min<size_t>(remaining, kPack);

      // The Auxiliary SPI controller inserts a small dead time
      // between each FIFO entry, even if the FIFO is all full up.
      // Thus, we work to minimize this by using all 3 available bytes
      // of each FIFO entry when possible.
      const uint32_t data_value =
          0
          | (0 << 29) // CS
          | ((to_write * 8) << 24) // data width
          | [&]() {
        if (to_write == 1) {
          return data[offset] << 16;
        } else if (to_write == 2) {
          return (data[offset] << 16) | (data[offset + 1] << 8);
        } else if (to_write == 3) {
          return (data[offset] << 16) | (data[offset + 1] << 8) | data[offset + 2];
        }
        // We should never get here.
        return 0;
      }();

      if (offset + to_write == size) {
        spi_->io = data_value;
      } else {
        spi_->txhold = data_value;
      }
      offset += to_write;
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

    while (true) {
      const auto stat = spi_->stat;
      if ((stat & AUXSPI_STAT_BUSY) == 0 &&
          (stat & AUXSPI_STAT_TX_EMPTY) != 0) {
        break;
      }
    }

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
      const bool can_write = (remaining_read - remaining_write) < (3 * kPack);
      const uint32_t cur_stat = spi_->stat;
      const bool tx_full = (cur_stat & AUXSPI_STAT_TX_FULL) != 0;

      if (can_write && remaining_write && !tx_full) {
        const size_t to_read = std::min<size_t>(remaining_write, kPack);
        const uint32_t to_write =
            0
            | (0 << 29) // CS
            | ((8 * to_read) << 24) // data width
            | (0) // data
            ;
        remaining_write -= to_read;
        if (remaining_write == 0) {
          spi_->io = to_write;
        } else {
          spi_->txhold = to_write;
        }
      }

      if (remaining_read && (spi_->stat & AUXSPI_STAT_RX_EMPTY) == 0) {
        const uint32_t value = spi_->io;

        const size_t byte_count = std::min<size_t>(remaining_read, kPack);
        switch (byte_count) {
          case 3:
            *ptr++ = (value >> 16) & 0xff;
            // fall through
          case 2:
            *ptr++ = (value >> 8) & 0xff;
            // fall through
          case 1:
            *ptr++ = (value >> 0) & 0xff;
        }
        remaining_read -= byte_count;
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

struct DeviceImuConfiguration {
  float roll_deg = 0;
  float pitch_deg = 0;
  float yaw_deg = 0;
  uint32_t rate_hz = 0;

  bool operator==(const DeviceImuConfiguration& rhs) const {
    return yaw_deg == rhs.yaw_deg &&
        pitch_deg == rhs.pitch_deg &&
        roll_deg == rhs.roll_deg &&
        rate_hz == rhs.rate_hz;
  }

  bool operator!=(const DeviceImuConfiguration& rhs) const {
    return !(*this == rhs);
  }
} __attribute__((packed));

struct DeviceSlotData {
  uint32_t age_ms = 0;
  uint8_t size = 0;
  uint8_t data[16] = {};
} __attribute__((packed));

struct DeviceRfStatus {
  uint32_t bitfield = 0;
  uint32_t lock_age_ms = 0;
} __attribute__((packed));

struct DeviceDeviceInfo {
  uint8_t git_hash[20] = {};
  uint8_t dirty = 0;
  uint8_t serial_number[12] = {};
} __attribute__((packed));

struct DevicePerformance {
  uint32_t cycles_per_ms = 0;
  uint32_t min_cycles_per_ms = 0;
} __attribute__((packed));

struct DeviceCanRate {
  int8_t prescaler = -1;
  int8_t sync_jump_width = -1;
  int8_t time_seg1 = -1;
  int8_t time_seg2 = -1;

  bool operator==(const DeviceCanRate& rhs) const {
    return prescaler == rhs.prescaler &&
        sync_jump_width == rhs.sync_jump_width &&
        time_seg1 == rhs.time_seg1 &&
        time_seg2 == rhs.time_seg2;
  }

  bool operator!=(const DeviceCanRate& rhs) const {
    return !(*this == rhs);
  }
} __attribute__((packed));

struct DeviceCanConfiguration {
  int32_t slow_bitrate = 1000000;
  int32_t fast_bitrate = 5000000;
  int8_t fdcan_frame = 1;
  int8_t bitrate_switch = 1;
  int8_t automatic_retransmission = 0;
  int8_t restricted_mode = 0;
  int8_t bus_monitor = 0;

  // If any members of either 'rate' structure are non-negative, use
  // them instead of the 'bitrate' fields above.  Each rate applies
  // to a base clock rate of 85MHz.
  DeviceCanRate std_rate;
  DeviceCanRate fd_rate;

  bool operator==(const DeviceCanConfiguration& rhs) const {
    return slow_bitrate == rhs.slow_bitrate &&
        fast_bitrate == rhs.fast_bitrate &&
        fdcan_frame == rhs.fdcan_frame &&
        bitrate_switch == rhs.bitrate_switch &&
        automatic_retransmission == rhs.automatic_retransmission &&
        restricted_mode == rhs.restricted_mode &&
        bus_monitor == rhs.bus_monitor &&
        std_rate == rhs.std_rate &&
        fd_rate == rhs.fd_rate;
  }

  bool operator!=(const DeviceCanConfiguration& rhs) const {
    return !(*this == rhs);
  }
} __attribute__((packed));

struct DeviceCanConfigurationV4 : DeviceCanConfiguration {
  uint32_t cancel_all_ms = 50;

  bool operator==(const DeviceCanConfigurationV4& rhs) const {
    return DeviceCanConfiguration::operator==(rhs) &&
        cancel_all_ms == rhs.cancel_all_ms;
  }

  bool operator!=(const DeviceCanConfigurationV4& rhs) const {
    return !(*this == rhs);
  }
} __attribute__((packed));

template <typename Spi>
Pi3Hat::ProcessorInfo GetProcessorInfo(Spi* spi, int cs) {
  DeviceDeviceInfo di;
  spi->Read(cs, 97, reinterpret_cast<char*>(&di), sizeof(di));

  Pi3Hat::ProcessorInfo result;
  ::memcpy(&result.git_hash[0], &di.git_hash[0], sizeof(di.git_hash));
  result.dirty = di.dirty != 0;
  ::memcpy(&result.serial_number[0], &di.serial_number[0],
           sizeof(di.serial_number));
  return result;
}

template <typename Spi>
Pi3Hat::PerformanceInfo GetPerformance(Spi* spi, int cs) {
  DevicePerformance dp;
  spi->Read(cs, 100, reinterpret_cast<char*>(&dp), sizeof(dp));

  Pi3Hat::PerformanceInfo result;
  result.cycles_per_ms = dp.cycles_per_ms;
  result.min_cycles_per_ms = dp.min_cycles_per_ms;
  return result;
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

    // First, look to see if we have a pi3hat attached by looking for
    // the eeprom data.  This will prevent us from stomping on the SPI
    // registers if it isn't ours.
    const auto product_code =
        ReadContents("/sys/firmware/devicetree/base/hat/product");
    if (!StartsWith(product_code, "mjbots quad pi3 hat")) {
      throw std::runtime_error("No pi3hat detected");
    }

    // Since we directly poke at /dev/mem, nothing good can come of
    // multiple instances of this class existing at once on the same
    // system.
    //
    // Thus, we use a lock to ensure that at most one copy runs at a
    // time.
    LockFile();

    if (config_.raw_spi_only) {
      return;
    }

    // Verify the versions of all peripherals we will use.
    VerifyVersions();

    if (config_.enable_aux) {
      ConfigureAux();
    }
    ConfigureCan();
  }

  ~Impl() {
    if (lock_file_fd_ >= 0) {
      // Who cares about errors here?
      ::close(lock_file_fd_);
    }
  }

  void ConfigureAux() {
    // See if we need to update the IMU configuration.
    DeviceImuConfiguration original_imu_configuration;
    primary_spi_.Read(
        0, 35,
        reinterpret_cast<char*>(&original_imu_configuration),
        sizeof(original_imu_configuration));

    DeviceImuConfiguration desired_imu;
    desired_imu.yaw_deg = config_.mounting_deg.yaw;
    desired_imu.pitch_deg = config_.mounting_deg.pitch;
    desired_imu.roll_deg = config_.mounting_deg.roll;
    desired_imu.rate_hz =
        std::min<uint32_t>(1000, config_.attitude_rate_hz);

    if (desired_imu != original_imu_configuration) {
      primary_spi_.Write(
          0, 36,
          reinterpret_cast<const char*>(&desired_imu),
          sizeof(desired_imu));

      // Give it some time to work.
      ::usleep(1000);
      DeviceImuConfiguration config_verify;
      primary_spi_.Read(
          0, 35,
          reinterpret_cast<char*>(&config_verify),
          sizeof(config_verify));
      ThrowIf(
          desired_imu != config_verify,
          [&]() {
            return Format(
                "IMU config not set properly (%f,%f,%f) %d != (%f,%f,%f) %d",
                desired_imu.yaw_deg,
                desired_imu.pitch_deg,
                desired_imu.roll_deg,
                desired_imu.rate_hz,
                config_verify.yaw_deg,
                config_verify.pitch_deg,
                config_verify.roll_deg,
                config_verify.rate_hz);
          });
    }

    // Configure our RF id if necessary.
    uint32_t original_id = 0;
    primary_spi_.Read(
        0, 49,
        reinterpret_cast<char*>(&original_id), sizeof(original_id));
    if (original_id != config_.rf_id) {
      primary_spi_.Write(
          0, 50,
          reinterpret_cast<const char*>(&config_.rf_id), sizeof(uint32_t));
      // Changing the ID takes at least a few milliseconds.
      ::usleep(10000);
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
  }

  template <typename Spi>
  void UpdateCanConfig(Spi* spi, int cs, int canbus,
                       const CanConfiguration& can_config) {
    // If the version is before 3, then we can't config anything.
    const auto version = ReadByte(spi, cs, 0);
    if (version < 3) { return; }

    // Populate what we want our config to look like.
    DeviceCanConfigurationV4 out;
    out.slow_bitrate = can_config.slow_bitrate;
    out.fast_bitrate = can_config.fast_bitrate;
    out.fdcan_frame = can_config.fdcan_frame ? 1 : 0;
    out.bitrate_switch = can_config.bitrate_switch ? 1 : 0;
    out.automatic_retransmission = can_config.automatic_retransmission ? 1 : 0;
    out.bus_monitor = can_config.bus_monitor ? 1 : 0;

    out.std_rate.prescaler = can_config.std_rate.prescaler;
    out.std_rate.sync_jump_width = can_config.std_rate.sync_jump_width;
    out.std_rate.time_seg1 = can_config.std_rate.time_seg1;
    out.std_rate.time_seg2 = can_config.std_rate.time_seg2;

    out.fd_rate.prescaler = can_config.fd_rate.prescaler;
    out.fd_rate.sync_jump_width = can_config.fd_rate.sync_jump_width;
    out.fd_rate.time_seg1 = can_config.fd_rate.time_seg1;
    out.fd_rate.time_seg2 = can_config.fd_rate.time_seg2;

    out.cancel_all_ms = can_config.cancel_all_ms;

    // Check to see if this is what is already there.
    DeviceCanConfigurationV4 original_config;
    const auto spi_size = version <= 3 ?
        sizeof(DeviceCanConfiguration) :
        sizeof(DeviceCanConfigurationV4);
    spi->Read(cs, canbus ? 8 : 7,
              reinterpret_cast<char*>(&original_config), spi_size);
    if (original_config == out) {
      // We have nothing to do, so just bail early.
      return;
    }

    // Update the configuration on the device.
    spi->Write(cs, canbus ? 10 : 9,
               reinterpret_cast<const char*>(&out), spi_size);

    // Give it some time to work.
    ::usleep(100);

    if (version <= 3) {
      DeviceCanConfiguration verify;
      spi->Read(cs, canbus ? 8 : 7,
                reinterpret_cast<char*>(&verify), spi_size);
      ThrowIf(
          static_cast<DeviceCanConfiguration>(out) != verify,
          [&]() {
            return "Could not set CAN configuration properly";
          });
    } else {
      DeviceCanConfigurationV4 verify;
      spi->Read(cs, canbus ? 8 : 7,
                reinterpret_cast<char*>(&verify), spi_size);
      ThrowIf(
          out != verify,
          [&]() {
            return "Could not set CAN configuration properly";
          });
    }
  }

  void ConfigureCan() {
    UpdateCanConfig(&aux_spi_, 0, 0, config_.can[0]);
    UpdateCanConfig(&aux_spi_, 0, 1, config_.can[1]);
    UpdateCanConfig(&aux_spi_, 1, 0, config_.can[2]);
    UpdateCanConfig(&aux_spi_, 1, 1, config_.can[3]);
    if (config_.enable_aux) {
      UpdateCanConfig(&primary_spi_, 0, 0, config_.can[4]);
    }
  }

  void LockFile() {
    struct flock lock = {};
    lock.l_type = F_WRLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;
    lock.l_pid = -1;

    lock_file_fd_ = ::open("/tmp/.pi3hat-lock", O_RDWR | O_CREAT,
                           S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    ThrowIfErrno(lock_file_fd_ < 0, "pi3hat: could not open lock file");

    const int ret = ::fcntl(lock_file_fd_, F_SETLK, &lock);
    ThrowIfErrno(ret < 0, "pi3hat: could not acquire lock, is another process running?");
  }

  template <typename Spi>
  uint8_t ReadByte(Spi* spi, int cs, int address) {
    uint8_t data = 0;
    spi->Read(cs, address, reinterpret_cast<char*>(&data), 1);
    return data;
  }

  template <typename Spi>
  void TestCan(Spi* spi, int cs, const char* name) {
    const auto version = ReadByte(spi, cs, 0);
    if (version != 2 && version != 3 && version != 4) {
      throw std::runtime_error(
          Format(
              "Processor '%s' has incorrect CAN SPI version %d != [2,3,4]",
              name, version));
    }
  }

  void VerifyVersions() {
    constexpr int kAttitudeVersion = 0x20;
    constexpr int kRfVersion = 0x10;

    if (config_.enable_aux) {
      TestCan(&primary_spi_, 0, "aux");
    }
    TestCan(&aux_spi_, 0, "can1");
    TestCan(&aux_spi_, 1, "can2");

    if (config_.enable_aux) {
      const auto attitude_version = ReadByte(&primary_spi_, 0, 32);
      if (attitude_version != kAttitudeVersion) {
        throw std::runtime_error(
            Format(
                "Incorrect attitude version %d != %d",
                attitude_version, kAttitudeVersion));
      }


      const auto rf_version = ReadByte(&primary_spi_, 0, 48);
      if (rf_version != kRfVersion) {
        throw std::runtime_error(
            Format(
                "Incorrect RF version %d != %d",
                rf_version, kRfVersion));
      }
    }
  }

  DeviceInfo device_info() {
    DeviceInfo result;
    // Now get the device information from all three processors.
    result.can1 = GetProcessorInfo(&aux_spi_, 0);
    result.can2 = GetProcessorInfo(&aux_spi_, 1);
    if (config_.enable_aux) {
      result.aux = GetProcessorInfo(&primary_spi_, 0);
    }

    // Verify all the CAN protocols for "unknown address safety".
    const auto can1_can_protocol = ReadByte(&aux_spi_, 0, 0);
    const auto can2_can_protocol = ReadByte(&aux_spi_, 1, 0);
    const auto aux_can_protocol =
        config_.enable_aux ?
        ReadByte(&primary_spi_, 0, 0) : can1_can_protocol;

    result.can_unknown_address_safe =
        (can1_can_protocol > 3) &&
        (can2_can_protocol > 3) &&
        (aux_can_protocol > 3);

    return result;
  }

  DevicePerformance device_performance() {
    DevicePerformance result;
    // Now get the device information from all three processors.
    result.can1 = GetPerformance(&aux_spi_, 0);
    result.can2 = GetPerformance(&aux_spi_, 1);
    if (config_.enable_aux) {
      result.aux = GetPerformance(&primary_spi_, 0);
    }
    return result;
  }

  void ReadSpi(int spi_bus, int address, char* data, size_t size) {
    if (spi_bus == 0) {
      aux_spi_.Read(0, address, data, size);
    } else if (spi_bus == 1) {
      aux_spi_.Read(1, address, data, size);
    } else if (spi_bus == 2) {
      primary_spi_.Read(0, address, data, size);
    }
  }

  bool GetAttitude(Attitude* output, bool wait, bool detail) {
    device_attitude_ = {};

    // Busy loop until we get something.
    if (wait) {
      char buf[2] = {};
      do {
        primary_spi_.Read(0, 96, buf, sizeof(buf));
        if (buf[1] == 1) { break; }
        // If we spam the STM32 too hard, then it doesn't have any
        // cycles left to actually work on the IMU.
        BusyWaitUs(20);
      } while (true);
    }

    do {
      primary_spi_.Read(
          0, 34,
          reinterpret_cast<char*>(&device_attitude_),
          detail ? sizeof(device_attitude_) : 42);
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

    buf[0] = ((cpu_bus == 1) ? 0x80 : 0x00) | (size & 0x7f);

    if (can_frame.id <= 0xffff) {
      // We'll use the 2 byte ID formulation, cmd 5
      spi_address = 5;
      buf[1] = (can_frame.id >> 8) & 0xff;
      buf[2] = can_frame.id & 0xff;
      ::memcpy(&buf[3], can_frame.data, can_frame.size);
      for (std::size_t i = 3 + can_frame.size; i < (3 + size); i++) {
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
      ::memcpy(&buf[5], can_frame.data, can_frame.size);
      for (std::size_t i = 5 + can_frame.size; i < (5 + size); i++) {
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
        if (config_.enable_aux) {
          SendCanPacketSpi(primary_spi_, 0, 0, can_frame);
        }
        break;
      }
    }
  }

  struct ExpectedReply {
    std::array<int, 6> count = { {} };

    // How long we expect each bus to take to send the frames.
    std::array<int64_t, 6> send_ns = { {} };

    // How long we expect each bus to take to receive any responses.
    std::array<int64_t, 6> receive_ns = { {} };
  };

  ExpectedReply CalculateExpectedReply(const Input& input) {
    ExpectedReply result;

    for (size_t i = 0; i < input.tx_can.size(); i++) {
      const auto bus = input.tx_can[i].bus;

      const int64_t arbitration_bitrate =
          config_.can[bus - 1].slow_bitrate;
      const int64_t data_bitrate =
          config_.can[bus - 1].bitrate_switch ?
          config_.can[bus - 1].fast_bitrate :
          config_.can[bus - 1].slow_bitrate;

      const int64_t can_header_size_bits =
          (input.tx_can[i].id >= 2048 ? 32 : 16) + 8;
      const int64_t can_data_size_bits =
          input.tx_can[i].size * 8 + 28;
      const int64_t tx_spi_bits =
          (input.tx_can[i].size + 5) * 8;
      const int64_t can_send_ns =
          1000000000 * can_header_size_bits / arbitration_bitrate +
          1000000000 * can_data_size_bits / data_bitrate +
          1000000000 * tx_spi_bits / config_.spi_speed_hz;

      result.send_ns[bus] += can_send_ns;

      if (input.tx_can[i].expect_reply) {
        const int64_t rx_header_size_bits = 32;
        const int64_t rx_data_size_bits =
            input.tx_can[i].expected_reply_size * 8 + 28;
        const int64_t rx_spi_bits =
            (input.tx_can[i].expected_reply_size + 5) * 8;
        const int64_t rx_ns =
            1000000000 * rx_header_size_bits / arbitration_bitrate +
            1000000000 * rx_data_size_bits / data_bitrate +
            1000000000 * rx_spi_bits / config_.spi_speed_hz;

        result.receive_ns[bus] += rx_ns;
      }

      if (input.tx_can[i].expect_reply) {
        result.count[bus]++;
      }
    }
    return result;
  }

  void SendCan(const Input& input) {
    // We try to send packets on alternating buses if possible, so we
    // can reduce the average latency before the first data goes out
    // on any bus.
    for (auto& bus_packets : can_packets_) {
      bus_packets.resize(0);
    }

    for (size_t i = 0; i < input.tx_can.size(); i++) {
      const auto bus = input.tx_can[i].bus;
      can_packets_[bus].push_back(i);
    }

    int bus_offset[6] = {};
    while (true) {
      // We try to send out packets to buses in this order to minimize
      // latency.
      bool any_sent = false;
      for (const int bus : { 1, 3, 5, 2, 4}) {
        auto& offset = bus_offset[bus];
        if (offset >= static_cast<int>(can_packets_[bus].size())) {
          continue;
        }
        const auto& can_packet = input.tx_can[can_packets_[bus][offset]];
        offset++;

        SendCanPacket(can_packet);
        any_sent = true;
      }

      if (!any_sent) { break; }
    }
  }

  void SendRf(const Span<RfSlot>& slots) {
    if (!config_.enable_aux) { return; }

    constexpr int kHeaderSize = 5;
    constexpr int kMaxDataSize = 16;
    uint8_t buf[kHeaderSize + kMaxDataSize] = {};

    for (size_t i = 0; i < slots.size(); i++) {
      const auto& slot = slots[i];
      buf[0] = slot.slot;
      buf[1] = (slot.priority >> 0) & 0xff;
      buf[2] = (slot.priority >> 8) & 0xff;
      buf[3] = (slot.priority >> 16) & 0xff;
      buf[4] = (slot.priority >> 24) & 0xff;
      ::memcpy(&buf[kHeaderSize], slot.data, slot.size);

      primary_spi_.Write(
          0, 51, reinterpret_cast<const char*>(&buf[0]),
          kHeaderSize + slot.size);
    }
  }

  void ReadRf(const Input& input, Output* output) {
    if (!config_.enable_aux) { return; }

    DeviceRfStatus rf_status;
    primary_spi_.Read(
        0, 52, reinterpret_cast<char*>(&rf_status), sizeof(rf_status));

    output->rf_lock_age_ms = rf_status.lock_age_ms;

    const auto bitfield_delta = rf_status.bitfield ^ last_bitfield_;
    if (bitfield_delta == 0) { return; }

    DeviceSlotData slot_data;

    for (int i = 0; i < 15; i++) {
      if (output->rx_rf_size >= input.rx_rf.size()) {
        // No more room.
        return;
      }

      if ((bitfield_delta & (3 << (i * 2))) == 0) {
        continue;
      }

      last_bitfield_ ^= (bitfield_delta & (3 << (i * 2)));

      primary_spi_.Read(
          0, 64 + i, reinterpret_cast<char*>(&slot_data), sizeof(slot_data));
      auto& output_slot = input.rx_rf[output->rx_rf_size++];
      output_slot.slot = i;
      output_slot.age_ms = slot_data.age_ms;
      output_slot.size = slot_data.size;
      ::memcpy(output_slot.data, slot_data.data, slot_data.size);
    }
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
        if (output->rx_can_size >= rx_can->size()) {
          // We're full and can't read any more.
          break;
        }

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
        ::memcpy(output_frame.data, &buf[5], size - 5);
      }

      if (!any_read) {
        break;
      }
    }

    return count;
  }

  void FlushReadCan(const Input& input, const ExpectedReply& expected_replies,
                    Output* output) {
    const auto can1_expected =
        (expected_replies.count[1] + expected_replies.count[2]) ||
        input.force_can_check & 0x06;

    if (can1_expected) {
      ReadCanFrames(aux_spi_, 0, 1, &input.rx_can, output);
    }

    const auto can2_expected =
        (expected_replies.count[3] + expected_replies.count[4]) ||
        input.force_can_check & 0x18;
    if (can2_expected) {
      ReadCanFrames(aux_spi_, 1, 3, &input.rx_can, output);
    }

    const auto aux_expected =
        ((expected_replies.count[5]) || input.force_can_check & 0x20) &&
        config_.enable_aux;
    if (aux_expected) {
      ReadCanFrames(primary_spi_, 0, 5, &input.rx_can, output);
    }
  }

  void ReadCan(const Input& input, const ExpectedReply& expected_replies,
               Output* output) {
    int bus_replies[] = {
      expected_replies.count[1] + expected_replies.count[2],
      expected_replies.count[3] + expected_replies.count[4],
      expected_replies.count[5],
    };

    const int64_t min_rx_timeout_ns = [&]() {
      int64_t biggest = 0;
      for (int bus = 1; bus <= 5; bus++) {
        const int64_t this_ns =
            expected_replies.count[bus] ?
            (expected_replies.send_ns[bus] + expected_replies.receive_ns[bus]) : 0;
        if (this_ns > biggest) { biggest = this_ns; }
      }
      return biggest + input.rx_baseline_wait_ns;
    }();

    const bool to_check[] = {
      bus_replies[0] || input.force_can_check & 0x06,
      bus_replies[1] || input.force_can_check & 0x18,
      bus_replies[2] || input.force_can_check & 0x20,
    };

    const auto start_now = GetNow();
    int64_t last_reply = start_now;

    while (true) {
      bool any_found = false;
      // Then check for CAN responses as necessary.
      if (to_check[0]) {
        const int count = ReadCanFrames(aux_spi_, 0, 1, &input.rx_can, output);
        bus_replies[0] -= count;
        if (count) {
          last_reply = GetNow();
          any_found = true;
        }
      }
      if (to_check[1]) {
        const int count = ReadCanFrames(aux_spi_, 1, 3, &input.rx_can, output);
        bus_replies[1] -= count;
        if (count) {
          last_reply = GetNow();
          any_found = true;
        }
      }
      if (to_check[2] && config_.enable_aux) {
        const int count = ReadCanFrames(primary_spi_, 0, 5, &input.rx_can, output);
        bus_replies[2] -= count;
        if (count) {
          last_reply = GetNow();
          any_found = true;
        }
      }

      if (output->rx_can_size >= input.rx_can.size()) {
        // Our buffer is full, so no more frames could have been
        // returned.
        return;
      }

      const auto cur_now = GetNow();
      const auto delta_ns = cur_now - start_now;
      const auto since_last_ns = cur_now - last_reply;

      if (bus_replies[0] <= 0 &&
          bus_replies[1] <= 0 &&
          bus_replies[2] <= 0 &&
          delta_ns > input.min_tx_wait_ns &&
          since_last_ns > input.rx_extra_wait_ns) {
        // We've read all the replies we are expecting and have polled
        // everything at least once if requested.
        return;
      }

      if (delta_ns > input.timeout_ns &&
          !(delta_ns < input.min_tx_wait_ns ||
            delta_ns < min_rx_timeout_ns ||
            since_last_ns < input.rx_extra_wait_ns)) {
        // The timeout has expired.
        return;
      }

      if (!any_found) {
        // Give the controllers a chance to rest.
        BusyWaitUs(10);
      }
    }
  }

  Output Cycle(const Input& input) {
    Output result;

    auto expected_replies = CalculateExpectedReply(input);

    // First, ensure there aren't any receive frames sitting around
    // before we start for CAN busses we expect to have a reply for.
    FlushReadCan(input, expected_replies, &result);

    // Send off all our CAN data to all buses.
    SendCan(input);

    // While those are sending, do our other work.
    if (input.tx_rf.size()) {
      SendRf(input.tx_rf);
    }

    if (input.request_rf) {
      ReadRf(input, &result);
    }

    if (input.request_attitude) {
      result.attitude_present =
          GetAttitude(input.attitude, input.wait_for_attitude,
                      input.request_attitude_detail);
    }

    ReadCan(input, expected_replies, &result);

    primary_spi_.gpio()->SetGpioMode(13, Rpi3Gpio::OUTPUT);
    static bool debug_toggle = false;
    primary_spi_.gpio()->SetGpioOutput(13, debug_toggle);
    debug_toggle = !debug_toggle;

    return result;
  }

  const Configuration config_;

  int lock_file_fd_ = -1;

  PrimarySpi primary_spi_;
  AuxSpi aux_spi_;

  DeviceAttitudeData device_attitude_;

  // This is a member variable purely so that in steady state we don't
  // have to allocate memory.
  //
  // It is 1 indexed to match the bus naming.
  std::vector<int> can_packets_[6];

  // To keep track of which RF slots we have processed.
  uint32_t last_bitfield_ = 0;
};

Pi3Hat::Pi3Hat(const Configuration& configuration)
    : impl_(new Impl(configuration)) {}

Pi3Hat::~Pi3Hat() {
  delete impl_;
}

Pi3Hat::Output Pi3Hat::Cycle(const Input& input) {
  return impl_->Cycle(input);
}

Pi3Hat::DeviceInfo Pi3Hat::device_info() {
  return impl_->device_info();
}

Pi3Hat::DevicePerformance Pi3Hat::device_performance() {
  return impl_->device_performance();
}

void Pi3Hat::ReadSpi(int spi_bus, int address, char* data, size_t size) {
  impl_->ReadSpi(spi_bus, address, data, size);
}

}
}
