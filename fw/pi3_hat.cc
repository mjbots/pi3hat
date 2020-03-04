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

#include <atomic>
#include <cstring>

#include "mbed.h"
#include "PeripheralPins.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/string_span.h"
#include "mjlib/micro/callback_table.h"

#include "fw/attitude_reference.h"
#include "fw/bmi088.h"
#include "fw/fdcan.h"
#include "fw/millisecond_timer.h"
#include "fw/nrf24l01.h"

namespace fw {
namespace {
template <typename T>
uint32_t u32(T value) {
  return reinterpret_cast<uint32_t>(value);
}

template <typename T>
uint8_t u8(T value) {
  return static_cast<uint8_t>(value);
}

void EnableSPI(SPI_TypeDef* spi) {
#if defined (SPI1_BASE)
  if (spi == SPI1) {
    __HAL_RCC_SPI1_CLK_ENABLE();
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    return;
  }
#endif
#if defined (SPI2_BASE)
  if (spi == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST;
    return;
  }
#endif
#if defined (SPI3_BASE)
  if (spi == SPI3) {
    __HAL_RCC_SPI3_CLK_ENABLE();
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI3RST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI3RST;
    return;
  }
#endif
#if defined (SPI4_BASE)
  if (spi == SPI4) {
    __HAL_RCC_SPI4_CLK_ENABLE();
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    return;
  }
#endif
  mbed_die();
}

IRQn_Type GetSpiIrq(SPI_TypeDef* spi) {
  switch (u32(spi)) {
#if defined (SPI1_BASE)
    case SPI_1: { return SPI1_IRQn; }
#endif
#if defined (SPI2_BASE)
    case SPI_2: { return SPI2_IRQn; }
#endif
#if defined (SPI3_BASE)
    case SPI_3: { return SPI3_IRQn; }
#endif
#if defined (SPI4_BASE)
    case SPI_4: { return SPI4_IRQn; }
#endif
  }
  mbed_die();
}

template <typename T>
T merge(T a, T b) {
  if (a == b) { return a; }
  return T();
}

template <typename T>
uint8_t ReadRegister(T* value) {
  return *reinterpret_cast<__IO uint8_t*>(value);
}

template <typename T>
void WriteRegister(T* reg, uint8_t value) {
  *reinterpret_cast<__IO uint8_t*>(reg) = value;
}

/// Implements a register based SPI slave.
///
/// 8 bit transactions are assumed, with a 16 bit address scheme.  The
/// master first sends the address, then simultaneously reads and
/// writes from the logical address space.
///
/// The user of this class provides two functions:
///  * StartHandler - this is called after the slave has been selected
///    and the address has been sent.  It is responsible for returning
///    two buffers, one which holds the data to be written out to the
///    master, and one to hold the data the master sends.
///
///    This callback must complete in under 1 microsecond.
///
///  * EndHandler - this is called when the most recent transaction is
///    complete.  It is passed the number of bytes which were
///    transferred.
class RegisterSPISlave {
 public:
  struct Buffer {
    std::string_view tx;
    mjlib::base::string_span rx;
  };

  using StartHandler = mjlib::base::inplace_function<Buffer (uint16_t)>;
  using EndHandler = mjlib::base::inplace_function<void (uint16_t, int)>;

  RegisterSPISlave(PinName mosi, PinName miso, PinName sclk, PinName ssel,
                   StartHandler start_handler, EndHandler end_handler)
      : nss_{ssel},
        start_handler_(start_handler),
        end_handler_(end_handler) {
    spi_ = [&]() {
      const auto spi_mosi = pinmap_peripheral(mosi, PinMap_SPI_MOSI);
      const auto spi_miso = pinmap_peripheral(miso, PinMap_SPI_MISO);
      const auto spi_sclk = pinmap_peripheral(sclk, PinMap_SPI_SCLK);
      const auto spi_ssel = pinmap_peripheral(ssel, PinMap_SPI_SSEL);
      return reinterpret_cast<SPI_TypeDef*>(
          merge(spi_mosi, merge(spi_miso, merge(spi_sclk, spi_ssel))));
    }();

    MJ_ASSERT(spi_ != nullptr);

    pinmap_pinout(mosi, PinMap_SPI_MOSI);
    pinmap_pinout(miso, PinMap_SPI_MISO);
    pinmap_pinout(sclk, PinMap_SPI_SCLK);
    pin_mode(sclk, PullDown);
    pin_mode(ssel, PullUp);

    Init();

    nss_.rise(callback(this, &RegisterSPISlave::ISR_HandleNssRise));
    nss_.fall(callback(this, &RegisterSPISlave::ISR_HandleNssFall));

    isr_ = mjlib::micro::CallbackTable::MakeFunction([this]() {
        this->ISR_SPI();
      });

    const auto irq = GetSpiIrq(spi_);

    NVIC_SetVector(
        irq,
        u32(isr_.raw_function));

    HAL_NVIC_SetPriority(irq, 0, 0);
    HAL_NVIC_EnableIRQ(irq);
  }

  void Init() {
    EnableSPI(spi_);

    spi_handle_.Instance = spi_;
    auto& init = spi_handle_.Init;
    init.Mode = SPI_MODE_SLAVE;
    init.Direction = SPI_DIRECTION_2LINES;
    init.DataSize = SPI_DATASIZE_8BIT;
    init.CLKPolarity = SPI_POLARITY_LOW;
    init.CLKPhase = SPI_PHASE_1EDGE;
    init.NSS = SPI_NSS_SOFT;
    init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    init.FirstBit = SPI_FIRSTBIT_MSB;
    init.TIMode = SPI_TIMODE_DISABLE;
    init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    init.CRCPolynomial = {};
    init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    HAL_SPI_Init(&spi_handle_);

    // Enable the RXNE interrupt.
    spi_->CR2 |= SPI_CR2_RXNEIE;

    __HAL_SPI_ENABLE(&spi_handle_);
  }

  void ISR_HandleNssRise() {
    led2_.write(0);
    // Mark the transfer as completed if we actually started one.
    if (mode_ == kTransfer) {
      end_handler_(current_address_, rx_bytes_);
    }
    tx_bytes_ = 0;
    rx_bytes_ = 0;

    // We need to flush out the peripheral's FIFOs and get it ready to
    // start again.  Sigh... ST provides no way to do this aside from
    // a complete re-initialization.
    Init();

    mode_ = kInactive;
  }

  void ISR_HandleNssFall() {
    led2_.write(1);

    // Get ready to start receiving the address.
    mode_ = kWaitingAddress1;

    // Queue up our response for the address bytes.
    *(__IO uint16_t *)spi_->DR = 0x0000;
  }

  void ISR_SPI() {
    led1_.write(1);
    while (spi_->SR & SPI_SR_RXNE) {
      switch (mode_) {
        case kInactive: {
          // Just ignore this.
          (void) ReadRegister(&spi_->DR);
          break;
        }
        case kWaitingAddress1: {
          current_address_ = ReadRegister(&spi_->DR) << 8;
          mode_ = kWaitingAddress2;
          led2_.write(0);

          break;
        }
        case kWaitingAddress2: {
          current_address_ |= ReadRegister(&spi_->DR);
          led1_.write(0);
          buffer_ = start_handler_(current_address_);
          led1_.write(1);
          mode_ = kTransfer;

          ISR_PrepareTx();
          led2_.write(1);

          break;
        }
        case kTransfer: {
          const auto this_byte = ReadRegister(&spi_->DR);
          if (rx_bytes_ < buffer_.rx.size()) {
            buffer_.rx[rx_bytes_] = this_byte;
          }
          rx_bytes_++;
          ISR_PrepareTx();

          break;
        }
      }
    }
    led1_.write(0);
  }

  void ISR_PrepareTx() {
    while (spi_->SR & SPI_SR_TXE) {
      const auto this_byte =
          (tx_bytes_ < buffer_.tx.size()) ?
          buffer_.tx[tx_bytes_] : 0;
      WriteRegister(&spi_->DR, this_byte);
      tx_bytes_++;
    }
  }

  SPI_HandleTypeDef spi_handle_ = {};
  SPI_TypeDef* spi_ = nullptr;
  InterruptIn nss_;

  StartHandler start_handler_;
  EndHandler end_handler_;

  mjlib::micro::CallbackTable::Callback isr_;

  enum Mode {
    kInactive,
    kWaitingAddress1,
    kWaitingAddress2,
    kTransfer,
  };
  Mode mode_ = kInactive;
  uint16_t current_address_ = 0;

  Buffer buffer_;
  size_t tx_bytes_ = 0;
  ssize_t rx_bytes_ = 0;

  DigitalOut led1_{PF_0, 1};
  DigitalOut led2_{PF_1, 1};
};

int ParseDlc(uint32_t dlc_code) {
  if (dlc_code == FDCAN_DLC_BYTES_0) { return 0; }
  if (dlc_code == FDCAN_DLC_BYTES_1) { return 1; }
  if (dlc_code == FDCAN_DLC_BYTES_2) { return 2; }
  if (dlc_code == FDCAN_DLC_BYTES_3) { return 3; }
  if (dlc_code == FDCAN_DLC_BYTES_4) { return 4; }
  if (dlc_code == FDCAN_DLC_BYTES_5) { return 5; }
  if (dlc_code == FDCAN_DLC_BYTES_6) { return 6; }
  if (dlc_code == FDCAN_DLC_BYTES_7) { return 7; }
  if (dlc_code == FDCAN_DLC_BYTES_8) { return 8; }
  if (dlc_code == FDCAN_DLC_BYTES_12) { return 12; }
  if (dlc_code == FDCAN_DLC_BYTES_16) { return 16; }
  if (dlc_code == FDCAN_DLC_BYTES_20) { return 20; }
  if (dlc_code == FDCAN_DLC_BYTES_24) { return 24; }
  if (dlc_code == FDCAN_DLC_BYTES_32) { return 32; }
  if (dlc_code == FDCAN_DLC_BYTES_48) { return 48; }
  if (dlc_code == FDCAN_DLC_BYTES_64) { return 64; }
  mbed_die();
  return 0;
}

constexpr int kMaxSpiFrameSize = 70;
constexpr int kBufferItems = 6;

/// This presents the following SPI interface.
///
/// Each "register" points to a discontiguous block of data that
/// cannot be read or written piecemeal.
///
/// 0: Protocol version
///    byte 0: the constant value 1
/// 1: Interface type
///    byte 0: the constant value 1
/// 16: Receive status
///    byte 0-6 - Size of up to 6 received frames.  0 means no frame
///      is available.  this size includes the header, so is the total
///      number of bytes that must be read from register 17 to get a
///      complete frame
/// 17: Received frame: reading this register returns exactly one frame
///    from the received queue.
///   byte 0 - CAN bus (0/1/2) - 0 means this frame is not present
///   byte 1,2,3,4 - CAN ID, MSB first
///   byte 5 - size of remaining payload
///   byte 6+ payload
/// 18: Send a CAN frame
///   byte 0 - CAN bus (1/2)
///   byte 1,2,3,4 - CAN ID, MSB first
///   byte 5 - size of remaining payload
///   byte 6+ payload

class CanBridge {
 public:
  CanBridge(fw::FDCan* can1, fw::FDCan* can2,
            RegisterSPISlave::StartHandler start_handler,
            RegisterSPISlave::EndHandler end_handler)
      : can1_{can1},
        can2_{can2},
        start_handler_(start_handler),
        end_handler_(end_handler) {}

  void Poll() {
    auto can_poll = [&](int bus_id, auto* can) {
      const bool rx = can->Poll(&can_header_, rx_buffer_);
      if (!rx) {
        return;
      }

      auto* this_buf = [&]() -> CanReceiveBuf* {
        for (auto& buf : can_buf_) {
          if (!buf.active) { return &buf; }
        }
        return nullptr;
      }();

      if (this_buf == nullptr) {
        // Record an error.
        return;
      }

      this_buf->active = true;

      this_buf->data[0] = bus_id;
      const auto id = can_header_.Identifier;
      this_buf->data[1] = (id >> 24) & 0xff;
      this_buf->data[2] = (id >> 16) & 0xff;
      this_buf->data[3] = (id >> 8) & 0xff;
      this_buf->data[4] = (id >> 0) & 0xff;
      const int size = ParseDlc(can_header_.DataLength);
      this_buf->data[5] = size;
      this_buf->size = size;
      std::memcpy(&this_buf->data[6], rx_buffer_, size);

      // Insert this item into the receive queue.
      __disable_irq();
      [&]() {
        for (auto& item : can_rx_queue_) {
          if (item == nullptr) {
            item = this_buf;
            return;
          }
        }
        // Hmm, we couldn't.  Just throw it away.
        this_buf->active = false;
      }();
      __enable_irq();
    };

    if (can1_) {
      can_poll(1, can1_);
    }
    if (can2_) {
      can_poll(2, can2_);
    }

    // Look to see if we have anything to send.
    for (auto& spi_buf : spi_buf_) {
      if (spi_buf.ready_to_send) {
        // Let's send this out.
        const auto send_result = SendCan(&spi_buf);
        if (send_result == fw::FDCan::kNoSpace) {
          // Just finish this poll and try again later.
          return;
        }

        spi_buf.ready_to_send = false;
        spi_buf.active = false;
      }
    }
  }

 private:
  struct SpiReceiveBuf {
    char data[kMaxSpiFrameSize] = {};
    int size = 0;
    std::atomic<bool> active{false};
    std::atomic<bool> ready_to_send{false};
  };

  struct CanReceiveBuf {
    char data[kMaxSpiFrameSize] = {};
    int size = 0;
    std::atomic<bool> active{false};
  };

  fw::FDCan::SendResult SendCan(const SpiReceiveBuf* spi) {
    if (spi->size < 6) {
      // This is not big enough for a header.
      // TODO record an error.
      return fw::FDCan::kSuccess;
    }

    const int can_bus = spi->data[0];
    if (can_bus != 1 && can_bus != 2) {
      // TODO Record an error of some sort.
      return fw::FDCan::kSuccess;
    }
    const uint32_t id =
        (u8(spi->data[1]) << 24) |
        (u8(spi->data[2]) << 16) |
        (u8(spi->data[3]) << 8) |
        (u8(spi->data[4]));
    const int size = u8(spi->data[5]);
    if (size + 6 > spi->size) {
      // There is not enough data.  TODO Record an error.
      return fw::FDCan::kSuccess;
    }

    auto* const can = (can_bus == 1) ?
        can1_ :
        can2_;

    if (can) {
      return can->Send(id, std::string_view(&spi->data[6], size));
    } else {
      return fw::FDCan::kSuccess;
    }
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 0) {
      return {
        std::string_view("\x01", 1),
        {},
      };
    }
    if (address == 1) {
      return {
        std::string_view("\x01", 1),
        {},
      };
    }
    if (address == 16) {
      for (size_t i = 0; i < kBufferItems; i++) {
        const auto item = can_rx_queue_[i];
        address16_buf_[i] = (item == nullptr) ? 0 : (item->size + 6);
      }
      return {
        address16_buf_,
        {},
      };
    }
    if (address == 17) {
      if (!can_rx_queue_[0]) {
        current_can_buf_ = nullptr;
        return { {}, {} };
      }
      current_can_buf_ = can_rx_queue_[0];

      // Shift everything over.
      for (size_t i = 1; i < kBufferItems; i++) {
        can_rx_queue_[i - 1] = can_rx_queue_[i];
      }
      can_rx_queue_[kBufferItems - 1] = nullptr;

      return {
        std::string_view(current_can_buf_->data, current_can_buf_->size + 6),
        {},
      };
    }
    if (address == 18) {
      current_spi_buf_ = [&]() -> SpiReceiveBuf* {
        for (auto& buf : spi_buf_) {
          if (!buf.active) { return &buf; }
        }
        // All our buffers are full.  There must be something wrong on the CAN bus.
        //
        // TODO: Log an error.
        return nullptr;
      }();

      if (!current_spi_buf_) {
        return { {}, {} };
      } else {
        current_spi_buf_->active = true;

        return {
          {},
              mjlib::base::string_span(current_spi_buf_->data, kMaxSpiFrameSize),
        };
      }
    }

    if (start_handler_) {
      return start_handler_(address);
    }
    return { {}, {} };
  }

  void ISR_End(uint16_t address, int bytes) {
    if (current_can_buf_) {
      current_can_buf_->active = false;
      current_can_buf_ = nullptr;
    }

    if (current_spi_buf_) {
      current_spi_buf_->size = bytes;
      current_spi_buf_->ready_to_send = true;
      current_spi_buf_ = nullptr;
    }

    if (address != 0 && address != 1 &&
        address != 16 && address != 17 && address != 18) {
      if (end_handler_) {
        end_handler_(address, bytes);
      }
    }
  }

  RegisterSPISlave spi_{
    PA_7, PA_6, PA_5, PA_4,
        [this](uint16_t address) {
          return this->ISR_Start(address);
        },
        [this](uint16_t address, int bytes) {
          return this->ISR_End(address, bytes);
        }};

  fw::FDCan* can1_ = nullptr;
  fw::FDCan* can2_ = nullptr;

  RegisterSPISlave::StartHandler start_handler_;
  RegisterSPISlave::EndHandler end_handler_;

  SpiReceiveBuf spi_buf_[kBufferItems] = {};
  SpiReceiveBuf* current_spi_buf_ = nullptr;

  CanReceiveBuf can_buf_[kBufferItems] = {};
  CanReceiveBuf* can_rx_queue_[kBufferItems] = {};
  CanReceiveBuf* current_can_buf_ = nullptr;

  FDCAN_RxHeaderTypeDef can_header_ = {};
  char rx_buffer_[64] = {};

  char address16_buf_[kBufferItems] = {};
};

class CanApplication {
 public:
  void Poll() {
    bridge_.Poll();
  }

  void PollMillisecond() {
  }

  fw::FDCan can1_{[]() {
      fw::FDCan::Options o;
      o.td = PB_4;
      o.rd = PB_3;
      o.slow_bitrate = 1000000;
      o.fast_bitrate = 5000000;
      o.fdcan_frame = true;
      o.bitrate_switch = true;
      o.automatic_retransmission = true;
      return o;
    }()
  };
  fw::FDCan can2_{[]() {
      fw::FDCan::Options o;
      o.td = PA_12;
      o.rd = PA_11;
      o.slow_bitrate = 1000000;
      o.fast_bitrate = 5000000;
      o.fdcan_frame = true;
      o.bitrate_switch = true;
      o.automatic_retransmission = true;
      return o;
    }()
  };

  CanBridge bridge_{&can1_, &can2_, {}, {}};
};


void SetupClock() {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 170 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  //  85 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  //  85 MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
    mbed_die();
  }

  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      mbed_die();
    }
  }
}


class AuxApplication {
 public:
  struct ImuRegister {
    uint16_t present = 0;
    float gx_dps = 0;
    float gy_dps = 0;
    float gz_dps = 0;
    float ax_mps2 = 0;
    float ay_mps2 = 0;
    float az_mps2 = 0;

    ImuRegister() {}
    ImuRegister(const Bmi088::Data& data) {
      present = 1;
      gx_dps = data.rate_dps.x();
      gy_dps = data.rate_dps.y();
      gz_dps = data.rate_dps.z();
      ax_mps2 = data.accel_mps2.x();
      ay_mps2 = data.accel_mps2.y();
      az_mps2 = data.accel_mps2.z();
    }
  } __attribute__((packed));

  struct AttitudeRegister {
    uint8_t present = 0;
    uint8_t update_time_10us = 0;
    float w = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    float bx_dps = 0;
    float by_dps = 0;
    float bz_dps = 0;
  } __attribute__((packed));

  AuxApplication(mjlib::micro::Pool* pool, fw::MillisecondTimer* timer)
      : pool_(pool), timer_(timer) {
    setup_data_ = imu_.setup_data();
  }

  void Poll() {
    bridge_.Poll();
    if (imu_.data_ready()) {
      DoImu();
    }
  }

  void PollMillisecond() {
    nrf_.PollMillisecond();
    auto nrf_regs = nrf_.register_map();
    std::memcpy(nrf_registers_, nrf_regs.data(), nrf_regs.size());
  }

 private:
  void DoImu() {
    const auto start = timer_->read_us();

    const auto data = imu_.read_data();
    ImuRegister my_data{data};

    attitude_reference_.ProcessMeasurement(
        0.001f,
        (M_PI / 180.0f) * data.rate_dps,
        data.accel_mps2);

    AttitudeRegister my_att;
    my_att.present = 1;
    const Quaternion att = attitude_reference_.attitude();
    my_att.w = att.w();
    my_att.x = att.x();
    my_att.y = att.y();
    my_att.z = att.z();
    const Point3D rate_dps = (180.0 / M_PI) * attitude_reference_.rate_rps();
    my_att.bx_dps = rate_dps.x();
    my_att.by_dps = rate_dps.y();
    my_att.bz_dps = rate_dps.z();

    const auto end = timer_->read_us();
    my_att.update_time_10us = std::min<decltype(end)>(255, (end - start) / 10);

    __disable_irq();
    data_ = my_data;
    attitude_ = my_att;
    __enable_irq();
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 32) {
      return {
        std::string_view("\x20", 1),
        {},
      };
    }
    if (address == 33) {
      isr_data_ = data_;
      data_ = {};
      return {
        std::string_view(reinterpret_cast<const char*>(&isr_data_),
                         sizeof(isr_data_)),
        {},
      };
    }
    if (address == 34) {
      isr_attitude_ = attitude_;
      attitude_ = {};
      return {
        std::string_view(
            reinterpret_cast<const char*>(&isr_attitude_),
            sizeof(isr_attitude_)),
        {},
      };
    }
    if (address == 48) {
      return {
        {nrf_registers_, sizeof(nrf_registers_)},
        {},
      };
    }
    return {};
  }

  void ISR_End(uint16_t address, int bytes) {
  }

  mjlib::micro::Pool* const pool_;
  MillisecondTimer* const timer_;

  fw::FDCan can1_{[]() {
      fw::FDCan::Options o;
      o.td = PA_12;
      o.rd = PA_11;
      o.slow_bitrate = 125000;
      o.fast_bitrate = 125000;
      o.fdcan_frame = false;
      o.bitrate_switch = false;
      o.automatic_retransmission = true;
      return o;
    }()
  };

  CanBridge bridge_{
    &can1_, nullptr,
        [this](uint16_t address) {
      return this->ISR_Start(address);
    },
        [this](uint16_t address, int bytes) {
          this->ISR_End(address, bytes);
        }
  };

  Bmi088 imu_{pool_, timer_, []() {
      Bmi088::Options options;
      options.mosi = PB_15;
      options.miso = PB_14;
      options.sck = PB_13;
      options.acc_cs = PA_9;
      options.gyro_cs = PB_12;
      options.acc_int = PA_10;
      options.gyro_int = PA_8;

      options.rate_hz = 800;
      options.gyro_max_dps = 1000;
      options.accel_max_g = 6;

      return options;
    }()
  };

  ImuRegister data_;
  ImuRegister isr_data_;
  Bmi088::SetupData setup_data_;
  AttitudeReference attitude_reference_;
  AttitudeRegister attitude_;
  AttitudeRegister isr_attitude_;

  Nrf24l01 nrf_{pool_, timer_, []() {
      Nrf24l01::Options options;
      options.mosi = PB_5_ALT0;
      options.miso = PB_4_ALT0;
      options.sck = PB_3_ALT0;
      options.cs = PA_15;
      options.irq = PB_7;
      options.ce = PB_6;
      return options;
    }()
  };

  char nrf_registers_[32] = {};
};
}
}

int main(void) {
  DigitalIn id_select1(PC_14);
  DigitalIn id_select2(PC_15);

  fw::SetupClock();

  mjlib::micro::SizedPool<16384> pool;
  fw::MillisecondTimer timer;

  auto run = [&](auto& app) {
    uint32_t last_ms = 0;
    while (true) {
      app.Poll();
      const auto now = timer.read_ms();
      if (now != last_ms) {
        app.PollMillisecond();
        last_ms = now;
      }
    }
  };

  // See what thing we're going to do.
  if (id_select1.read() == 1) {
    // We are the AUX processor.
    fw::AuxApplication application{&pool, &timer};

    run(application);
  } else {
    fw::CanApplication application;

    run(application);
  }
}

extern "C" {
void abort() {
  mbed_die();
}

void mbed_die(void) {
  // Flash an LED that exists.
  gpio_t led;
  gpio_init_out(&led, PF_0);

  for (;;) {
    gpio_write(&led, 0);
    wait_ms(200);
    gpio_write(&led, 1);
    wait_ms(200);
  }
}
}
