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

#include "mbed.h"
#include "PeripheralPins.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/string_span.h"

#include "fw/millisecond_timer.h"

namespace fw {

/// Implements a register based SPI slave.
///
/// 8 bit transactions are assumed, with a 8 bit address scheme.  The
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

  struct Pins {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sclk = NC;
    PinName ssel = NC;
    PinName status_led = NC;

    DMA_Channel_TypeDef* rx_dma = DMA1_Channel2;
    DMA_Channel_TypeDef* tx_dma = DMA1_Channel1;
  };

  RegisterSPISlave(fw::MillisecondTimer* timer,
                   const Pins& pins,
                   StartHandler start_handler, EndHandler end_handler)
      : timer_{timer},
        dma_rx_{pins.rx_dma},
        dma_tx_{pins.tx_dma},
        nss_{pins.ssel},
        status_led_{pins.status_led, 1},
        start_handler_(start_handler),
        end_handler_(end_handler) {
    g_impl_ = this;

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    spi_ = [&]() {
      const auto spi_mosi = pinmap_peripheral(pins.mosi, PinMap_SPI_MOSI);
      const auto spi_miso = pinmap_peripheral(pins.miso, PinMap_SPI_MISO);
      const auto spi_sclk = pinmap_peripheral(pins.sclk, PinMap_SPI_SCLK);
      const auto spi_ssel = pinmap_peripheral(pins.ssel, PinMap_SPI_SSEL);
      return reinterpret_cast<SPI_TypeDef*>(
          merge(spi_mosi, merge(spi_miso, merge(spi_sclk, spi_ssel))));
    }();

    MJ_ASSERT(spi_ != nullptr);

    const auto channel_index = [&](DMA_Channel_TypeDef* channel) -> uint32_t {
      if (channel < DMA2_Channel1) {
        return ((u32(channel) - u32(DMA1_Channel1)) /
                (u32(DMA1_Channel2) - u32(DMA1_Channel1))) << 2;
      }
      return ((u32(channel) - u32(DMA2_Channel1)) /
              (u32(DMA2_Channel2) - u32(DMA2_Channel1))) << 2;
    };

    const auto select_dmamux = [&](DMA_Channel_TypeDef* channel) {
      const auto base = (channel < DMA2_Channel1) ? DMAMUX1_Channel0 :
#if defined (STM32G471xx) || defined (STM32G473xx) || defined (STM32G474xx) || defined (STM32G483xx) || defined (STM32G484xx)
      DMAMUX1_Channel8;
#elif defined (STM32G431xx) || defined (STM32G441xx) || defined (STM32GBK1CB)
      DMAMUX1_Channel6;
#else
      DMAMUX1_Channel7;
#endif /* STM32G4x1xx) */
      return reinterpret_cast<DMAMUX_Channel_TypeDef*>(
          u32(base) + (channel_index(channel) >> 2U) *
          (u32(DMAMUX1_Channel1) - u32(DMAMUX1_Channel0)));
    };

    dmamux_rx_ = select_dmamux(dma_rx_);
    dmamux_tx_ = select_dmamux(dma_tx_);

    dma_rx_->CCR =
        DMA_PERIPH_TO_MEMORY |
        DMA_PINC_DISABLE |
        DMA_MINC_ENABLE |
        DMA_PDATAALIGN_BYTE |
        DMA_MDATAALIGN_BYTE |
        DMA_PRIORITY_HIGH;
    dma_tx_->CCR =
        DMA_MEMORY_TO_PERIPH |
        DMA_PINC_DISABLE |
        DMA_MINC_ENABLE |
        DMA_PDATAALIGN_BYTE |
        DMA_MDATAALIGN_BYTE |
        DMA_PRIORITY_HIGH;
    dmamux_rx_->CCR = GetSpiRxRequest(spi_) & DMAMUX_CxCR_DMAREQ_ID;
    dmamux_tx_->CCR = GetSpiTxRequest(spi_) & DMAMUX_CxCR_DMAREQ_ID;

    pinmap_pinout(pins.mosi, PinMap_SPI_MOSI);
    pinmap_pinout(pins.miso, PinMap_SPI_MISO);
    pinmap_pinout(pins.sclk, PinMap_SPI_SCLK);
    pin_mode(pins.sclk, PullDown);
    pin_mode(pins.ssel, PullUp);

    Init();

    nss_.rise(callback(this, &RegisterSPISlave::ISR_HandleNssRise));
    nss_.fall(callback(this, &RegisterSPISlave::ISR_HandleNssFall));

    const auto irq = GetSpiIrq(spi_);

    NVIC_SetVector(
        irq, reinterpret_cast<uint32_t>(&RegisterSPISlave::GlobalInterruptSPI));

    HAL_NVIC_SetPriority(irq, 0, 0);
    HAL_NVIC_EnableIRQ(irq);
  }

  static void GlobalInterruptSPI() {
    g_impl_->ISR_SPI();
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
  }

  void PollMillisecond() {
    // We might have had our LED set high by a NSS fall.  Here we just
    // turn it off again.  That results in a pulse pattern if the SPI
    // bus is being used.
    status_led_.write(1);
  }

  void ISR_HandleNssRise() {
    // Stop all DMA and figure out how much was transferred.
    dma_tx_->CCR &= ~(DMA_CCR_EN);
    dma_rx_->CCR &= ~(DMA_CCR_EN);
    spi_->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    // TODO: Clear any errors.

    // Figure out how many we transferred.
    const size_t rx_bytes = [&]() {
      if (static_cast<int>(buffer_.tx.size()) > buffer_.rx.size()) {
        return buffer_.tx.size() - dma_tx_->CNDTR;
      } else {
        return buffer_.rx.size() - dma_rx_->CNDTR;
      }
    }();

    // Mark the transfer as completed if we actually started one.
    if (mode_ == kTransfer) {
      end_handler_(current_address_, rx_bytes);
    }

    // We need to flush out the peripheral's FIFOs and get it ready to
    // start again.  Sigh... ST provides no way to do this aside from
    // a complete re-initialization.
    Init();

    mode_ = kInactive;
  }

  void ISR_HandleNssFall() {
    status_led_.write(0);

    __HAL_SPI_ENABLE(&spi_handle_);

    // Enable the RXNE interrupt.
    spi_->CR2 |= SPI_CR2_RXNEIE;

    // Get ready to start receiving the address.
    mode_ = kWaitingAddress;

    // Queue up our response for the address byte.
    *(__IO uint16_t *)spi_->DR = 0x0000;
  }

  void ISR_SPI() {
    if (mode_ == kTransfer) { return; }

    if (spi_->SR & SPI_SR_RXNE) {
      switch (mode_) {
        case kInactive: {
          // Just ignore this.
          (void) ReadRegister(&spi_->DR);
          break;
        }
        case kWaitingAddress: {
          current_address_ = ReadRegister(&spi_->DR);
          buffer_ = start_handler_(current_address_);
          ISR_StartDMA();
          mode_ = kTransfer;

          // We won't be using the SPI interrupts going forward.
          spi_->CR2 &= ~SPI_CR2_RXNEIE;

          break;
        }
        case kTransfer: {
          break;
        }
      }
    }
  }

  void ISR_StartDMA() {
    uint32_t en = 0;

    dma_tx_->CNDTR = buffer_.tx.size();

    if (buffer_.tx.size()) {
      en |= SPI_CR2_TXDMAEN;

      dma_tx_->CPAR = u32(&spi_->DR);
      dma_tx_->CMAR = u32(&buffer_.tx[0]);
      dma_tx_->CCR |= DMA_CCR_EN;
    }

    dma_rx_->CNDTR = buffer_.rx.size();

    if (buffer_.rx.size()) {
      en |= SPI_CR2_RXDMAEN;

      dma_rx_->CPAR = u32(&spi_->DR);
      dma_rx_->CMAR = u32(&buffer_.rx[0]);
      dma_rx_->CCR |= DMA_CCR_EN;
    }

    spi_->CR2 |= en;
  }

  template <typename T>
  static uint8_t ReadRegister(T* value) {
    return *reinterpret_cast<__IO uint8_t*>(value);
  }

  template <typename T>
  static void WriteRegister(T* reg, uint8_t value) {
    *reinterpret_cast<__IO uint8_t*>(reg) = value;
  }

  template <typename T>
  static T merge(T a, T b) {
    if (a == b) { return a; }
    return T();
  }

  static void EnableSPI(SPI_TypeDef* spi) {
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

  static IRQn_Type GetSpiIrq(SPI_TypeDef* spi) {
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

  uint32_t GetSpiTxRequest(SPI_TypeDef* spi) {
    switch (u32(spi)) {
      case SPI_1: return DMA_REQUEST_SPI1_TX;
      case SPI_2: return DMA_REQUEST_SPI2_TX;
      case SPI_3: return DMA_REQUEST_SPI3_TX;
    }
    mbed_die();
  }

  uint32_t GetSpiRxRequest(SPI_TypeDef* spi) {
    switch (u32(spi)) {
      case SPI_1: return DMA_REQUEST_SPI1_RX;
      case SPI_2: return DMA_REQUEST_SPI2_RX;
      case SPI_3: return DMA_REQUEST_SPI3_RX;
    }
    mbed_die();
  }

  template <typename T>
  static uint32_t u32(T value) {
    return reinterpret_cast<uint32_t>(value);
  }

  fw::MillisecondTimer* const timer_;
  SPI_HandleTypeDef spi_handle_ = {};
  SPI_TypeDef* spi_ = nullptr;

  DMA_Channel_TypeDef* const dma_rx_;
  DMA_Channel_TypeDef* const dma_tx_;
  DMAMUX_Channel_TypeDef* dmamux_rx_ = nullptr;
  DMAMUX_Channel_TypeDef* dmamux_tx_ = nullptr;

  InterruptIn nss_;
  DigitalOut status_led_;

  StartHandler start_handler_;
  EndHandler end_handler_;

  enum Mode {
    kInactive,
    kWaitingAddress,
    kTransfer,
  };
  Mode mode_ = kInactive;
  uint16_t current_address_ = 0;

  Buffer buffer_;

  static RegisterSPISlave* g_impl_;
};

}
