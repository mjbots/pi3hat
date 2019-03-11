// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include "mbed.h"

#include "fw/millisecond_timer.h"
#include "fw/stm32_serial.h"

int main(void) {
  DigitalOut led1(PB_3, 1);
  DigitalOut led2(PB_4, 1);

  fw::Stm32Serial master_serial{[]() {
      fw::Stm32Serial::Options options;
      options.tx = PA_9;
      options.rx = PA_10;
      options.baud_rate = 3000000;
      return options;
    }()};
  auto* const master_uart = master_serial.uart();

  master_uart->CR1 &= ~USART_CR1_UE;
  master_uart->CR1 |= USART_CR1_TE | USART_CR1_RE;
  // master_uart->CR3 |= USART_CR3_OVRDIS;
  master_uart->CR1 |= USART_CR1_UE;

  fw::Stm32Serial slave_serial{[]() {
      fw::Stm32Serial::Options options;
      options.tx = PA_2;
      options.rx = PA_3;
      options.baud_rate = 3000000;
      return options;
    }()};
  auto* const slave_uart = slave_serial.uart();

  slave_uart->CR1 &= ~USART_CR1_UE;
  slave_uart->CR3 |= USART_CR3_DEM;
  // slave_uart->CR3 |= USART_CR3_OVRDIS;
  slave_uart->CR1 |= USART_CR1_TE | USART_CR1_RE;
  slave_uart->CR1 =
      (slave_uart->CR1 & ~USART_CR1_DEAT) | (6 << USART_CR1_DEAT_Pos);
  slave_uart->CR1 =
      (slave_uart->CR1 & ~USART_CR1_DEDT) | (6 << USART_CR1_DEDT_Pos);
  slave_uart->CR1 |= USART_CR1_UE;

  pinmap_pinout(PA_1, PinMap_UART_RTS);

  fw::MillisecondTimer timer;
  uint32_t flash_count = 0;

    // We don't want *any* interrupts while we are running.  Our inner
    // loop runs at about 1us per iteration, and any glitches can
    // cause a receiver overrun.
  __disable_irq();

  while (true) {
    if (master_uart->ISR & USART_ISR_RXNE) {
      const uint8_t value = master_uart->RDR;
      // Wait for any previous write to complete.
      while ((slave_uart->ISR & USART_ISR_TXE) == 0);
      slave_uart->TDR = value;
    }
    if (slave_uart->ISR & USART_ISR_RXNE) {
      const int value = slave_uart->RDR;
      while ((master_uart->ISR & USART_ISR_TXE) == 0);
      master_uart->TDR = value;
    }
    if (true) {
      const uint32_t time_s = timer.read_us() >> 20;
      if (time_s != flash_count) {
        flash_count = time_s;
        led1.write(!led1.read());
      }
    }
    led2.write(!led2.read());
  }
}
