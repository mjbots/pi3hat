// Copyright 2021 Josh Pieper, jjp@pobox.com.
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

#include "mbed.h"
#include "PeripheralPins.h"
#include "PinNames.h"

#include "mjlib/base/assert.h"

namespace fw {

class Stm32PwmOut {
 public:
  struct Options {
    uint32_t prescaler = 1;
    uint32_t period = 10000;
    uint32_t on_counts = 5000;

    Options() {}
  };

  Stm32PwmOut(PinName pin, const Options& options = Options())
      : pwm_(pin) {
    // pwm_.period_us(1000);
    // pwm_.pulsewidth_us(500);

    const auto pwm_timer = pinmap_peripheral(pin, PinMap_PWM);
    MJ_ASSERT(pwm_timer != 0);
    timer_ = reinterpret_cast<TIM_TypeDef*>(pwm_timer);

    ccr_ = FindCcr(timer_, pin);
    timer_->CR1 =
        // MODE
        (0 << TIM_CR1_CMS_Pos) |  // edge aligned mode

        // ARR register is buffered.
        TIM_CR1_ARPE |

        // And turn on the counter.
        TIM_CR1_CEN;

    timer_->PSC = options.prescaler - 1; // prescaler
    timer_->ARR = options.period;
    *ccr_ = options.on_counts;
  }

  void set_on_counts(uint32_t counts) {
    *ccr_ = counts;
  }

  static volatile uint32_t* FindCcr(TIM_TypeDef* timer, PinName pin) {
    const auto function = pinmap_function(pin, PinMap_PWM);

    const auto inverted = STM_PIN_INVERTED(function);
    MJ_ASSERT(!inverted);

    const auto channel = STM_PIN_CHANNEL(function);

    switch (channel) {
      case 1: { return &timer->CCR1; }
      case 2: { return &timer->CCR2; }
      case 3: { return &timer->CCR3; }
      case 4: { return &timer->CCR4; }
    }
    MJ_ASSERT(false);
    return nullptr;
  }

  // This is used just to initialize the pin as output and PWM mode.
  PwmOut pwm_;

  TIM_TypeDef* timer_ = nullptr;
  volatile uint32_t* ccr_ = nullptr;
};

}
