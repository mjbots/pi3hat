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

namespace fw {

/// Exposes the microphone through the following SPI registers.
///
/// 80: Protocol version:
///  byte0: 0x20
/// 81: Microphone data
///  byte0: number of bytes of data
///  byte1-255: Up to 254 bytes of data
class Microphone {
 public:
  Microphone(fw::MillisecondTimer* timer, PinName adc_name) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    {
      // Set PB10 to be alternate function for the opamp.
      GPIO_InitTypeDef init = {};
      init.Pin = GPIO_PIN_10;
      init.Mode = GPIO_MODE_ANALOG;
      init.Pull = {};
      init.Speed = {};
      init.Alternate = {};
      HAL_GPIO_Init(GPIOB, &init);
    }

    __HAL_RCC_DAC4_CLK_ENABLE();

    dac_.Instance = DAC4;

    {
      DAC4->MCR = (
          (2 << DAC_MCR_HFSEL_Pos) | // High frequency mode
          (3 << DAC_MCR_MODE1_Pos) | // on chip peripherals w/ buffer disabled
          0);

      DAC4->CR = (
          (DAC_CR_EN1) | // enable channel 1
          0);

      // tWAKEUP is defined as max 7.5us
      timer->wait_us(10);

      DAC4->DHR12R1 = 2048;
    }

    opamp_.Instance = OPAMP4;
    opamp_.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
    opamp_.Init.Mode = OPAMP_PGA_MODE;
    opamp_.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0; // PB10
    opamp_.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;  // DAC4_CH1
    opamp_.Init.InternalOutput = ENABLE;
    opamp_.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    opamp_.Init.InvertingInputSecondary = {};
    opamp_.Init.NonInvertingInputSecondary = {};
    opamp_.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
    opamp_.Init.PgaGain = OPAMP_PGA_GAIN_32_OR_MINUS_31;
    opamp_.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    opamp_.Init.TrimmingValueP = {};
    opamp_.Init.TrimmingValueN = {};

    if (HAL_OPAMP_Init(&opamp_) != HAL_OK) {
      mbed_die();
    }

    HAL_OPAMP_Start(&opamp_);

    __HAL_RCC_ADC345_CLK_ENABLE();

    adc_.Instance = ADC5;
    auto* adc = adc_.Instance;

    // Initialize the ADC.  First, we have to disable to ensure a
    // known state.
    if (adc->CR & ADC_CR_ADEN) {
      adc->CR |= ADC_CR_ADDIS;
      while (adc->CR & ADC_CR_ADEN);
    }

    ADC345_COMMON->CCR =
        (1 << ADC_CCR_PRESC_Pos);  // Prescaler / 2

    // 20.4.6: ADC Deep power-down mode startup procedure
    adc->CR &= ~ADC_CR_DEEPPWD;
    adc->CR |= ADC_CR_ADVREGEN;
    timer->wait_us(20);
    adc->CR |= ADC_CR_ADCAL;
    while ((adc->CR & ADC_CR_ADCAL));

    timer->wait_us(1);

    // 20.4.9: Software procedure to enable the ADC
    adc->ISR |= ADC_ISR_ADRDY;
    adc->CR |= ADC_CR_ADEN;

    while (!(adc->ISR & ADC_ISR_ADRDY));

    adc->ISR |= ADC_ISR_ADRDY;

    const auto sqr = 5;  // adc_IN5 is the output of the opamp

    adc->SQR1 =
        (1 << ADC_SQR1_L_Pos) |  // length 1
        (sqr << ADC_SQR1_SQ1_Pos);

    auto make_cycles = [](auto v) {
      return
        (v << 0) |
        (v << 3) |
        (v << 6) |
        (v << 9) |
        (v << 12) |
        (v << 15) |
        (v << 18) |
        (v << 21) |
        (v << 24);
    };
    const uint32_t all_aux_cycles = make_cycles(4); // 47 cycles

    adc->SMPR1 = all_aux_cycles;
    adc->SMPR2 = all_aux_cycles;
  }

  class DisableIrqs {
   public:
    DisableIrqs() {
      __disable_irq();
    }

    ~DisableIrqs() {
      __enable_irq();
    }
  };

  void PollMillisecond() {
    auto* adc = adc_.Instance;
    adc->CR |= ADC_CR_ADSTART;
    while ((adc->ISR & ADC_ISR_EOC) == 0);
    adc->ISR |= ADC_ISR_EOC;
    const auto adc_value = adc->DR >> 4;
    adc->CR |= ADC_CR_ADSTP;
    while (adc->CR & ADC_CR_ADSTP);

    DisableIrqs disable_irqs;

    auto* data = current_data_.load();

    // Simple hack.  Just sample at 1kHz for now in this poll call.
    // We should get the ADC to free run with DMA dumping its data
    // somewhere so we can sample at a more reasonable 22kHz or 44kHz.
    if (data->size >= sizeof(data->data)) {
      // Overrun!
      return;
    }

    data->data[data->size++] = adc_value;
  }

  RegisterSPISlave::Buffer ISR_Start(uint16_t address) {
    if (address == 80) {
      return {
        std::string_view("\x20", 1),
        {},
      };
    }
    if (address == 81) {
      auto* to_read = current_data_.load();
      current_data_.store(to_read == &data_[0] ? &data_[1] : &data_[0]);
      (*current_data_).size = 0;
      return {
        std::string_view(reinterpret_cast<const char*>(to_read), to_read->size + 1),
        {},
      };
    }
    return {};
  }

  void ISR_End(uint16_t address, int bytes) {
  }

 private:
  DAC_HandleTypeDef dac_ = {};
  OPAMP_HandleTypeDef opamp_ = {};
  ADC_HandleTypeDef adc_ = {};

  struct Data {
    uint8_t size = 0;
    uint8_t data[254] = {};
  } __attribute__((packed)) ;

  Data data_[2] = {};
  std::atomic<Data*> current_data_{&data_[0]};

};

}
