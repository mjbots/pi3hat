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
  Microphone(fw::MillisecondTimer*, PinName adc_name) {
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
    if (HAL_DAC_Init(&dac_) != HAL_OK) {
      mbed_die();
    }

    {
      DAC_ChannelConfTypeDef config = {};
      config.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
      config.DAC_DMADoubleDataMode = DISABLE;
      config.DAC_SignedFormat = DISABLE;
      config.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
      config.DAC_Trigger = DAC_TRIGGER_NONE;
      config.DAC_Trigger2 = DAC_TRIGGER_NONE;
      config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;  // probably not needed, since we're just going to an op-amp?
      config.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
      config.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
      config.DAC_TrimmingValue = {};
      config.DAC_SampleAndHoldConfig = {};

      const auto channel = DAC_CHANNEL_1;

      HAL_DAC_ConfigChannel(&dac_, &config, channel);
      HAL_DAC_Start(&dac_, channel);

      HAL_DAC_SetValue(&dac_, channel, DAC_ALIGN_12B_R, 2048);
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

    adc_.Instance = ADC5;
    adc_.State = HAL_ADC_STATE_RESET;
    adc_.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    adc_.Init.Resolution = ADC_RESOLUTION_12B;
    adc_.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc_.Init.ScanConvMode = DISABLE;
    adc_.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    adc_.Init.LowPowerAutoWait = DISABLE;
    adc_.Init.ContinuousConvMode = DISABLE;
    adc_.Init.NbrOfConversion = 1;
    adc_.Init.DiscontinuousConvMode = DISABLE;
    adc_.Init.NbrOfDiscConversion = 0;
    adc_.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_CC1;
    adc_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc_.Init.DMAContinuousRequests = DISABLE;
    adc_.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

    __HAL_RCC_ADC345_CLK_ENABLE();

    if (HAL_ADC_Init(&adc_) != HAL_OK) {
      mbed_die();
    }

    if (!HAL_ADCEx_Calibration_GetValue(&adc_, ADC_SINGLE_ENDED)) {
        HAL_ADCEx_Calibration_Start(&adc_, ADC_SINGLE_ENDED);
    }

    ADC_ChannelConfTypeDef adc_conf;
    adc_conf.Rank = ADC_REGULAR_RANK_1;
    adc_conf.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    adc_conf.SingleDiff = ADC_SINGLE_ENDED;
    adc_conf.OffsetNumber = ADC_OFFSET_NONE;
    adc_conf.Offset = 0;
    adc_conf.Channel = ADC_CHANNEL_5;

    HAL_ADC_ConfigChannel(&adc_, &adc_conf);
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
    HAL_ADC_Start(&adc_);
    HAL_ADC_PollForConversion(&adc_, 10);
    const auto adc_value = HAL_ADC_GetValue(&adc_) >> 8;
    HAL_ADC_Stop(&adc_);

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
