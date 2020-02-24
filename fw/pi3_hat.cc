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

#include "mbed.h"

#include "fw/millisecond_timer.h"

int main(void) {
  DigitalOut led1(PF_0, 1);
  DigitalOut led2(PF_1, 1);

  fw::MillisecondTimer timer;
  uint32_t flash_count = 0;

    // We don't want *any* interrupts while we are running.  Our inner
    // loop runs at about 1us per iteration, and any glitches can
    // cause a receiver overrun.
  __disable_irq();

  while (true) {
    const uint32_t time_s = timer.read_us() >> 20;
    led1.write(time_s % 2);
  }
}
