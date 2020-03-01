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

#include "PinNames.h"

#include "mjlib/micro/pool_ptr.h"

#include "fw/millisecond_timer.h"

namespace fw {

class Nrf24l01 {
 public:
  struct Options {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;
    PinName irq = NC;
    PinName ce = NC;

    Options() {}
  };

  Nrf24l01(mjlib::micro::Pool*, MillisecondTimer*, const Options& = Options());
  ~Nrf24l01();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
