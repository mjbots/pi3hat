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

#include <string>

#include <pybind11/pybind11.h>

#include "mjbots/pi3hat/pi3hat.h"

namespace {
std::string dummy() {
  return "I'm not a dummy?";
}
}

PYBIND11_MODULE(_pi3hat_router, m) {
  m.doc() = "implementation of pi3hat specific moteus functionality";

  m.def("dummy", &dummy, "just do the thing");
}
