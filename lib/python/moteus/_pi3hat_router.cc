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

#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include "mjbots/pi3hat/pi3hat.h"
#include "mjbots/moteus/realtime.h"

namespace py = pybind11;
namespace pi3hat = mjbots::pi3hat;

namespace {
struct SingleCan {
  int id = 0;
  std::string data;
  int bus = 0;
  bool expect_reply = false;
};

struct Input {
  std::vector<SingleCan> tx_can;
};

struct Output {
  std::vector<SingleCan> rx_can;
};

class Pi3HatRouter {
 public:
  struct Options : pi3hat::Pi3Hat::Configuration {
    int cpu = 3;
  };

  Pi3HatRouter(const Options& options)
      : options_(options),
        thread_(std::bind(&Pi3HatRouter::CHILD_Run, this)) {
  }

  ~Pi3HatRouter() {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      done_ = true;
      condition_.notify_one();
    }
    thread_.join();
  }

  using CallbackFunction = std::function<void (Output)>;

  void Cycle(const Input& input, CallbackFunction callback) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (active_) {
      throw std::logic_error("cannot invoke multiple cycles at the same time");
    }

    callback_ = std::move(callback);
    active_ = true;
    PARENT_PopulateInput(input);

    condition_.notify_all();
  }

 private:
  void PARENT_PopulateInput(const Input& input) {
    tx_can_.resize(input.tx_can.size());
    for (size_t i = 0; i < input.tx_can.size(); i++) {
      auto& out = tx_can_[i];
      auto& in = input.tx_can[i];

      out.id = in.id;
      out.size = in.data.size();
      std::memcpy(&out.data[0], in.data.data(), out.size);
      out.bus = in.bus;
      out.expect_reply = in.expect_reply;
    }
    rx_can_.resize(tx_can_.size() * 2);
  }

  void CHILD_Run() {
    mjbots::moteus::ConfigureRealtime(options_.cpu);
    pi3hat_.reset(new pi3hat::Pi3Hat(options_));

    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!active_) {
          condition_.wait(lock);
          if (done_) { return; }
          if (!active_) { continue; }
        }
      }

      auto output = CHILD_Cycle();
      CallbackFunction callback_copy;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        active_ = false;
        std::swap(callback_copy, callback_);
      }
      callback_copy(output);
    }
  }

  Output CHILD_Cycle() {
    pi3hat::Pi3Hat::Input input;
    input.tx_can = { tx_can_.data(), tx_can_.size() };
    input.rx_can = { rx_can_.data(), rx_can_.size() };

    Output result;

    const auto output = pi3hat_->Cycle(input);

    for (size_t i = 0; i < output.rx_can_size; i++) {
      SingleCan out;
      const auto& in = rx_can_[i];
      out.id = in.id;
      out.data = std::string(reinterpret_cast<const char*>(&in.data[0]), in.size);
      out.bus = in.bus;
      result.rx_can.push_back(std::move(out));
    }
    return result;
  }

  const Options options_;

  /// This block of variables are all controlled by the mutex.
  std::mutex mutex_;
  std::condition_variable condition_;
  bool active_ = false;
  bool done_ = false;
  CallbackFunction callback_;

  std::thread thread_;

  // Used in the child thread.
  std::unique_ptr<pi3hat::Pi3Hat> pi3hat_;


  // These are populated in the parent thread when active_ == false,
  // and from the child thread when active_ == true.
  //
  // We leave these around just so that we don't have to repeatedly
  // allocate and also to make lifetime management easier.
  std::vector<pi3hat::CanFrame> tx_can_;
  std::vector<pi3hat::CanFrame> rx_can_;
};
}

PYBIND11_MODULE(_pi3hat_router, m) {
  m.doc() = "implementation of pi3hat specific moteus functionality";

  py::class_<Pi3HatRouter::Options>(m, "Pi3HatRouterOptions")
      .def(py::init<>())
      .def_readwrite("cpu", &Pi3HatRouter::Options::cpu)
      .def_readwrite("spi_speed_hz", &Pi3HatRouter::Options::spi_speed_hz)
      ;

  py::class_<SingleCan>(m, "SingleCan")
      .def(py::init<>())
      .def_readwrite("id", &SingleCan::id)
      .def_property("data",
                    [](const SingleCan& i) { return py::bytes(i.data); },
                    [](SingleCan& i, const std::string& value) { i.data = value; })
      .def_readwrite("bus", &SingleCan::bus)
      .def_readwrite("expect_reply", &SingleCan::expect_reply)
      ;

  py::class_<Input>(m, "Input")
      .def(py::init<>())
      .def_readwrite("tx_can", &Input::tx_can)
      ;

  py::class_<Output>(m, "Output")
      .def(py::init<>())
      .def_readwrite("rx_can", &Output::rx_can)
      ;

  py::class_<Pi3HatRouter>(m, "Pi3HatRouter")
      .def(py::init<Pi3HatRouter::Options>())
      .def("cycle", &Pi3HatRouter::Cycle)
      ;
}
