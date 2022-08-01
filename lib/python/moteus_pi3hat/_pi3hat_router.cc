// Copyright 2020-2021 Josh Pieper, jjp@pobox.com.
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
#include <future>
#include <iostream>
#include <string>
#include <thread>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include "mjbots/pi3hat/pi3hat.h"
#include "mjbots/moteus/realtime.h"

namespace py = pybind11;
namespace pi3hat = mjbots::pi3hat;

namespace {
struct SingleCan {
  int arbitration_id = 0;
  bool is_extended_id = true;
  int dlc = 0;
  std::string data;
  bool is_fd = true;
  bool bitrate_switch = true;

  int bus = 0;
  bool expect_reply = false;
};

struct Input {
  std::vector<SingleCan> tx_can;
  uint32_t force_can_check = 0;
  int32_t max_rx = -1;
  bool request_attitude = false;
  uint32_t timeout_ns = 0;
  uint32_t min_tx_wait_ns = 200000;
  uint32_t rx_extra_wait_ns = 40000;
};

using Euler = pi3hat::Euler;

struct Attitude : pi3hat::Attitude {
  Euler euler_rad;
};

struct Output {
  std::vector<SingleCan> rx_can;
  bool attitude_present = false;
  Attitude attitude;
};

Euler ConvertEulerRad(const pi3hat::Quaternion& q) {
  Euler result_rad;

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  if (sinp >= (1.0 - 1e-8)) {
    result_rad.pitch = M_PI_2;
    result_rad.roll = 0.0;
    result_rad.yaw = -2.0 * std::atan2(q.x, q.w);
  } else if (sinp <= (-1.0 + 1e-8)) {
    result_rad.pitch = -M_PI_2;
    result_rad.roll = 0.0;
    result_rad.yaw = 2.0 * std::atan2(q.x, q.w);
  } else {
    result_rad.pitch = std::asin(sinp);

    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    result_rad.roll = std::atan2(sinr_cosp, cosr_cosp);

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    result_rad.yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  return result_rad;
}

class Pi3HatRouter {
 public:
  struct Options : pi3hat::Pi3Hat::Configuration {
    int cpu = 3;
  };

  Pi3HatRouter(const Options& options)
      : options_(options),
        thread_(std::bind(&Pi3HatRouter::CHILD_Run, this)) {
    auto future = init_promise_.get_future();
    auto ep = future.get();
    if (ep) {
      thread_.join();
      std::rethrow_exception(ep);
    }
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

      out.id = in.arbitration_id;
      out.size = in.data.size();
      std::memcpy(&out.data[0], in.data.data(), out.size);
      out.bus = in.bus;
      out.expect_reply = in.expect_reply;
    }
    force_can_check_ = input.force_can_check;
    request_attitude_ = input.request_attitude;
    timeout_ns_ = input.timeout_ns;
    min_tx_wait_ns_ = input.min_tx_wait_ns;
    rx_extra_wait_ns_ = input.rx_extra_wait_ns;
    if (input.max_rx >= 0) {
      rx_can_.resize(input.max_rx);
    } else {
      rx_can_.resize(tx_can_.size() * 2);
    }
  }

  void CHILD_Run() {
    std::exception_ptr ep = nullptr;
    try {
      mjbots::moteus::ConfigureRealtime(options_.cpu);
      pi3hat_.reset(new pi3hat::Pi3Hat(options_));
    } catch (...) {
      ep = std::current_exception();
    }
    init_promise_.set_value(ep);
    if (ep) { return; }

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

      // This will be a python object, so we need the GIL.
      py::gil_scoped_acquire acquire;

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
    input.force_can_check = force_can_check_;
    input.attitude = &attitude_;

    input.request_attitude = request_attitude_;
    // If we're requesting attitude, always wait for it.
    input.wait_for_attitude = request_attitude_;

    input.timeout_ns = timeout_ns_;
    input.min_tx_wait_ns = min_tx_wait_ns_;
    input.rx_extra_wait_ns = rx_extra_wait_ns_;

    Output result;

    const auto output = pi3hat_->Cycle(input);

    for (size_t i = 0; i < output.rx_can_size; i++) {
      SingleCan out;
      const auto& in = rx_can_[i];
      out.arbitration_id = in.id;
      out.is_extended_id = in.id > 0x7ff;
      out.dlc = in.size;
      out.data = std::string(reinterpret_cast<const char*>(&in.data[0]), in.size);
      out.bus = in.bus;
      result.rx_can.push_back(std::move(out));
    }
    result.attitude_present = output.attitude_present;
    if (result.attitude_present) {
      result.attitude = attitude_;
      result.attitude.euler_rad = ConvertEulerRad(result.attitude.attitude);
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
  std::promise<std::exception_ptr> init_promise_;

  // Used in the child thread.
  std::unique_ptr<pi3hat::Pi3Hat> pi3hat_;


  // These are populated in the parent thread when active_ == false,
  // and from the child thread when active_ == true.
  //
  // We leave these around just so that we don't have to repeatedly
  // allocate and also to make lifetime management easier.
  std::vector<pi3hat::CanFrame> tx_can_;
  uint32_t force_can_check_ = 0;
  bool request_attitude_ = false;
  uint32_t timeout_ns_ = 0;
  uint32_t min_tx_wait_ns_ = 0;
  uint32_t rx_extra_wait_ns_ = 0;
  std::vector<pi3hat::CanFrame> rx_can_;
  Attitude attitude_;
};
}

PYBIND11_MODULE(_pi3hat_router, m) {
  m.doc() = "implementation of pi3hat specific moteus functionality";
  using PH = pi3hat::Pi3Hat;

  py::class_<PH::CanRateOverride>(m, "CanRateOverride")
      .def(py::init<>())
      .def_readwrite("prescaler", &PH::CanRateOverride::prescaler)
      .def_readwrite("sync_jump_width",
                     &PH::CanRateOverride::sync_jump_width)
      .def_readwrite("time_seg1", &PH::CanRateOverride::time_seg1)
      .def_readwrite("time_seg2", &PH::CanRateOverride::time_seg2)
      ;

  py::class_<PH::CanConfiguration>(m, "CanConfiguration")
      .def(py::init<>())
      .def_readwrite("slow_bitrate", &PH::CanConfiguration::slow_bitrate)
      .def_readwrite("fast_bitrate", &PH::CanConfiguration::fast_bitrate)
      .def_readwrite("fdcan_frame", &PH::CanConfiguration::fdcan_frame)
      .def_readwrite("bitrate_switch", &PH::CanConfiguration::bitrate_switch)
      .def_readwrite("automatic_retransmission",
                     &PH::CanConfiguration::automatic_retransmission)
      .def_readwrite("restricted_mode",
                     &PH::CanConfiguration::restricted_mode)
      .def_readwrite("bus_monitor",
                     &PH::CanConfiguration::bus_monitor)
      .def_readwrite("std_rate", &PH::CanConfiguration::std_rate)
      .def_readwrite("fd_rate", &PH::CanConfiguration::fd_rate)
      ;

  py::class_<Pi3HatRouter::Options>(m, "Options")
      .def(py::init<>())
      .def_readwrite("cpu", &Pi3HatRouter::Options::cpu)
      .def_readwrite("spi_speed_hz", &Pi3HatRouter::Options::spi_speed_hz)
      .def_readwrite("mounting_deg", &Pi3HatRouter::Options::mounting_deg)
      .def_readwrite("attitude_rate_hz", &Pi3HatRouter::Options::attitude_rate_hz)
      .def_readwrite("enable_aux", &Pi3HatRouter::Options::enable_aux)
      // We rely on the fact that std::array has the same in-memory
      // layout as a C style array.
      .def_readwrite("can", reinterpret_cast<
                     std::array<PH::CanConfiguration, 5>
                     Pi3HatRouter::Options::*>(&Pi3HatRouter::Options::can))
      ;

  py::class_<SingleCan>(m, "SingleCan")
      .def(py::init<>())
      .def_readwrite("arbitration_id", &SingleCan::arbitration_id)
      .def_readwrite("is_extended_id", &SingleCan::is_extended_id)
      .def_readwrite("dlc", &SingleCan::dlc)
      .def_property("data",
                    [](const SingleCan& i) { return py::bytes(i.data); },
                    [](SingleCan& i, const std::string& value) { i.data = value; })
      .def_readwrite("is_fd", &SingleCan::is_fd)
      .def_readwrite("bitrate_switch", &SingleCan::bitrate_switch)

      .def_readwrite("bus", &SingleCan::bus)
      .def_readwrite("expect_reply", &SingleCan::expect_reply)
      ;

  py::class_<Input>(m, "Input")
      .def(py::init<>())
      .def_readwrite("tx_can", &Input::tx_can)
      .def_readwrite("force_can_check", &Input::force_can_check)
      .def_readwrite("max_rx", &Input::max_rx)
      .def_readwrite("request_attitude", &Input::request_attitude)
      .def_readwrite("timeout_ns", &Input::timeout_ns)
      .def_readwrite("min_tx_wait_ns", &Input::min_tx_wait_ns)
      .def_readwrite("rx_extra_wait_ns", &Input::rx_extra_wait_ns)
      ;

  py::class_<Output>(m, "Output")
      .def(py::init<>())
      .def_readwrite("rx_can", &Output::rx_can)
      .def_readwrite("attitude_present", &Output::attitude_present)
      .def_readwrite("attitude", &Output::attitude)
      ;

  py::class_<Euler>(m, "Euler")
      .def(py::init<>())
      .def_readwrite("roll", &Euler::roll)
      .def_readwrite("pitch", &Euler::pitch)
      .def_readwrite("yaw", &Euler::yaw)
      ;

  py::class_<Attitude>(m, "Attitude")
      .def(py::init<>())
      .def_readwrite("attitude", &Attitude::attitude)
      .def_readwrite("rate_dps", &Attitude::rate_dps)
      .def_readwrite("accel_mps2", &Attitude::accel_mps2)
      .def_readwrite("euler_rad", &Attitude::euler_rad)
      ;

  py::class_<pi3hat::Quaternion>(m, "Quaternion")
      .def(py::init<>())
      .def_readwrite("w", &pi3hat::Quaternion::w)
      .def_readwrite("x", &pi3hat::Quaternion::x)
      .def_readwrite("y", &pi3hat::Quaternion::y)
      .def_readwrite("z", &pi3hat::Quaternion::z)
      ;

  py::class_<pi3hat::Point3D>(m, "Point3D")
      .def(py::init<>())
      .def_readwrite("x", &pi3hat::Point3D::x)
      .def_readwrite("y", &pi3hat::Point3D::y)
      .def_readwrite("z", &pi3hat::Point3D::z)
      ;

  py::class_<Pi3HatRouter>(m, "Pi3HatRouter")
      .def(py::init<Pi3HatRouter::Options>())
      .def("cycle", &Pi3HatRouter::Cycle)
      ;
}
