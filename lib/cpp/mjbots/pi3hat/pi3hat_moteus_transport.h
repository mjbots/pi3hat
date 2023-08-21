// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "pi3hat.h"

#include "moteus_transport.h"
#include "realtime.h"

namespace mjbots {
namespace pi3hat {

/// This can be used to connect a pi3hat to software that uses the
/// moteus::Transport mechanism for interacting with moteus
/// controllers.  The easiest way to use it is to simply include the
/// code from lib/cpp/examples/pi3hat_moteus_transport_register.cc or
/// link against it.
///
/// It is also possible to manually construct an instance and pass it
/// to the `Controller::Options.transport` method, or manually
/// construct an instance and use its `Cycle` method with the results
/// of `Controller::MakeFoo` methods.
class Pi3HatMoteusTransport : public moteus::Transport {
 public:
  using CanFdFrame = moteus::CanFdFrame;

  struct Options : public pi3hat::Pi3Hat::Configuration {
    int cpu = -1;

    // If the bus for a command is left at 0, and the servo ID is
    // present here, assume it is attached to the given bus.
    std::map<int, int> servo_map;

    // This can be used to configure timeouts.  Any pointers used to
    // accept results will be ignored.
    pi3hat::Pi3Hat::Input default_input;
  };

  Pi3HatMoteusTransport(const Options& options)
      : options_(options),
        thread_(std::bind(&Pi3HatMoteusTransport::CHILD_Run, this)) {
    // Try to clear any stale replies.
    std::vector<CanFdFrame> replies;
    pi3hat::Pi3Hat::Input input_override;
    input_override.force_can_check = (1 << 1) | (1 << 2) | (1 << 3);

    moteus::BlockingCallback cbk;
    Cycle(nullptr, 0, &replies, nullptr, nullptr,
          &input_override, cbk.callback());
    cbk.Wait();
  }

  virtual ~Pi3HatMoteusTransport() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      done_ = true;
      condition_.notify_one();
    }
    thread_.join();
  }

  virtual void Cycle(const CanFdFrame* frames,
                     size_t size,
                     std::vector<CanFdFrame>* replies,
                     moteus::CompletionCallback completed_callback) override {
    Cycle(frames, size, replies, nullptr, nullptr, nullptr, completed_callback);
  }

  virtual void Post(std::function<void()> callback) override {
    std::unique_lock<std::mutex> lock(mutex_);
    event_queue_.push_back(std::move(callback));

    condition_.notify_one();
  }

  void Cycle(const CanFdFrame* frames,
             size_t size,
             std::vector<CanFdFrame>* replies,
             pi3hat::Attitude* attitude,
             pi3hat::Pi3Hat::Output* pi3hat_output,
             pi3hat::Pi3Hat::Input* input_override,
             moteus::CompletionCallback completed_callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
      throw std::logic_error(
          "Cycle cannot be called until the previous has completed");
    }

    cycle_data_ = CycleData{
      frames, size,
      replies,
      attitude,
      pi3hat_output,
      input_override,
      completed_callback,
    };
    active_ = true;

    condition_.notify_all();
  }

 private:
  void CHILD_Run() {
    if (options_.cpu >= 0) {
      ConfigureRealtime(options_.cpu);
    }

    pi3hat_.reset(new pi3hat::Pi3Hat(options_));

    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&]() {
          return (done_ ||
                  active_ ||
                  !event_queue_.empty());
        });

        if (done_) { return; }
      }

      if (active_) {
        CHILD_Cycle();
        moteus::CompletionCallback completed_callback;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          active_ = false;
          std::swap(completed_callback, cycle_data_.completed_callback);
        }
        completed_callback(0);
      }

      // Check for any events to post.
      std::function<void()> maybe_callback;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!event_queue_.empty()) {
          maybe_callback = event_queue_.front();
          event_queue_.pop_front();
        }
      }
      if (maybe_callback) { maybe_callback(); }
    }
  }

  void CHILD_Cycle() {
    // We want to have room to receive frames even if we aren't
    // sending anything.
    rx_can_.resize(std::max<size_t>(5, tx_can_.size() * 2));

    // Now do our actual transaction.
    tx_can_.resize(cycle_data_.size);
    int out_idx = 0;
    for (size_t i = 0; i < cycle_data_.size; i++) {
      const auto& cmd = cycle_data_.frames[i];
      auto& can = tx_can_[out_idx++];

      can.expect_reply = cmd.reply_required;
      can.id = cmd.arbitration_id;
      can.bus = CHILD_FindBus(cmd.destination, cmd.bus);
      can.size = cmd.size;
      std::memcpy(can.data, cmd.data, cmd.size);
      can.expected_reply_size = cmd.expected_reply_size;
    }

    pi3hat::Pi3Hat::Input input =
        cycle_data_.input_override ?
        *cycle_data_.input_override :
        options_.default_input;
    input.tx_can = { tx_can_.data(), tx_can_.size() };
    input.rx_can = { rx_can_.data(), rx_can_.size() };
    input.attitude = cycle_data_.attitude;
    if (input.attitude) { input.request_attitude = true; }

    const auto output = pi3hat_->Cycle(input);

    if (cycle_data_.pi3hat_output) { *cycle_data_.pi3hat_output = output; }
    if (cycle_data_.replies) {
      cycle_data_.replies->clear();
    }
    for (size_t i = 0; i < output.rx_can_size; i++) {
      const auto& can = rx_can_[i];

      if (cycle_data_.replies) {
        cycle_data_.replies->push_back({});
        auto& reply = cycle_data_.replies->back();

        reply.arbitration_id = can.id;
        reply.destination = can.id & 0x7f;
        reply.source = (can.id >> 8) & 0x7f;
        reply.can_prefix = can.id >> 16;
        reply.size = can.size;
        std::memcpy(reply.data, can.data, can.size);
        reply.bus = can.bus;
      }
    }
  }

  int CHILD_FindBus(int id, int supplied_bus) const {
    if (supplied_bus > 1) { return supplied_bus; }

    auto it = options_.servo_map.find(id);
    if (it != options_.servo_map.end()) { return it->second; }

    // Assume 1 if we have nothing else to go on.
    return 1;
  }

  const Options options_;

  ////////////////////////////////////////////////////////////////////
  // This block of variables are all controlled by the mutex.
  std::mutex mutex_;
  std::condition_variable condition_;
  bool active_ = false;
  bool done_ = false;
  struct CycleData {
    const CanFdFrame* frames = nullptr;
    size_t size = 0;
    std::vector<CanFdFrame>* replies = nullptr;
    pi3hat::Attitude* attitude = nullptr;
    pi3hat::Pi3Hat::Output* pi3hat_output = nullptr;
    pi3hat::Pi3Hat::Input* input_override = nullptr;
    moteus::CompletionCallback completed_callback;
  };
  CycleData cycle_data_;
  std::deque<std::function<void()>> event_queue_;

  std::thread thread_;

  ////////////////////////////////////////////////////////////////////
  // All further variables are only used from within the child thread.

  std::unique_ptr<pi3hat::Pi3Hat> pi3hat_;

  // These are kept persistently so that no memory allocation is
  // required in steady state.
  std::vector<pi3hat::CanFrame> tx_can_;
  std::vector<pi3hat::CanFrame> rx_can_;
};

class Pi3HatMoteusFactory : public moteus::TransportFactory {
 public:
  virtual ~Pi3HatMoteusFactory() {}

  virtual int priority() override { return 5; }
  virtual std::string name() override { return "pi3hat"; }

  virtual TransportFactory::TransportArgPair
  make(const std::vector<std::string>& args_in) override{
    auto args = args_in;
    Pi3HatMoteusTransport::Options options;

    auto find_arg_it = [&](const std::string& name) {
      return std::find(args.begin(), args.end(), name);
    };

    auto find_bool_arg = [&](const std::string& name) {
      auto it = find_arg_it(name);
      if (it != args.end()) {
        args.erase(it);
        return true;
      }
      return false;
    };

    auto find_arg_option = [&](const std::string& name) -> std::string {
      auto it = find_arg_it(name);
      if (it != args.end() && (it + 1) != args.end()) {
        auto result = *(it + 1);
        args.erase(it, it + 2);
        return result;
      }
      return {};
    };


    if (find_bool_arg("--can-disable-brs")) {
      for (auto& can : options.can) { can.bitrate_switch = false; }
    }

    const auto pi3hat_cpu = find_arg_option("--pi3hat-cpu");
    if (!pi3hat_cpu.empty()) {
      options.cpu = std::stol(pi3hat_cpu);
    }

    const auto pi3hat_spi_hz = find_arg_option("--pi3hat-spi-hz");
    if (!pi3hat_spi_hz.empty()) {
      options.spi_speed_hz = std::stol(pi3hat_spi_hz);
    }

    const auto pi3hat_cfg = find_arg_option("--pi3hat-cfg");
    if(!pi3hat_cfg.empty()) {
      options.servo_map = ParseServoMap(pi3hat_cfg);
    }

    const auto pi3hat_disable_aux = find_arg_it("--pi3hat-disable-aux");
    if (pi3hat_disable_aux != args.end()) {
      options.enable_aux = false;
    }

    return std::make_pair(std::make_shared<Pi3HatMoteusTransport>(options),
                          args);
  }

  virtual std::vector<Argument> cmdline_arguments() override {
    return {
      { "--can-disable-brs", 0, "do not set BRS" },
      { "--pi3hat-cpu", 1, "CPU used for busy-looping on pi3hat" },
      { "--pi3hat-spi-hz", 1, "SPI speed to use" },
      { "--pi3hat-cfg", 1, "1=ID1,ID2;N=IDX,IDY..." },
      { "--pi3hat-disable-aux", 0, "Prevent use of the IMU/JC5" },
    };
  }

  virtual bool is_args_set(const std::vector<std::string>& args) override {
    for (const auto& arg : args) {
      if (arg == "--pi3hat-cpu" ||
          arg == "--pi3hat-spi-hz" ||
          arg == "--pi3hat-cfg" ||
          arg == "--pi3hat-disable-aux") {
        return true;
      }
    }
    return false;
  }

  static void Register() {
    static bool called = false;
    if (called) { return; }
    called = true;
    moteus::TransportRegistry::singleton().Register<Pi3HatMoteusFactory>();
  }

 private:

  static std::vector<std::string> Split(const std::string& message,
                                        char delim) {
    std::vector<std::string> result;
    size_t pos = 0;
    while (pos < message.size()) {
      const size_t next = message.find(delim, pos);
      if (next == std::string::npos) {
        result.push_back(message.substr(pos));
        return result;
      }
      result.push_back(message.substr(pos, next - pos));
      pos = next + 1;
    }

    return result;
  }

  static std::map<int, int> ParseServoMap(const std::string& str_map) {
    std::map<int, int> result;

    const auto buses = Split(str_map, ';');
    for (const auto& bus : buses) {
      const auto busid_servoids = Split(bus, '=');
      if (busid_servoids.size() != 2) {
        throw std::runtime_error("Invalid bus specifier: " + bus);
      }
      const auto bus_id = std::stol(busid_servoids.at(0));
      const auto servoids = Split(busid_servoids.at(1), ',');
      for (const auto& servoid : servoids) {
        const auto int_servo_id = std::stol(servoid);
        result.insert({int_servo_id, bus_id});
      }
    }

    return result;
  }

};

}
}
