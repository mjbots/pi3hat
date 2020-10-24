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

/// @file
///
/// This is a simple application that demonstrates how to efficiently
/// monitor and control multiple moteus servos at a high rate using
/// the pi3hat.
///
/// It is contained in a single file for the purposes of
/// demonstration.  A real application should likely be implemented in
/// multiple translation units or structured for longer term
/// maintenance.

#include <sched.h>
#include <sys/mman.h>

#include <condition_variable>
#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mjbots/pi3hat/pi3hat.h"
#include "mjbots/moteus/moteus_protocol.h"

using namespace mjbots;

namespace {
struct Arguments {
  Arguments(const std::vector<std::string>& args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--main-cpu") {
        main_cpu = std::stoull(args.at(++i));
      } else if (arg == "--can-cpu") {
        can_cpu = std::stoull(args.at(++i));
      } else if (arg == "--period-s") {
        period_s = std::stod(args.at(++i));
      } else {
        throw std::runtime_error("Unknown argument: " + arg);
      }
    }
  }

  bool help = false;
  int main_cpu = 1;
  int can_cpu = 2;
  double period_s = 0.002;
};

void DisplayUsage() {
  std::cout << "Usage: moteus_control_example [options]\n";
  std::cout << "\n";
  std::cout << "  -h, --help        display this usage message\n";
  std::cout << "  --main-cpu CPU    run main thread on a fixed CPU [default: 1]\n";
  std::cout << "  --can-cpu CPU     run CAN thread on a fixed CPU [default: 2]\n";
  std::cout << "  --period-s S      period to run control\n";
}

void LockMemory() {
  // We lock all memory so that we don't end up having to page in
  // something later which can take time.
  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

void ConfigureRealtime(int cpu) {
  {
    cpu_set_t cpuset = {};
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);

    const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (r < 0) {
      throw std::runtime_error("Error setting CPU affinity");
    }

    std::cout << "Affinity set to " << cpu << "\n";
  }

  {
    struct sched_param params = {};
    params.sched_priority = 10;
    const int r = ::sched_setscheduler(0, SCHED_RR, &params);
    if (r < 0) {
      throw std::runtime_error("Error setting realtime scheduler");
    }
  }
}


/// This class represents the interface to the moteus controllers.
/// Internally it uses a background thread to operate the pi3hat,
/// enabling the main thread to perform work while servo communication
/// is taking place.
class MoteusInterface {
 public:
  struct Options {
    int cpu = -1;

    // If a servo is not present, it is assumed to be on bus 1.
    std::map<int, int> servo_bus_map;
  };

  MoteusInterface(const Options& options)
      : options_(options),
        thread_(std::bind(&MoteusInterface::CHILD_Run, this)) {
  }

  ~MoteusInterface() {
    std::lock_guard<std::mutex> lock(mutex_);
    done_ = true;
    condition_.notify_one();
    thread_.join();
  }

  // This describes what you would like to do in a given control cycle
  // in terms of sending commands or querying data.
  struct Data {
    pi3hat::Span<int> servo_ids;
    pi3hat::Span<moteus::PositionCommand> positions;
    pi3hat::Span<moteus::PositionResolution> resolutions;
    pi3hat::Span<moteus::QueryCommand> queries;

    pi3hat::Span<int> query_ids;
    pi3hat::Span<moteus::QueryResult> query_results;
  };

  struct Output {
    size_t query_result_size = 0;
  };

  /// When called, this will schedule a cycle of communication with
  /// the servos.  The future will be notified when the communication
  /// cycle has completed.  All memory pointed to by @p data must
  /// remain valid until the future is set.
  std::future<Output> Cycle(const Data& data) {
    // We require all the input structures to have the same size.
    if ((data.servo_ids.size() !=
         data.positions.size()) ||
        (data.servo_ids.size() !=
         data.resolutions.size()) ||
        (data.servo_ids.size() !=
         data.queries.size())) {
      throw std::logic_error("invalid input");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
      throw std::logic_error(
          "Cycle cannot be called until the previous has completed");
    }

    active_ = true;
    data_ = data;
    std::promise<Output> discard;
    promise_.swap(discard);

    condition_.notify_all();

    return promise_.get_future();
  }

 private:
  void CHILD_Run() {
    ConfigureRealtime(options_.cpu);

    pi3hat_.reset(new pi3hat::Pi3Hat({}));

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
      {
        std::unique_lock<std::mutex> lock(mutex_);
        active_ = false;
      }
      promise_.set_value(output);
    }
  }

  Output CHILD_Cycle() {
    tx_can_.resize(data_.servo_ids.size());
    for (size_t i = 0; i < data_.servo_ids.size(); i++) {
      const auto& query = data_.queries[i];
      const auto servo_id = data_.servo_ids[i];

      auto& can = tx_can_[i];
      can.expect_reply = query.any_set();
      can.id = servo_id | (can.expect_reply ? 0x8000 : 0x0000);
      can.size = 0;

      can.bus = [&]() {
        const auto it = options_.servo_bus_map.find(servo_id);
        if (it == options_.servo_bus_map.end()) {
          return 1;
        }
        return it->second;
      }();

      moteus::WriteCanFrame write_frame(can.data, &can.size);
      moteus::EmitPositionCommand(&write_frame, data_.positions[i], data_.resolutions[i]);
      moteus::EmitQueryCommand(&write_frame, data_.queries[i]);
    }

    rx_can_.resize(data_.servo_ids.size() * 2);

    pi3hat::Pi3Hat::Input input;
    input.tx_can = { tx_can_.data(), tx_can_.size() };
    input.rx_can = { rx_can_.data(), rx_can_.size() };

    Output result;

    const auto output = pi3hat_->Cycle(input);
    for (size_t i = 0; i < output.rx_can_size && i < data_.query_ids.size(); i++) {
      const auto& can = rx_can_[i];

      data_.query_ids[i] = can.id & 0x7f00 >> 8;
      data_.query_results[i] = moteus::ParseQueryResult(can.data, can.size);
      result.query_result_size = i + 1;
    }

    return result;
  }

  const Options options_;

  std::promise<Output> promise_;

  /// This block of variables are all controlled by the mutex.
  std::mutex mutex_;
  std::condition_variable condition_;
  bool active_ = false;;
  bool done_ = false;
  Data data_;

  std::thread thread_;


  /// All further variables are only used from within the child thread.

  std::unique_ptr<pi3hat::Pi3Hat> pi3hat_;

  // These are kept persistently so that no memory allocation is
  // required in steady state.
  std::vector<pi3hat::CanFrame> tx_can_;
  std::vector<pi3hat::CanFrame> rx_can_;
};

std::pair<double, double> MinMaxVoltage(const std::vector<moteus::QueryResult>& r) {
  double rmin = std::numeric_limits<double>::infinity();
  double rmax = -std::numeric_limits<double>::infinity();

  for (const auto& i : r) {
    if (i.voltage > rmax) { rmax = i.voltage; }
    if (i.voltage < rmin) { rmin = i.voltage; }
  }

  return std::make_pair(rmin, rmax);
}

void Run(const Arguments& args) {
  if (args.help) {
    DisplayUsage();
    return;
  }

  ConfigureRealtime(args.main_cpu);
  MoteusInterface moteus_interface{[&]() {
      MoteusInterface::Options options;
      options.cpu = args.can_cpu;
      options.servo_bus_map = {
        { 1, 1 },
        { 2, 1 },
        { 3, 1 },
        { 4, 2 },
        { 5, 2 },
        { 6, 2 },
        { 7, 3 },
        { 8, 3 },
        { 9, 3 },
        { 10, 4 },
        { 11, 4 },
        { 12, 4 },
      };
      return options;
    }()};

  std::vector<int> servo_ids = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
  std::vector<moteus::PositionCommand> position_commands{12};
  std::vector<moteus::PositionResolution> position_resolutions;
  std::vector<moteus::QueryCommand> queries{12};

  std::vector<int> query_ids;
  query_ids.resize(12);
  std::vector<moteus::QueryResult> query_results{12};
  std::vector<int> saved_ids;
  std::vector<moteus::QueryResult> saved_results;

  {
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt16;
    res.velocity = moteus::Resolution::kInt16;
    res.feedforward_torque = moteus::Resolution::kInt16;
    res.kp_scale = moteus::Resolution::kIgnore;
    res.kd_scale = moteus::Resolution::kIgnore;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;
    position_resolutions.assign(12, res);
  }

  MoteusInterface::Data moteus_data;
  moteus_data.servo_ids = { servo_ids.data(), servo_ids.size() };
  moteus_data.positions = { position_commands.data(), position_commands.size() };
  moteus_data.resolutions = { position_resolutions.data(), position_resolutions.size() };
  moteus_data.queries = { queries.data(), queries.size() };
  moteus_data.query_ids = { query_ids.data(), query_ids.size() };
  moteus_data.query_results = { query_results.data(), query_results.size() };

  std::future<MoteusInterface::Output> can_result;

  const auto period =
      std::chrono::microseconds(static_cast<int64_t>(args.period_s * 1e6));
  auto next_cycle = std::chrono::steady_clock::now() + period;

  const auto status_period = std::chrono::seconds(1);
  auto next_status = next_cycle + status_period;
  uint64_t cycle_count = 0;
  double total_margin = 0.0;

  // We will run at a fixed cycle time.
  while (true) {
    cycle_count++;
    {
      const auto now = std::chrono::steady_clock::now();
      if (now > next_status) {
        const auto volts = MinMaxVoltage(saved_results);
        std::cout << "Cycles " << cycle_count
                  << "  margin: " << (total_margin / cycle_count)
                  << "  volts: " << volts.first << "/" << volts.second
                  << "   \r";
        std::cout.flush();
        next_status += status_period;
      }

      int skip_count = 0;
      while (now > next_cycle) {
        skip_count++;
        next_cycle += period;
      }
      if (skip_count) {
        std::cout << "\nSkipped " << skip_count << " cycles\n";
      }
    }
    // Wait for the next control cycle to come up.
    {
      const auto pre_sleep = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(next_cycle);
      const auto post_sleep = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
      total_margin += elapsed.count();
    }
    next_cycle += period;


    // TODO: Run control using saved_results.
    for (auto& pos : position_commands) {
      pos.position = std::numeric_limits<double>::quiet_NaN();
      // Leave everything else at the default.
    }


    if (can_result.valid()) {
      // Now we get the result of our last query and send off our new
      // one.
      const auto current_values = can_result.get();

      // We copy out the results we just got out.
      const auto rx_count = current_values.query_result_size;
      saved_results.resize(rx_count);
      saved_ids.resize(rx_count);
      std::copy(query_ids.begin(), query_ids.begin() + rx_count,
                saved_ids.begin());
      std::copy(query_results.begin(), query_results.begin() + rx_count,
                saved_results.begin());
    }

    // Then we can immediately ask them to be used again.
    can_result = moteus_interface.Cycle(moteus_data);
  }
}
}

int main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  // Lock memory for the whole process.
  LockMemory();

  Run(args);

  return 0;
}
