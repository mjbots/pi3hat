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

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

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

  moteus::ConfigureRealtime(args.main_cpu);
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

  std::vector<int> servo_ids = { 1, 2, 4, 5, 7, 8, 10, 11  };
  const auto servo_count = servo_ids.size();
  std::vector<moteus::PositionCommand> position_commands{servo_count};
  std::vector<moteus::PositionResolution> position_resolutions;
  std::vector<moteus::QueryCommand> queries{servo_count};

  std::vector<int> query_ids;
  query_ids.resize(servo_count);
  std::vector<moteus::QueryResult> query_results{servo_count};
  std::vector<int> saved_ids;
  std::vector<moteus::QueryResult> saved_results;

  {
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt16;
    res.velocity = moteus::Resolution::kInt16;
    res.feedforward_torque = moteus::Resolution::kInt16;
    res.kp_scale = moteus::Resolution::kInt16;
    res.kd_scale = moteus::Resolution::kInt16;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;
    position_resolutions.assign(servo_count, res);
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
