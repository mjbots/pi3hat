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

#include <sched.h>

#include <cerrno>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace mjbots {
namespace pi3hat {

// Return the highest CPU the kernel isolates
// (/sys/devices/system/cpu/isolated, i.e. the `isolcpus=` list), or -1
// if the system isolates no CPUs.  Used as the default --pi3hat-cpu so
// the busy SPI worker lands on an isolated core, keeping the kernel's
// general-purpose CPUs free.
inline int DetectIsolatedCpu() {
  std::ifstream inf("/sys/devices/system/cpu/isolated");
  std::string line;
  std::getline(inf, line);

  int best = -1;
  std::stringstream ss(line);
  std::string part;
  while (std::getline(ss, part, ',')) {
    if (part.empty()) { continue; }
    const auto dash = part.find('-');
    try {
      // For a range "a-b" the highest CPU is b; for a single "n" it is n.
      const int hi = (dash == std::string::npos) ?
          std::stoi(part) : std::stoi(part.substr(dash + 1));
      if (hi > best) { best = hi; }
    } catch (...) {
      // Ignore a malformed entry.
    }
  }
  return best;
}

inline void ConfigureRealtime(int cpu) {
  {
    cpu_set_t cpuset = {};
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);

    const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (r < 0) {
      throw std::runtime_error("Error setting CPU affinity");
    }
  }

  {
    struct sched_param params = {};
    params.sched_priority = 10;
    const int r = ::sched_setscheduler(0, SCHED_RR, &params);
    // musl does not implement sched_setscheduler (it returns ENOSYS);
    // the CPU affinity set above is what keeps the worker off the
    // kernel's general CPUs, so tolerate that one case.
    if (r < 0 && errno != ENOSYS) {
      throw std::runtime_error(
          "Error setting realtime scheduler, try running as root (use sudo)");
    }
  }
}

}
}
