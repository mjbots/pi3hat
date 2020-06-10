// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include <time.h>
#include <unistd.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "pi3hat.h"

namespace mjbots {
namespace pi3hat {

namespace {
int64_t GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000ll +
      static_cast<int64_t>(ts.tv_nsec);
}

const auto kNaN = std::numeric_limits<double>::quiet_NaN();

struct Arguments {
  Arguments(const std::vector<std::string>& args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--spi-speed") {
        spi_speed_hz = std::stoi(args.at(++i));
      } else if (arg == "--mount-y") {
        mounting_deg.yaw = std::stod(args.at(++i));
      } else if (arg == "--mount-p") {
        mounting_deg.pitch = std::stod(args.at(++i));
      } else if (arg == "--mount-r") {
        mounting_deg.roll = std::stod(args.at(++i));
      } else if (arg == "--rf-id") {
        rf_id = std::stoul(args.at(++i));
      } else if (arg == "-c" || arg == "--write-can") {
        write_can.push_back(args.at(++i));
      } else if (arg == "--read-can") {
        read_can |= (1 << (std::stoi(args.at(++i))));
      } else if (arg == "--can-timeout-ns") {
        can_timeout_ns = std::stoull(args.at(++i));
      } else if (arg == "-r" || arg == "--run") {
        run = true;
      } else {
        throw std::runtime_error("Unknown argument: " + arg);
      }
    }
  }

  bool help = false;
  bool run = false;

  int spi_speed_hz = -1;
  Euler mounting_deg = {kNaN, kNaN, kNaN};
  uint32_t rf_id = 0;

  std::vector<std::string> write_can;
  uint32_t read_can = 0;
  int64_t can_timeout_ns = 0;
};

void DisplayUsage() {
  std::cout << "Usage: pi3hat_tool [options]\n";
  std::cout << "\n";
  std::cout << "  -h,--help       display this usage\n";
  std::cout << "\n";
  std::cout << "Configuration\n";
  std::cout << "  --spi-speed HZ      set the SPI speed\n";
  std::cout << "  --mount-y DEG       set the mounting yaw angle\n";
  std::cout << "  --mount-p DEG       set the mounting pitch angle\n";
  std::cout << "  --mount-r DEG       set the mounting roll angle\n";
  std::cout << "  --rf-id ID          set the RF id\n";
  std::cout << "  --can-timeout-ns T  set the receive timeout\n";
  std::cout << "\n";
  std::cout << "Actions\n";
  std::cout << "  -c,--write-can  write a CAN frame (can be listed more)\n";
  std::cout << "     BUS,HEXID,HEXDATA,FLAGS\n";
  std::cout << "        FLAGS - r (expect reply)\n";
  std::cout << "  --read-can BUS  force data to be read from the given bus (1-5)\n";
  std::cout << "  -r,--run        run a sample high rate cycle\n";
}

Pi3Hat::Configuration MakeConfig(const Arguments& args) {
  Pi3Hat::Configuration config;
  if (args.spi_speed_hz >= 0) {
    config.spi_speed_hz = args.spi_speed_hz;
  }
  if (std::isfinite(args.mounting_deg.yaw) ||
      std::isfinite(args.mounting_deg.roll) ||
      std::isfinite(args.mounting_deg.pitch)) {
    config.mounting_deg.yaw =
        std::isfinite(args.mounting_deg.yaw) ? args.mounting_deg.yaw : 0.0;
    config.mounting_deg.pitch =
        std::isfinite(args.mounting_deg.pitch) ? args.mounting_deg.pitch : 0.0;
    config.mounting_deg.roll =
        std::isfinite(args.mounting_deg.roll) ? args.mounting_deg.roll : 0.0;
  }
  if (args.rf_id != 0) {
    config.rf_id = args.rf_id;
  }
  return config;
}

void CheckError(int error) {
  if (! error) { return; }
  throw std::runtime_error(
      "Unexpected pi3hat error: " + std::to_string(error));
}

int ParseHexNybble(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  return -1;
}

int ParseHexByte(const char* value) {
  int high = ParseHexNybble(value[0]);
  if (high < 0) { return high; }
  int low = ParseHexNybble(value[1]);
  if (low < 0) { return low; }
  return (high << 4) | low;
}

size_t ParseHexData(uint8_t* output, const char* data, size_t size) {
  if ((size % 2) != 0) {
    throw std::runtime_error("CAN data invalid length: " +
                             std::string(data, size));
  }
  size_t bytes = 0;
  for (size_t i = 0; i < size; i += 2) {
    const int value = ParseHexByte(&data[i]);
    if (value < 0) {
      throw std::runtime_error("Invalid CAN data: " + std::string(data, size));
    }
    output[bytes++] = value;
  }

  return bytes;
}

CanFrame ParseCanString(const std::string& can_string) {
  size_t pos = 0;
  size_t next = 0;

  auto find_next = [&]() {
    next = can_string.find(',', pos);
    if (next == std::string::npos) {
      next = can_string.size();
    }
  };

  find_next();

  auto advance = [&]() {
    pos = next + 1;
    find_next();
  };

  CanFrame result;

  result.bus = std::strtoul(&can_string[pos], nullptr, 0);
  advance();
  result.id  = std::strtoul(&can_string[pos], nullptr, 16);
  advance();
  result.size = ParseHexData(result.data, &can_string[pos], next - pos);

  if (next < can_string.size()) {
    advance();
    // Any remaining things are flags
    for (; pos < can_string.size(); pos++) {
      switch (std::tolower(can_string[pos])) {
        case 'r': {
          result.expect_reply = true;
          break;
        }
      }
    }
  }

  return result;
}

std::string FormatCanFrame(const CanFrame& can_frame) {
  char buf[10] = {};
  std::string result;
  result += std::to_string(can_frame.bus) + ",";
  ::snprintf(buf, sizeof(buf) - 1, "%X", can_frame.id);
  result += std::string(buf) + ",";
  for (size_t i = 0; i < can_frame.size; i++) {
    ::snprintf(buf, sizeof(buf) - 1, "%02X", static_cast<int>(can_frame.data[i]));
    result += std::string(buf);
  }
  return result;
}

void DoCan(Pi3Hat* pi3hat, const Arguments& args) {
  Pi3Hat::Input input;
  input.request_attitude = false;
  input.wait_for_attitude = false;

  input.force_can_check = args.read_can;
  std::vector<CanFrame> can_frames;
  can_frames.resize(args.write_can.size());
  for (const auto& can_string : args.write_can) {
    can_frames.push_back(ParseCanString(can_string));
  }
  input.tx_can = { &can_frames[0], can_frames.size() };

  std::vector<CanFrame> rx_frames;
  // Try to make sure we have plenty of room to receive things.
  rx_frames.resize(std::max<size_t>(can_frames.size() * 2, 20u));

  input.rx_can = { &rx_frames[0], rx_frames.size() };

  const auto result = pi3hat->Cycle(input);
  CheckError(result.error);

  for (std::size_t i = 0; i < result.rx_can_size; i++) {
    std::cout << FormatCanFrame(rx_frames.at(i)) << "\n";
  }
}

void Run(Pi3Hat* pi3hat) {
  Pi3Hat::Input input;
  Attitude attitude;
  input.wait_for_attitude = true;
  input.attitude = &attitude;
  char buf[2048] = {};
  double filtered_period_s = 0.0;
  int64_t old_now = GetNow();


  while (true) {
    // For now, we'll just ask for attitude in a blocking manner.
    const auto result = pi3hat->Cycle(input);
    CheckError(result.error);
    if (!result.attitude_present) {
      throw std::runtime_error("Missing attitude");
    }

    {
      const auto now = GetNow();
      const double delta_s = (now - old_now) * 1e-9;
      const double alpha = 0.98;
      filtered_period_s = alpha * filtered_period_s + (1.0 - alpha) * delta_s;
      old_now = now;
    }

    {
      const auto& a = attitude;
      ::snprintf(
          buf, sizeof(buf) - 1,
          "w=%5.3f x=%5.3f y=%5.3f z=%5.3f  dps=(%5.1f,%5.1f,%5.1f) "
          "a=(%4.1f,%4.1f,%4.1f)  %5.1f Hz  \r",
          a.attitude.w, a.attitude.x, a.attitude.y, a.attitude.z,
          a.rate_dps.x, a.rate_dps.y, a.rate_dps.z,
          a.accel_mps2.x, a.accel_mps2.y, a.accel_mps2.z,
          1.0 / filtered_period_s);
      std::cout << buf;
      std::cout.flush();
    }

    ::usleep(500);
  }
}

int do_main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  if (args.help) {
    DisplayUsage();
    return 0;
  }

  Pi3Hat pi3hat{MakeConfig(args)};

  if (args.read_can != 0 || !args.write_can.empty()) {
    DoCan(&pi3hat, args);
  } else if (args.run) {
    Run(&pi3hat);
  } else {
    std::cout << "Nothing to do!\n";
  }

  return 0;
}

}  // namespace
}  // namespace pi3hat
}  // namespace mjbots

int main(int argc, char** argv) {
  return mjbots::pi3hat::do_main(argc, argv);
}
