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
      } else if (arg == "--write-rf") {
        write_rf.push_back(args.at(++i));
      } else if (arg == "--read-rf") {
        read_rf = true;
      } else if (arg == "--read-att") {
        read_attitude = true;
      } else if (arg == "--info") {
        info = true;
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
  uint32_t rf_id = 5678;

  std::vector<std::string> write_can;
  uint32_t read_can = 0;
  int64_t can_timeout_ns = 0;

  std::vector<std::string> write_rf;
  bool read_rf = false;
  bool read_attitude = false;

  bool info = false;
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
  std::cout << "  -c,--write-can  write a CAN frame (can be listed 0+)\n";
  std::cout << "     BUS,HEXID,HEXDATA,FLAGS\n";
  std::cout << "        FLAGS - r (expect reply)\n";
  std::cout << "  --read-can BUS  force data to be read from the given bus (1-5)\n";
  std::cout << "  --write-rf      write an RF slot (can be listed 0+)\n";
  std::cout << "     SLOT,PRIORITY,DATA\n";
  std::cout << "  --read-rf       request any RF data\n";
  std::cout << "  --read-att      request attitude data\n";
  std::cout << "  --info          display device info\n";
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

std::string FormatHexBytes(const uint8_t* data, size_t size) {
  std::string result;
  char buf[10] = {};
  for (size_t i = 0; i < size; i++) {
    ::snprintf(buf, sizeof(buf) - 1, "%02x", static_cast<int>(data[i]));
    result += std::string(buf);
  }
  return result;
}

struct MiniTokenizer {
 public:
  MiniTokenizer(const std::string& data) : data_(data) {
    find_next();
  }

  void find_next() {
    next = data_.find(',', pos);
    if (next == std::string::npos) {
      next = data_.size();
    }
  }

  void advance() {
    pos = next + 1;
    find_next();
  }

  size_t pos = 0;
  size_t next = 0;
  std::string data_;
};

CanFrame ParseCanString(const std::string& can_string) {
  MiniTokenizer tok{can_string};

  CanFrame result;

  result.bus = std::strtoul(&can_string[tok.pos], nullptr, 0);
  tok.advance();
  result.id  = std::strtoul(&can_string[tok.pos], nullptr, 16);
  tok.advance();
  result.size = ParseHexData(result.data, &can_string[tok.pos],
                             tok.next - tok.pos);

  if (tok.next < can_string.size()) {
    tok.advance();
    // Any remaining things are flags
    for (; tok.pos < can_string.size(); tok.pos++) {
      switch (std::tolower(can_string[tok.pos])) {
        case 'r': {
          result.expect_reply = true;
          break;
        }
      }
    }
  }

  return result;
}

RfSlot ParseRfString(const std::string& rf_string) {
  MiniTokenizer tok{rf_string};

  RfSlot result;
  result.slot = std::strtoul(&rf_string[tok.pos], nullptr, 0);
  tok.advance();
  result.priority = std::strtoul(&rf_string[tok.pos], nullptr, 16);
  tok.advance();
  result.size = ParseHexData(result.data, &rf_string[tok.pos],
                             tok.next - tok.pos);

  return result;
}

std::string FormatCanFrame(const CanFrame& can_frame) {
  char buf[10] = {};
  std::string result;
  result += std::to_string(can_frame.bus) + ",";
  ::snprintf(buf, sizeof(buf) - 1, "%X", can_frame.id);
  result += std::string(buf) + ",";
  result += FormatHexBytes(&can_frame.data[0], can_frame.size);
  return result;
}

std::string FormatAttitude(const Attitude& a) {
  char buf[2048] = {};
  ::snprintf(
      buf, sizeof(buf) - 1,
      "w=%5.3f x=%5.3f y=%5.3f z=%5.3f  dps=(%5.1f,%5.1f,%5.1f) "
      "a=(%4.1f,%4.1f,%4.1f)",
      a.attitude.w, a.attitude.x, a.attitude.y, a.attitude.z,
      a.rate_dps.x, a.rate_dps.y, a.rate_dps.z,
      a.accel_mps2.x, a.accel_mps2.y, a.accel_mps2.z);
  return std::string(buf);
}

std::string FormatRf(const RfSlot& r) {
  std::string result = std::to_string(r.slot) + " ";
  result += FormatHexBytes(&r.data[0], r.size);
  return result;
}

void Run(Pi3Hat* pi3hat) {
  Pi3Hat::Input input;
  Attitude attitude;
  input.request_attitude = true;
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
      ::snprintf(
          buf, sizeof(buf) - 1,
          "%s  %5.1f Hz  \r",
          FormatAttitude(attitude).c_str(),
          1.0 / filtered_period_s);
      std::cout << buf;
      std::cout.flush();
    }

    ::usleep(500);
  }
}

std::string FormatProcessorInfo(const Pi3Hat::ProcessorInfo& pi) {
  std::string result = FormatHexBytes(&pi.git_hash[0], 20) + " ";
  result += (pi.dirty ? "dirty" : "clean");
  result += " " + FormatHexBytes(&pi.serial_number[0], 12);
  return result;
}

void DoInfo(Pi3Hat* pi3hat) {
  const auto di = pi3hat->device_info();
  std::cout << "CAN1: " << FormatProcessorInfo(di.can1) << "\n";
  std::cout << "CAN2: " << FormatProcessorInfo(di.can2) << "\n";
  std::cout << "AUX:  " << FormatProcessorInfo(di.aux) << "\n";
}

void SingleCycle(Pi3Hat* pi3hat, const Arguments& args) {
  Pi3Hat::Input input;
  Attitude attitude;
  input.attitude = &attitude;
  if (args.read_attitude) {
    input.request_attitude = true;
    input.wait_for_attitude = true;
  }
  if (args.read_rf) {
    input.request_rf = true;
  }
  input.force_can_check = args.read_can;
  std::vector<CanFrame> can_frames;
  for (const auto& can_string : args.write_can) {
    can_frames.push_back(ParseCanString(can_string));
  }
  if (!can_frames.empty()) {
    input.tx_can = { &can_frames[0], can_frames.size() };
  }

  input.timeout_ns = args.can_timeout_ns;

  // Try to make sure we have plenty of room to receive things.
  std::vector<CanFrame> rx_frames;
  rx_frames.resize(std::max<size_t>(can_frames.size() * 2, 20u));
  input.rx_can = { &rx_frames[0], rx_frames.size() };

  std::vector<RfSlot> rf_slots;
  for (const auto& rf_string : args.write_rf) {
    rf_slots.push_back(ParseRfString(rf_string));
  }
  if (!rf_slots.empty()) {
    input.tx_rf = { &rf_slots[0], rf_slots.size() };
  }

  std::vector<RfSlot> rx_rf_data;
  rx_rf_data.resize(16);
  input.rx_rf = { &rx_rf_data[0], rx_rf_data.size() };

  const auto result = pi3hat->Cycle(input);
  CheckError(result.error);

  for (size_t i = 0; i < result.rx_can_size; i++) {
    std::cout << "CAN " << FormatCanFrame(rx_frames.at(i)) << "\n";
  }

  if (args.read_rf) {
    std::cout << "RFLOCK: " << result.rf_lock_age_ms << "\n";
  }

  for (size_t i = 0; i < result.rx_rf_size; i++) {
    std::cout << "RF  " << FormatRf(rx_rf_data.at(i)) << "\n";
  }

  if (result.attitude_present) {
    std::cout << "ATT " << FormatAttitude(attitude) << "\n";
  }
}

int do_main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  if (args.help) {
    DisplayUsage();
    return 0;
  }

  Pi3Hat pi3hat{MakeConfig(args)};

  if (args.run) {
    Run(&pi3hat);
  } else if (args.info) {
    DoInfo(&pi3hat);
  } else {
    SingleCycle(&pi3hat, args);
  }

  return 0;
}

}  // namespace
}  // namespace pi3hat
}  // namespace mjbots

int main(int argc, char** argv) {
  return mjbots::pi3hat::do_main(argc, argv);
}
