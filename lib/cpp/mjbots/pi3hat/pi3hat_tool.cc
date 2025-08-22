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

/// @file
///
/// NOTE: This file can be compiled manually on a raspberry pi using
/// the following command line:
///
/// g++ -mcpu=cortex-a53 -L /opt/vc/lib -I /opt/vc/include -std=c++11 pi3hat_tool.cc pi3hat.cc -o pi3hat_tool


#include <sched.h>
#include <sys/mman.h>

#include <time.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
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

std::vector<std::string> Split(const std::string& input, const char* delim = ",") {
  std::vector<std::string> result;
  size_t pos = 0;
  while(pos < input.size()) {
    const size_t next = input.find_first_of(delim, pos);
    result.push_back(input.substr(pos, next - pos));
    if (next == std::string::npos) { break; }
    pos = next + 1;
  }
  return result;
}

/// Euler angles are in roll, pitch, then yaw.
///  +roll -> clockwise about X axis
///  +pitch -> clockwise about Y axis
///  +yaw -> clockwise about Z axis
struct Euler {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

class Quaternion;
inline Quaternion operator*(const Quaternion& lhs,
                            const Quaternion& rhs);

class Quaternion {
 public:
  Quaternion(double w, double x, double y, double z)
      : w_(w), x_(x), y_(y), z_(z) {}

  Quaternion()
      : w_(1.0), x_(0.), y_(0.), z_(0.) {}

  std::string str() const;

  Quaternion conjugated() const {
    return Quaternion(w_, -x_, -y_, -z_);
  }

  double norm() const {
    return std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
  }

  Quaternion normalized() const {
    const double n = norm();
    return Quaternion(w_ / n, x_ / n, y_ / n, z_ / n);
  }

  Euler euler_rad() const {
    Euler result_rad;

    const double sinp = 2.0 * (w_ * y_ - z_ * x_);
    if (sinp >= (1.0 - 1e-8)) {
      result_rad.pitch = M_PI_2;
      result_rad.roll = 0.0;
      result_rad.yaw = -2.0 * std::atan2(x_, w_);
    } else if (sinp <= (-1.0 + 1e-8)) {
      result_rad.pitch = -M_PI_2;
      result_rad.roll = 0.0;
      result_rad.yaw = 2.0 * std::atan2(x_, w_);
    } else {
      result_rad.pitch = std::asin(sinp);

      const double sinr_cosp = 2.0 * (w_ * x_ + y_ * z_);
      const double cosr_cosp = 1.0 - 2.0 * (x_ * x_ + y_ * y_);
      result_rad.roll = std::atan2(sinr_cosp, cosr_cosp);

      const double siny_cosp = 2.0 * (w_ * z_ + x_ * y_);
      const double cosy_cosp = 1.0 - 2.0 * (y_ * y_ + z_ * z_);
      result_rad.yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    return result_rad;
  }

  static Quaternion FromEuler(
      double roll_rad, double pitch_rad, double yaw_rad) {
    // Quaternions multiply in opposite order, and we want to get into
    // roll, pitch, then yaw as standard.
    return (Quaternion::FromAxisAngle(yaw_rad, 0, 0, 1) *
            Quaternion::FromAxisAngle(pitch_rad, 0, 1, 0) *
            Quaternion::FromAxisAngle(roll_rad, 1, 0, 0));
  }

  static Quaternion FromEuler(Euler euler_rad) {
    return FromEuler(euler_rad.roll, euler_rad.pitch, euler_rad.yaw);
  }

  static Quaternion FromAxisAngle(
      double angle_rad, double x, double y, double z) {
    double c = std::cos(angle_rad / 2.0);
    double s = std::sin(angle_rad / 2.0);

    return Quaternion(c, x * s, y * s, z * s);
  }

  double w() const { return w_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

 private:
  double w_;
  double x_;
  double y_;
  double z_;
};

inline Quaternion operator*(const Quaternion& lhs,
                            const Quaternion& rhs) {
  double a = lhs.w();
  double b = lhs.x();
  double c = lhs.y();
  double d = lhs.z();

  double e = rhs.w();
  double f = rhs.x();
  double g = rhs.y();
  double h = rhs.z();

  return Quaternion(a * e - b * f - c * g - d * h,
                    b * e + a * f + c * h - d * g,
                    a * g - b * h + c * e + d * f,
                    a * h + b * g - c * f + d * e);
}

struct Arguments {
  Arguments(const std::vector<std::string>& args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--can-help") {
        can_help = true;
      } else if (arg == "--spi-speed") {
        spi_speed_hz = std::stoi(args.at(++i));
      } else if (arg == "--disable-aux") {
        disable_aux = true;
      } else if (arg == "--mount-y") {
        mounting_deg.yaw = std::stod(args.at(++i));
      } else if (arg == "--mount-p") {
        mounting_deg.pitch = std::stod(args.at(++i));
      } else if (arg == "--mount-r") {
        mounting_deg.roll = std::stod(args.at(++i));
      } else if (arg == "--attitude-rate") {
        attitude_rate_hz = std::stol(args.at(++i));
      } else if (arg == "--rf-id") {
        rf_id = std::stoul(args.at(++i));
      } else if (arg == "--can-config") {
        can_config.push_back(args.at(++i));
      } else if (arg == "-c" || arg == "--write-can") {
        write_can.push_back(args.at(++i));
      } else if (arg == "--read-can") {
        read_can |= (1 << (std::stoi(args.at(++i))));
      } else if (arg == "--can-timeout-ns") {
        can_timeout_ns = std::stoull(args.at(++i));
      } else if (arg == "--can-min-tx-wait-ns") {
        can_min_tx_wait_ns = std::stoull(args.at(++i));
      } else if (arg == "--can-rx-extra-wait-ns") {
        can_rx_extra_wait_ns = std::stoull(args.at(++i));
      } else if (arg == "--time-log") {
        time_log = args.at(++i);
      } else if (arg == "--realtime") {
        realtime = std::stoull(args.at(++i));
      } else if (arg == "--write-rf") {
        write_rf.push_back(args.at(++i));
      } else if (arg == "--read-rf") {
        read_rf = true;
      } else if (arg == "--read-att") {
        read_attitude = true;
      } else if (arg == "--read-spi") {
        read_spi = args.at(++i);
      } else if (arg == "--info") {
        info = true;
      } else if (arg == "--performance") {
        performance = true;
      } else if (arg == "-r" || arg == "--run") {
        run = true;
      } else {
        throw std::runtime_error("Unknown argument: " + arg);
      }
    }
  }

  bool help = false;
  bool can_help = false;
  bool run = false;
  std::string time_log;

  int spi_speed_hz = -1;
  bool disable_aux = false;
  Euler mounting_deg;
  uint32_t attitude_rate_hz = 400;
  uint32_t rf_id = 5678;

  std::vector<std::string> can_config;
  std::vector<std::string> write_can;
  uint32_t read_can = 0;
  int64_t can_timeout_ns = 0;
  int64_t can_min_tx_wait_ns = 200000;
  int64_t can_rx_extra_wait_ns = 40000;
  int realtime = -1;

  std::vector<std::string> write_rf;
  bool read_rf = false;
  bool read_attitude = false;
  std::string read_spi;

  bool info = false;
  bool performance = false;
};

void DisplayUsage() {
  std::cout << "Usage: pi3hat_tool [options]\n";
  std::cout << "\n";
  std::cout << "  -h,--help       display this usage\n";
  std::cout << "  --can-help      display CAN configuration format\n";
  std::cout << "\n";
  std::cout << "Configuration\n";
  std::cout << "  --spi-speed HZ      set the SPI speed\n";
  std::cout << "  --disable-aux       disable the auxiliary processor\n";
  std::cout << "  --mount-y DEG       set the mounting yaw angle\n";
  std::cout << "  --mount-p DEG       set the mounting pitch angle\n";
  std::cout << "  --mount-r DEG       set the mounting roll angle\n";
  std::cout << "  --attitude-rate HZ  set the attitude rate\n";
  std::cout << "  --rf-id ID          set the RF id\n";
  std::cout << "  --can-config CFG    configure a specific CAN bus\n";
  std::cout << "  --can-timeout-ns T  set the receive timeout\n";
  std::cout << "  --can-min-wait-ns T set the receive timeout\n";
  std::cout << "  --realtime CPU      run in a realtime configuration on a CPU\n";
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
  std::cout << "  --read-spi      read raw SPI data\n";
  std::cout << "     SPIBUS,ADDRESS,SIZE\n";
  std::cout << "  --info          display device info\n";
  std::cout << "  --performance   print runtime performance\n";
  std::cout << "  -r,--run        run a sample high rate cycle\n";
  std::cout << "  --time-log F    when running, write a delta-t log\n";
}

void DisplayCanConfigurationUsage() {
  std::cout << " Usage: pi3hat_tool [options]\n";
  std::cout << "  --can-config CFG\n";
  std::cout << "    CFG : CANBUS,[option,]...\n";
  std::cout << "     rN: slow_bitrate\n";
  std::cout << "     RN: fast_bitrate\n";
  std::cout << "     d/D: fdcan_frame=false/true\n";
  std::cout << "     b/B: bitrate_switch=false/true\n";
  std::cout << "     t/T: automatic_retransmission=false/true\n";
  std::cout << "     m/M: bus_monitor=false/true\n";
  std::cout << "     [sf]pN: (std|fd)_rate.prescaler\n";
  std::cout << "     [sf]jN: (std|fd)_rate.sync_jump_width\n";
  std::cout << "     [sf]1N: (std|fd)_rate.time_seg1\n";
  std::cout << "     [sf]2N: (std|fd)_rate.time_seg2\n";
}

template <typename Iterator>
Pi3Hat::CanConfiguration ParseCanConfig(Iterator begin, Iterator end) {
  Pi3Hat::CanConfiguration result;
  for (auto it = begin; it != end; ++it) {
    const auto item = *it;
    if (item.at(0) == 'r') {
      result.slow_bitrate = std::stoi(item.substr(1));
    } else if (item.at(0) == 'R') {
      result.fast_bitrate = std::stoi(item.substr(1));
    } else if (item == "d") {
      result.fdcan_frame = false;
    } else if (item == "D") {
      result.fdcan_frame = true;
    } else if (item == "b") {
      result.bitrate_switch = false;
    } else if (item == "B") {
      result.bitrate_switch = true;
    } else if (item == "t") {
      result.automatic_retransmission = false;
    } else if (item == "T") {
      result.automatic_retransmission = true;
    } else if (item == "m") {
      result.bus_monitor = false;
    } else if (item == "M") {
      result.bus_monitor = true;
    } else if (item.at(0) == 's' ||
               item.at(0) == 'f') {
      auto& rate = (item.at(0) == 's') ? result.std_rate : result.fd_rate;
      const auto value = std::stoi(item.substr(2));
      if (item.at(1) == 'p') {
        rate.prescaler = value;
      } else if (item.at(1) == 'j') {
        rate.sync_jump_width = value;
      } else if (item.at(1) == '1') {
        rate.time_seg1 = value;
      } else if (item.at(1) == '2') {
        rate.time_seg2 = value;
      } else {
        throw std::runtime_error("Unknown rate code:" + item);
      }
    } else {
      throw std::runtime_error("Unknown CAN option: " + item);
    }
  }

  return result;
}

Pi3Hat::Configuration MakeConfig(const Arguments& args) {
  Pi3Hat::Configuration config;
  if (args.spi_speed_hz >= 0) {
    config.spi_speed_hz = args.spi_speed_hz;
  }
  config.mounting_deg.yaw = args.mounting_deg.yaw;
  config.mounting_deg.pitch = args.mounting_deg.pitch;
  config.mounting_deg.roll = args.mounting_deg.roll;
  config.attitude_rate_hz = args.attitude_rate_hz;
  config.enable_aux = !args.disable_aux;

  if (args.rf_id != 0) {
    config.rf_id = args.rf_id;
  }

  for (const auto& can_config : args.can_config) {
    const auto can_fields = Split(can_config);
    if (can_fields.size() < 2) {
      throw std::runtime_error("Error parsing can config: " + can_config);
    }
    const int can_bus = std::stoi(can_fields.at(0));
    if (can_bus < 1 || can_bus > 5) {
      throw std::runtime_error("Invalid CAN bus: " + can_config);
    }

    config.can[can_bus - 1] =
        ParseCanConfig(can_fields.begin() + 1, can_fields.end());
  }

  if (!args.read_spi.empty()) {
    config.raw_spi_only = true;
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

double R2D(double value) {
  return 180.0 * value / 3.141592653589793;
}

std::string FormatAttitude(const Attitude& a) {
  const auto euler_rad =
      Quaternion(a.attitude.w,
                 a.attitude.x,
                 a.attitude.y,
                 a.attitude.z).euler_rad();

  char buf[2048] = {};
  ::snprintf(
      buf, sizeof(buf) - 1,
      "rpy=(%6.1f,%6.1f,%6.1f) dps=(%5.1f,%5.1f,%5.1f) "
      "a=(%4.1f,%4.1f,%4.1f)",
      R2D(euler_rad.roll), R2D(euler_rad.pitch), R2D(euler_rad.yaw),
      a.rate_dps.x, a.rate_dps.y, a.rate_dps.z,
      a.accel_mps2.x, a.accel_mps2.y, a.accel_mps2.z);
  return std::string(buf);
}

std::string FormatRf(const RfSlot& r) {
  std::string result = std::to_string(r.slot) + " ";
  result += FormatHexBytes(&r.data[0], r.size);
  return result;
}

struct Input {
  Input(const Arguments& args) {
    auto& pi = pi3hat_input;
    pi.attitude = &attitude;
    if (args.read_attitude) {
      pi.request_attitude = true;
      pi.wait_for_attitude = true;
    }
    if (args.read_rf) {
      pi.request_rf = true;
    }
    pi.force_can_check = args.read_can;
    for (const auto& can_string : args.write_can) {
      can_frames.push_back(ParseCanString(can_string));
    }
    if (!can_frames.empty()) {
      pi.tx_can = { &can_frames[0], can_frames.size() };
    }

    pi.timeout_ns = args.can_timeout_ns;
    pi.min_tx_wait_ns = args.can_min_tx_wait_ns;
    pi.rx_extra_wait_ns = args.can_rx_extra_wait_ns;

    // Try to make sure we have plenty of room to receive things.
    rx_frames.resize(std::max<size_t>(can_frames.size() * 2, 20u));
    pi.rx_can = { &rx_frames[0], rx_frames.size() };

    for (const auto& rf_string : args.write_rf) {
      rf_slots.push_back(ParseRfString(rf_string));
    }
    if (!rf_slots.empty()) {
      pi.tx_rf = { &rf_slots[0], rf_slots.size() };
    }

    rx_rf_data.resize(16);
    pi.rx_rf = { &rx_rf_data[0], rx_rf_data.size() };

  }

  // We alias our pointers into the input structure.
  Input(const Input&) = delete;
  Input& operator=(const Input&) = delete;

  Pi3Hat::Input pi3hat_input;

  Attitude attitude;
  std::vector<CanFrame> can_frames;
  std::vector<CanFrame> rx_frames;
  std::vector<RfSlot> rf_slots;
  std::vector<RfSlot> rx_rf_data;
};

void Run(Pi3Hat* pi3hat, const Arguments& args) {
  Input input{args};

  std::unique_ptr<std::ofstream> time_of;
  if (!args.time_log.empty()) {
    time_of.reset(new std::ofstream(args.time_log));
  }

  char buf[2048] = {};
  double filtered_period_s = 0.0;
  int64_t old_now = GetNow();

  while (true) {
    const auto result = pi3hat->Cycle(input.pi3hat_input);
    CheckError(result.error);

    {
      const auto now = GetNow();
      const auto delta_ns = now - old_now;
      const double delta_s = delta_ns * 1e-9;
      const double alpha = 0.98;
      filtered_period_s = alpha * filtered_period_s + (1.0 - alpha) * delta_s;
      old_now = now;

      if (time_of) {
        *time_of << delta_ns << "\n";
      }
    }

    if (input.pi3hat_input.wait_for_attitude && !result.attitude_present) {
      throw std::runtime_error("Missing attitude data");
    }

    if (result.attitude_present) {
      std::cout << FormatAttitude(input.attitude) << " ";
    }

    {
      ::snprintf(
          buf, sizeof(buf) - 1,
          "%5.1f Hz  \r",
          1.0 / filtered_period_s);
      std::cout << buf;
      std::cout.flush();
    }

    ::usleep(50);
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

void ReadSpi(Pi3Hat* pi3hat, const std::string& command) {
  const std::vector<std::string> args = Split(command);
  const auto spi_bus = std::stoi(args.at(0));
  const auto address = std::stoi(args.at(1));
  const auto size = std::stoi(args.at(2));

  std::vector<char> data;
  data.resize(size);
  pi3hat->ReadSpi(spi_bus, address, &data[0], data.size());
  std::cout << FormatHexBytes(
      reinterpret_cast<uint8_t*>(&data[0]), data.size()) << "\n";
}

std::string FormatPerformance(const Pi3Hat::PerformanceInfo& p) {
  return
      "average:" + std::to_string(p.cycles_per_ms) + " " +
      "min:" + std::to_string(p.min_cycles_per_ms);
}

void DoPerformance(Pi3Hat* pi3hat) {
  const auto dp = pi3hat->device_performance();
  std::cout << "CAN1: " << FormatPerformance(dp.can1) << "\n";
  std::cout << "CAN2: " << FormatPerformance(dp.can2) << "\n";
  std::cout << "AUX:  " << FormatPerformance(dp.aux) << "\n";
}

void SingleCycle(Pi3Hat* pi3hat, const Arguments& args) {
  Input input{args};

  const auto result = pi3hat->Cycle(input.pi3hat_input);
  CheckError(result.error);

  for (size_t i = 0; i < result.rx_can_size; i++) {
    std::cout << "CAN " << FormatCanFrame(input.rx_frames.at(i)) << "\n";
  }

  if (args.read_rf) {
    std::cout << "RFLOCK: " << result.rf_lock_age_ms << "\n";
  }

  for (size_t i = 0; i < result.rx_rf_size; i++) {
    std::cout << "RF  " << FormatRf(input.rx_rf_data.at(i)) << "\n";
  }

  if (result.attitude_present) {
    std::cout << "ATT " << FormatAttitude(input.attitude) << "\n";
  }
}

void ConfigureRealtime(const Arguments& args) {
  {
    cpu_set_t cpuset = {};
    CPU_ZERO(&cpuset);
    CPU_SET(args.realtime, &cpuset);

    const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (r < 0) {
      throw std::runtime_error("Error setting CPU affinity");
    }

    std::cout << "Affinity set to " << args.realtime << "\n";
  }

  {
    struct sched_param params = {};
    params.sched_priority = 10;
    const int r = ::sched_setscheduler(0, SCHED_RR, &params);
    if (r < 0) {
      throw std::runtime_error("Error setting realtime scheduler");
    }
  }

  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

int do_main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  if (args.help) {
    DisplayUsage();
    return 0;
  }
  if (args.can_help) {
    DisplayCanConfigurationUsage();
    return 0;
  }

  Pi3Hat pi3hat{MakeConfig(args)};

  if (args.realtime >= 0) {
    ConfigureRealtime(args);
  }

  if (args.run) {
    Run(&pi3hat, args);
  } else if (args.info) {
    DoInfo(&pi3hat);
  } else if (!args.read_spi.empty()) {
    ReadSpi(&pi3hat, args.read_spi);
  } else if (args.performance) {
    DoPerformance(&pi3hat);
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
