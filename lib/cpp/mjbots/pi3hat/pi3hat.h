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

#ifndef _MJBOTS_PI3HAT_PI3HAT_H_
#define _MJBOTS_PI3HAT_PI3HAT_H_

/// @file
///
/// This header file (and associated C++ file), are intended to be
/// standalone, relying on nothing but a C++11 compiler and standard
/// library.

#include <cstdint>
#include <cstdlib>
#include <stdexcept>

namespace mjbots {
namespace pi3hat {

// Here are some vocabulary types used in the API.

template <typename T>
class Span {
 public:
  Span(T* ptr, size_t size) : ptr_(ptr), size_(size) {}
  Span() : ptr_(nullptr), size_(0) {}

  T* data() { return ptr_; }
  T* data() const { return ptr_; }
  size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  T& operator[](size_t i) { return ptr_[i]; }
  T& operator[](size_t i) const { return ptr_[i]; }
  T* begin() { return ptr_; }
  T* end() { return ptr_ + size_; }
  const T* begin() const { return ptr_; }
  const T* end() const { return ptr_ + size_; }

 private:
  T* ptr_;
  size_t size_;
};

struct CanFrame {
  uint32_t id = 0;
  uint8_t data[64] = {};
  uint8_t size = 0;

  /// Bus 1, 2, 3, 4 are the high speed buses labeled JC1, JC2, JC3,
  /// JC4.  Bus 5 is the low speed bus labeled JC5.
  int bus = 0;

  /// If true, then a reply will be expected for this frame on the
  /// same bus.
  bool expect_reply = false;

  /// If set, this is used as a hint to increase delays.  If unset,
  /// then the minimum delay may need to be increased.
  uint8_t expected_reply_size = 0;
};

struct Quaternion {
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Quaternion() {}
  Quaternion(double w_in, double x_in, double y_in, double z_in)
      : w(w_in), x(x_in), y(y_in), z(z_in) {}
};

struct Point3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Point3D() {}
  Point3D(double x_in, double y_in, double z_in)
      : x(x_in), y(y_in), z(z_in) {}
};

struct Euler {
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;

  Euler() {}
  Euler(double yaw_in, double pitch_in, double roll_in)
      : yaw(yaw_in), pitch(pitch_in), roll(roll_in) {}
};

struct Attitude {
  Quaternion attitude;
  Point3D rate_dps;
  Point3D accel_mps2;

  Point3D bias_dps;
  Quaternion attitude_uncertainty;
  Point3D bias_uncertainty_dps;
};

struct RfSlot {
  uint8_t slot = 0;
  uint32_t priority = 0;
  uint8_t size = 0;
  uint8_t data[16] = {};
  uint32_t age_ms = 0;
};

class Error : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// This class provides the top level interface to an application
/// which wants to use all the features of the mjbots pi3 hat in an
/// integrated fashion at high rate.
///
/// The primary operations are blocking and busy-loop the CPU.  For
/// best timing performance, the following steps should be taken.
///
///  * The CPU this runs on should be isolated from all other OS
///    operations.  This can be accomplished through the isolcpus
///    mechanism and sched_setaffinity.
///  * The thread this is running in should be set to realtime
///    priority.
class Pi3Hat {
 public:
  /// If any of these fields are non-negative, then they are used
  /// instead of the "bitrate" options in the CAN configuration.
  struct CanRateOverride {
    int prescaler = -1;
    int sync_jump_width = -1;
    int time_seg1 = -1;
    int time_seg2 = -1;
  };

  struct CanConfiguration {
    int slow_bitrate = 1000000;
    int fast_bitrate = 5000000;
    bool fdcan_frame = true;
    bool bitrate_switch = true;
    bool automatic_retransmission = true;
    bool restricted_mode = false;
    bool bus_monitor = false;

    CanRateOverride std_rate;
    CanRateOverride fd_rate;

    uint32_t cancel_all_ms = 50;
  };

  struct Configuration {
    int spi_speed_hz = 10000000;

    // All attitude data will be transformed by this mounting angle.
    Euler mounting_deg;

    // Only a fixed set of rates are achievable.  Valid values are
    // 100, 200, 400, 1000.  Selecting a higher rate than you need to
    // sample at will result in more noise.
    uint32_t attitude_rate_hz = 400;

    // RF communication will be with a transmitter having this ID.
    uint32_t rf_id = 5678;

    bool enable_aux = true;

    CanConfiguration can[5] = {};

    // If true, nothing is guaranteed to work but ReadSpi.
    bool raw_spi_only = false;

    Configuration() {}
  };

  /// This may throw an instance of `Error` if construction fails for
  /// some reason
  Pi3Hat(const Configuration&);
  ~Pi3Hat();

  // This object is non-copyable.
  Pi3Hat(const Pi3Hat&) = delete;
  Pi3Hat& operator=(const Pi3Hat&) = delete;

  struct Input {
    Span<CanFrame> tx_can;
    Span<RfSlot> tx_rf;

    /// When waiting for CAN replies, the call will return all data
    /// that is available (possibly more than was requested).  If no
    /// data is available, and the given timeout has been exceeded,
    /// then it will return anyway.
    ///
    /// The timeout is measured from when the reading phase begins.
    /// This is not super precise, as the writing process has various
    /// queues, so it will need to encompass some amount of the time
    /// spend writing as well.
    uint32_t timeout_ns = 0;

    /// When waiting for CAN replies, guarantee to wait for at least
    /// this many nanoseconds after the final transmission is sent
    /// over SPI (not necessarily over the CAN bus).
    uint32_t min_tx_wait_ns = 200000;

    /// In addition to the absolute min_tx_wait_ns, there is a
    /// parallel calculation that attempts to estimate the total delay
    /// path for command-response pairs to set a minimum delay.  This
    /// value controls the non-calculated part of that estimate.  It
    /// should be the worst case response latency for a single device.
    ///
    /// Note, by default this is much longer than a device should
    /// actually take, so as to handle a wide range of possible host
    /// configurations.  i.e. including non-isolcpus or non-chrt
    /// operation.  If you have a properly configured system running
    /// on an isolcpu, this can potentially be as small as 200us.
    uint32_t rx_baseline_wait_ns = 1000000;

    /// After each successful receipt, wait this much longer for more.
    uint32_t rx_extra_wait_ns = 0;

    bool request_attitude = false;

    // If true, then the bias and uncertainty information will be
    // filled in.  This increases the overall cycle time.
    bool request_attitude_detail = false;

    // If no new attitude is available, wait for it.
    bool wait_for_attitude = false;

    bool request_rf = false;

    // A bitmask indicating CAN buses to check for data even if no
    // replies are expected.
    //  * bit 0 is unused, so the used bits start from 1
    uint32_t force_can_check = 0;

    // These are data to store results in.
    Span<CanFrame> rx_can;
    Span<RfSlot> rx_rf;
    Attitude* attitude = nullptr;
  };

  struct Output {
    int error = 0;
    bool attitude_present = false;
    size_t rx_can_size = 0;
    size_t rx_rf_size = 0;

    // This will only be updated if 'Input::request_rf' is true
    uint32_t rf_lock_age_ms = 0;
  };

  /// Do some or all of the following:
  ///  * Send the given CAN frames
  ///  * Wait for at least the requested number of replies (on a
  ///    per-bus basis), or the given timeout
  ///  * Read the current ARS result
  ///  * Send any desired RF slots
  ///  * Return any RF slots that may have been received
  Output Cycle(const Input& input);

  struct ProcessorInfo {
    uint8_t git_hash[20] = {};
    bool dirty = false;
    uint8_t serial_number[12] = {};
  };

  struct DeviceInfo {
    ProcessorInfo can1;
    ProcessorInfo can2;
    ProcessorInfo aux;

    // Is it safe to send frames that may never be acknowledged?  Some
    // older firmware versions became effectively "frozen" if a CAN
    // frame was sent that was never acknowledged, either because no
    // device whatsoever was connected to the bus or because nothing
    // acknowledged the given ID.
    bool can_unknown_address_safe = false;
  };

  DeviceInfo device_info();

  struct PerformanceInfo {
    uint32_t cycles_per_ms = 0;
    uint32_t min_cycles_per_ms = 0;
  };

  struct DevicePerformance {
    PerformanceInfo can1;
    PerformanceInfo can2;
    PerformanceInfo aux;
  };

  DevicePerformance device_performance();

  /// Read raw SPI data.
  void ReadSpi(int spi_bus, int address, char* data, size_t size);

 private:
  class Impl;
  Impl* const impl_;
};

}
}

#endif
