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

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>

/// @file
///
/// This describes helper classes useful for constructing and parsing
/// CAN-FD packets for the moteus brushless servo.

namespace mjbots {
namespace moteus {

enum {
  kCurrentRegisterMapVersion = 4,
};

enum Multiplex : uint32_t {
  kWriteBase = 0x00,
  kWriteInt8 = 0x00,
  kWriteInt16 = 0x04,
  kWriteInt32 = 0x08,
  kWriteFloat = 0x0c,

  kReadBase = 0x10,
  kReadInt8 = 0x10,
  kReadInt16 = 0x14,
  kReadInt32 = 0x18,
  kReadFloat = 0x1c,

  kReplyBase = 0x20,
  kReplyInt8 = 0x20,
  kReplyInt16 = 0x24,
  kReplyInt32 = 0x28,
  kReplyFloat = 0x2c,

  kWriteError = 0x30,
  kReadError = 0x31,

  // # Tunneled Stream #
  kClientToServer = 0x40,
  kServerToClient = 0x41,
  kClientPollServer = 0x42,

  kNop = 0x50,
};

enum Register : uint32_t {
  kMode = 0x000,
  kPosition = 0x001,
  kVelocity = 0x002,
  kTorque = 0x003,
  kQCurrent = 0x004,
  kDCurrent = 0x005,
  kRezeroState = 0x00c,
  kVoltage = 0x00d,
  kTemperature = 0x00e,
  kFault = 0x00f,

  kPwmPhaseA = 0x010,
  kPwmPhaseB = 0x011,
  kPwmPhaseC = 0x012,

  kVoltagePhaseA = 0x014,
  kVoltagePhaseB = 0x015,
  kVoltagePhaseC = 0x016,

  kVFocTheta = 0x018,
  kVFocVoltage = 0x019,
  kVoltageDqD = 0x01a,
  kVoltageDqQ = 0x01b,

  kCommandQCurrent = 0x01c,
  kCommandDCurrent = 0x01d,

  kCommandPosition = 0x020,
  kCommandVelocity = 0x021,
  kCommandFeedforwardTorque = 0x022,
  kCommandKpScale = 0x023,
  kCommandKdScale = 0x024,
  kCommandPositionMaxTorque = 0x025,
  kCommandStopPosition = 0x026,
  kCommandTimeout = 0x027,

  kPositionKp = 0x030,
  kPositionKi = 0x031,
  kPositionKd = 0x032,
  kPositionFeedforward = 0x033,
  kPositionCommand = 0x034,

  kRegisterMapVersion = 0x102,
  kSerialNumber = 0x120,
  kSerialNumber1 = 0x120,
  kSerialNumber2 = 0x121,
  kSerialNumber3 = 0x122,

  kRezero = 0x130,
};

enum class Mode {
  kStopped = 0,
  kFault = 1,
  kEnabling = 2,
  kCalibrating = 3,
  kCalibrationComplete = 4,
  kPwm = 5,
  kVoltage = 6,
  kVoltageFoc = 7,
  kVoltageDq = 8,
  kCurrent = 9,
  kPosition = 10,
  kPositionTimeout = 11,
  kZeroVelocity = 12,
  kNumModes,
};

enum class Resolution {
  kInt8,
  kInt16,
  kInt32,
  kFloat,
  kIgnore,
};

template <typename T>
T Saturate(double value, double scale) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<T>::min();
  }

  const double scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();

  const double double_max = static_cast<T>(max);
  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved for NaN.
  if (scaled < -double_max) { return -max; }
  if (scaled > double_max) { return max; }
  return static_cast<T>(scaled);
}

/// A single CAN-FD frame, with methods to conveniently write encoded
/// values.
struct WriteCanFrame {
  uint8_t data[64] = {};
  size_t size = 0;

  template <typename T, typename X>
  void Write(X value_in) {
    T value = static_cast<T>(value_in);
    if (sizeof(value) + size > sizeof(data)) {
      throw std::runtime_error("overflow");
    }

#ifndef __ORDER_LITTLE_ENDIAN__
#error "only little endian architectures supported"
#endif
    std::memcpy(&data[size], reinterpret_cast<const char*>(&value), sizeof(value));
    size += sizeof(value);
  }

  void WriteMapped(
      double value,
      double int8_scale, double int16_scale, double int32_scale,
      Resolution res) {
    switch (res) {
      case Resolution::kInt8: {
        Write<int8_t>(Saturate<int8_t>(value, int8_scale));
        break;
      }
      case Resolution::kInt16: {
        Write<int16_t>(Saturate<int16_t>(value, int16_scale));
        break;
      }
      case Resolution::kInt32: {
        Write<int32_t>(Saturate<int32_t>(value, int32_scale));
        break;
      }
      case Resolution::kFloat: {
        Write<float>(static_cast<float>(value));
        break;
      }
      case Resolution::kIgnore: {
        throw std::runtime_error("Attempt to write ignored resolution");
      }
    }
  }

  void WritePosition(double value, Resolution res) {
    WriteMapped(value, 0.01, 0.0001, 0.00001, res);
  }

  void WriteVelocity(double value, Resolution res) {
    WriteMapped(value, 0.1, 0.00025, 0.00001, res);
  }

  void WriteTorque(double value, Resolution res) {
    WriteMapped(value, 0.5, 0.01, 0.001, res);
  }

  void WritePwm(double value, Resolution res) {
    WriteMapped(value,
                1.0 / 127.0,
                1.0 / 32767.0,
                1.0 / 2147483647.0,
                res);
  }

  void WriteVoltage(double value, Resolution res) {
    WriteMapped(value, 0.5, 0.1, 0.001, res);
  }

  void WriteTemperature(float value, Resolution res) {
    WriteMapped(value, 1.0, 0.1, 0.001, res);
  }

  void WriteTime(float value, Resolution res) {
    WriteMapped(value, 0.01, 0.001, 0.000001, res);
  }
};

/// Determines how to group registers when encoding them to minimize
/// the required bytes.
template <size_t N>
class WriteCombiner {
 public:
  template <typename T>
  WriteCombiner(WriteCanFrame* frame,
                T start_register,
                std::array<Resolution, N> resolutions)
      : frame_(frame),
        start_register_(start_register),
        resolutions_(resolutions) {}

  ~WriteCombiner() {
    if (offset_ != N) {
      ::abort();
    }
  }

  bool MaybeWrite() {
    const auto this_offset = offset_;
    offset_++;

    if (current_resolution_ == resolutions_[this_offset]) {
      // We don't need to write any register operations here, and the
      // value should go out only if requested.
      return current_resolution_ != Resolution::kIgnore;
    }
    // We need to do some kind of framing.  See how far ahead the new
    // resolution goes.
    const auto new_resolution = resolutions_[this_offset];
    current_resolution_ = new_resolution;

    // We are now in a new block of ignores.
    if (new_resolution == Resolution::kIgnore) {
      return false;
    }

    int count = 1;
    for (size_t i = this_offset + 1;
         i < N && resolutions_[i] == new_resolution;
         i++) {
      count++;
    }

    int8_t write_command = [&]() {
      switch (new_resolution) {
        case Resolution::kInt8: return Multiplex::kWriteInt8;
        case Resolution::kInt16: return Multiplex::kWriteInt16;
        case Resolution::kInt32: return Multiplex::kWriteInt32;
        case Resolution::kFloat: return Multiplex::kWriteFloat;
        case Resolution::kIgnore: {
          throw std::logic_error("unreachable");
        }
      }
    }();

    if (count <= 3) {
      // Use the shorthand formulation.
      frame_->Write<int8_t>(write_command + count);
    } else {
      // Nope, the long form.
      frame_->Write<int8_t>(write_command);
      frame_->Write<int8_t>(count);
    }
    frame_->Write<int8_t>(start_register_ + this_offset);
    return true;
  }

 private:
  WriteCanFrame* const frame_;
  uint32_t start_register_;
  std::array<Resolution, N> resolutions_;

  Resolution current_resolution_ = Resolution::kIgnore;
  size_t offset_ = 0;
};

struct PositionCommand {
  double position = 0.0;
  double velocity = 0.0;
  double feedforward_torque = 0.0;
  double kp_scale = 1.0;
  double kd_scale = 1.0;
  double maximum_torque = 0.0;
  double stop_position = std::numeric_limits<double>::quiet_NaN();;
  double watchdog_timeout = 0.0;
};

struct PositionResolution {
  Resolution position = Resolution::kFloat;
  Resolution velocity = Resolution::kFloat;
  Resolution feedforward_torque = Resolution::kFloat;
  Resolution kp_scale = Resolution::kFloat;
  Resolution kd_scale = Resolution::kFloat;
  Resolution maximum_torque = Resolution::kIgnore;
  Resolution stop_position = Resolution::kFloat;
  Resolution watchdog_timeout = Resolution::kFloat;
};

inline void EmitPositionCommand(
    WriteCanFrame* frame,
    const PositionCommand& command, const PositionResolution& resolution) {
  // First, set the position mode.
  frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
  frame->Write<int8_t>(Register::kMode);
  frame->Write<int8_t>(Mode::kPosition);

  // Now we use some heuristics to try and group consecutive registers
  // of the same resolution together into larger writes.
  WriteCombiner<8> combiner(frame, Register::kCommandPosition, {
      resolution.position,
          resolution.velocity,
          resolution.feedforward_torque,
          resolution.kp_scale,
          resolution.kd_scale,
          resolution.maximum_torque,
          resolution.stop_position,
          resolution.watchdog_timeout,
    });

  if (combiner.MaybeWrite()) {
    frame->WritePosition(command.position, resolution.position);
  }
  if (combiner.MaybeWrite()) {
    frame->WriteVelocity(command.velocity, resolution.velocity);
  }
  if (combiner.MaybeWrite()) {
    frame->WriteTorque(command.feedforward_torque, resolution.feedforward_torque);
  }
  if (combiner.MaybeWrite()) {
    frame->WritePwm(command.kp_scale, resolution.kp_scale);
  }
  if (combiner.MaybeWrite()) {
    frame->WritePwm(command.kd_scale, resolution.kd_scale);
  }
  if (combiner.MaybeWrite()) {
    frame->WriteTorque(command.maximum_torque, resolution.maximum_torque);
  }
  if (combiner.MaybeWrite()) {
    frame->WritePosition(command.stop_position, resolution.stop_position);
  }
  if (combiner.MaybeWrite()) {
    frame->WriteTime(command.watchdog_timeout, resolution.watchdog_timeout);
  }
}

}
}
