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

#include <string_view>

#include "mbed.h"

#include "mjlib/base/string_span.h"

namespace fw {

class FDCan {
 public:
  enum class FilterAction {
    kDisable,
    kAccept,
    kReject,
  };

  enum class FilterMode {
    kRange,
    kDual,
    kMask,
  };

  enum class FilterType {
    kStandard,
    kExtended,
  };

  struct Filter {
    uint32_t id1 = 0;
    uint32_t id2 = 0;

    FilterMode mode = FilterMode::kRange;
    FilterAction action = FilterAction::kDisable;
    FilterType type = FilterType::kStandard;
  };

  struct Rate {
    int prescaler = -1;
    int sync_jump_width = -1;
    int time_seg1 = -1;
    int time_seg2 = -1;
  };

  struct Options {
    PinName td = NC;
    PinName rd = NC;
    int slow_bitrate = 1000000;
    int fast_bitrate = 5000000;

    bool automatic_retransmission = false;
    bool remote_frame = false;
    bool fdcan_frame = false;
    bool bitrate_switch = false;
    bool restricted_mode = false;
    bool bus_monitor = false;

    // We for now default to use compensation that is appropriate for
    // the TCAN1057 with its longer delay:
    //
    //  start - 13 / 85Mhz ~= 152ns
    //  min   - 1 / 85Mhz ~= 12ns
    bool delay_compensation = true;
    uint32_t tdc_offset = 13;
    uint32_t tdc_filter = 1;

    FilterAction global_std_action = FilterAction::kAccept;
    FilterAction global_ext_action = FilterAction::kAccept;
    FilterAction global_remote_std_action = FilterAction::kAccept;
    FilterAction global_remote_ext_action = FilterAction::kAccept;

    const Filter* filter_begin = nullptr;
    const Filter* filter_end = nullptr;

    // If any members of this are non-negative, force them to be used
    // instead of the auto-calculated values.
    Rate rate_override;
    Rate fdrate_override;

    Options() {}
  };

  FDCan(const Options& options = Options());

  Options options() { return options_; }

  void Reset(const Options&);

  enum class Override {
    kDefault,
    kRequire,
    kDisable,
  };

  struct SendOptions {
    Override bitrate_switch = Override::kDefault;
    Override fdcan_frame = Override::kDefault;
    Override remote_frame = Override::kDefault;
    Override extended_id = Override::kDefault;
    bool abort_existing = false;

    SendOptions() {}
  };

  enum SendResult {
    kSuccess,
    kNoSpace,
  };

  SendResult Send(uint32_t dest_id,
                  std::string_view data,
                  const SendOptions& = SendOptions());

  /// Cancel all queued packets.
  void CancelAll();

  /// @return true if a packet was available.
  bool Poll(FDCAN_RxHeaderTypeDef* header, mjlib::base::string_span);

  void RecoverBusOff();

  FDCAN_ProtocolStatusTypeDef status();
  FDCAN_ErrorCountersTypeDef error_counters();

  struct Config {
    int clock = 0;
    Rate nominal;
    Rate data;
  };

  Config config() const;

  static int ParseDlc(uint32_t dlc_code);

 private:
  Options options_;
  Config config_;

  FDCAN_GlobalTypeDef* can_ = nullptr;
  FDCAN_HandleTypeDef hfdcan1_;
  uint32_t last_tx_request_ = 0;
};

}
