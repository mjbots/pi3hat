// Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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
/// Show how to read IMU data and moteus controller data
/// simultaneously using the Pi3HatMoteusTransport.

#include <unistd.h>

#include <iostream>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

int main(int argc, char** argv) {
  using namespace mjbots;

  using Transport = pi3hat::Pi3HatMoteusTransport;

  Transport::Options toptions;
  toptions.servo_map[1] = 1;
  toptions.attitude_rate_hz = 100;

  toptions.mounting_deg.pitch = 0;
  toptions.mounting_deg.yaw = 0;
  toptions.mounting_deg.roll = 0;

  auto transport = std::make_shared<Transport>(toptions);

  pi3hat::Attitude attitude;

  moteus::Controller controller([&]() {
    moteus::Controller::Options options;
    options.transport = transport;
    options.id = 1;
    return options;
  }());

  moteus::PositionMode::Command pos_cmd;
  pos_cmd.position = std::numeric_limits<float>::quiet_NaN();

  std::vector<moteus::CanFdFrame> frames;
  frames.push_back(controller.MakePosition(pos_cmd));

  std::vector<moteus::CanFdFrame> replies;
  moteus::BlockingCallback cbk;

  transport->Cycle(frames.data(), frames.size(),
                   &replies, &attitude,
                   nullptr, nullptr,
                   cbk.callback());
  cbk.Wait();

  // Do something with attitude.
  const auto& a = attitude.attitude;
  ::printf("Attitude quaternion: (%4.2f,%4.2f,%4.2f,%4.2f)\n",
           a.x, a.y, a.z, a.w);

  for (const auto& frame : replies) {
    if (frame.source == 1) {  // Check if it's from servo ID 1
      const auto result = moteus::Query::Parse(frame.data, frame.size);
      ::printf("Servo %d  p/v/t=(%7.3f,%7.3f,%7.3f)\n",
               frame.source, result.position, result.velocity, result.torque);
    }
  }
  return 0;
}
