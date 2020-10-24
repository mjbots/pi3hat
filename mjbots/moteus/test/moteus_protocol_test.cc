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

#include "mjbots/moteus/moteus_protocol.h"

#include <boost/test/auto_unit_test.hpp>

using namespace mjbots::moteus;

BOOST_AUTO_TEST_CASE(SaturateTest) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  BOOST_TEST(Saturate<int8_t>(-1000.0, 1.0) == -127);
  BOOST_TEST(Saturate<int8_t>(1000.0, 1.0) == 127);
  BOOST_TEST(Saturate<int8_t>(kNaN, 1.0) == -128);
  BOOST_TEST(Saturate<int8_t>(10.0, 1.0) == 10);
  BOOST_TEST(Saturate<int8_t>(-15.0, 1.0) == -15);
  BOOST_TEST(Saturate<int8_t>(0.0, 1.0) == 0);
  BOOST_TEST(Saturate<int8_t>(10, 0.1) == 100);

  BOOST_TEST(Saturate<int16_t>(-1000000, 1.0) == -32767);
  BOOST_TEST(Saturate<int16_t>(kNaN, 1.0) == -32768);
  BOOST_TEST(Saturate<int16_t>(123, 1.0) == 123);
}

BOOST_AUTO_TEST_CASE(EmitPositionCommandTest) {
  CanFrame f;
  WriteCanFrame can_frame{&f};

  PositionCommand pos;
  PositionResolution res;
  res.position = Resolution::kInt8;
  res.velocity = Resolution::kIgnore;
  res.feedforward_torque = Resolution::kIgnore;
  res.kp_scale = Resolution::kIgnore;
  res.kd_scale = Resolution::kIgnore;
  res.maximum_torque = Resolution::kIgnore;
  res.stop_position = Resolution::kIgnore;
  res.watchdog_timeout = Resolution::kIgnore;

  EmitPositionCommand(&can_frame, pos, res);

  BOOST_TEST(f.size == 6);
  BOOST_TEST(f.data[0] == 0x01);
  BOOST_TEST(f.data[1] == 0x00);
  BOOST_TEST(f.data[2] == 0x0a);
  BOOST_TEST(f.data[3] == 0x01);
  BOOST_TEST(f.data[4] == 0x20);
  BOOST_TEST(f.data[5] == 0x00);

  // Now try with more than one register of a different type after.
  res.velocity = Resolution::kInt16;
  res.feedforward_torque = Resolution::kInt16;
  res.kp_scale = Resolution::kIgnore;
  res.kd_scale = Resolution::kFloat;
  res.maximum_torque = Resolution::kFloat;
  res.stop_position = Resolution::kFloat;
  res.watchdog_timeout = Resolution::kFloat;

  pos.position = 0.4;
  pos.velocity = 0.2;
  pos.feedforward_torque = -1.0;
  pos.kd_scale = 0.3;
  pos.maximum_torque = 4.0;
  pos.stop_position = 1.2;
  pos.watchdog_timeout = 0.5;

  f = {};
  EmitPositionCommand(&can_frame, pos, res);
  BOOST_TEST(f.size == 31);

  BOOST_TEST(f.data[0] == 0x01);
  BOOST_TEST(f.data[1] == 0x00);
  BOOST_TEST(f.data[2] == 0x0a);
  BOOST_TEST(f.data[3] == 0x01);
  BOOST_TEST(f.data[4] == 0x20);
  BOOST_TEST(f.data[5] == 0x28);

  BOOST_TEST(f.data[6] == 0x06);  // write 2 int16
  BOOST_TEST(f.data[7] == 0x21);  // starting at 21
  BOOST_TEST(f.data[8] == 0x20);  // velocity 0.2 / 0.00025 = 0x320
  BOOST_TEST(f.data[9] == 0x03);
  BOOST_TEST(f.data[10] == 0x9c);  // torque -1.0 / 0.01 = 0xff9c
  BOOST_TEST(f.data[11] == 0xff);

  BOOST_TEST(f.data[12] == 0x0c);  // write float
  BOOST_TEST(f.data[13] == 0x04);  // 4x floats
  BOOST_TEST(f.data[14] == 0x24);  // starting at 24 (kd scale)
}

BOOST_AUTO_TEST_CASE(EmitQueryCommandTest) {
  CanFrame f;
  WriteCanFrame can_frame{&f};

  QueryCommand cmd;
  EmitQueryCommand(&can_frame,cmd);

  BOOST_TEST(f.size == 5);
  BOOST_TEST(f.data[0] == 0x14);
  BOOST_TEST(f.data[1] == 0x04);
  BOOST_TEST(f.data[2] == 0x00);
  BOOST_TEST(f.data[3] == 0x13);
  BOOST_TEST(f.data[4] == 0x0d);
}

BOOST_AUTO_TEST_CASE(ParseQueryResultTest) {
  CanFrame can_frame;
  can_frame.size = 5;
  can_frame.data[0] = 0x23;
  can_frame.data[1] = 0x00;
  can_frame.data[2] = 0x01;
  can_frame.data[3] = 0x02;
  can_frame.data[4] = 0x03;

  {
    const auto result = ParseQueryResult(can_frame.data, can_frame.size);
    BOOST_TEST((result.mode == Mode::kFault));
    BOOST_TEST(result.position == 0.02);
    BOOST_TEST(result.velocity == 0.30000000000000004);
  }

  can_frame.size = 13;

  // Verify we can skip a nop in the middle.
  can_frame.data[5] = 0x50;

  can_frame.data[6] = 0x24;  // n int16s
  can_frame.data[7] = 0x02;  // 2 of them
  can_frame.data[8] = 0x0d;  // starting at voltage
  can_frame.data[9] = 0x40;  // 6.4V
  can_frame.data[10] = 0x00;
  can_frame.data[11] = 0x50; // 8.0C
  can_frame.data[12] = 0x00;

  {
    const auto result = ParseQueryResult(can_frame.data, can_frame.size);
    BOOST_TEST((result.mode == Mode::kFault));
    BOOST_TEST(result.position == 0.02);
    BOOST_TEST(result.velocity == 0.30000000000000004);
    BOOST_TEST(result.voltage == 6.4);
    BOOST_TEST(result.temperature == 8.0);
  }
}
