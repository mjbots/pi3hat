#!/usr/bin/python3

# Copyright 2021 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Demonstrates using the RAW CAN API of the pi3hat.'''

import asyncio
import moteus
import moteus_pi3hat
import time


async def main():
    # The parameters of each CAN bus can be set at construction time.
    # The available fields can be found in the C++ header at
    # Pi3Hat::CanConfiguration
    slow_can = moteus_pi3hat.CanConfiguration()
    slow_can.slow_bitrate = 125000
    slow_can.fdcan_frame = False
    slow_can.bitrate_switch = False

    # If buses are not listed, then they default to the parameters
    # necessary to communicate with a moteus controller.
    can_config = {
        5: slow_can,
    }
    transport = moteus_pi3hat.Pi3HatRouter(can=can_config)

    # Since we didn't specify a 'servo_bus_map' to Pi3HatRouter, this
    # will be assumed to be on bus 1.
    controller = moteus.Controller(id = 1, transport=transport)

    while True:
        # To send a raw CAN, you must manually instantiate a
        # 'moteus.Command' and fill in its fields, along with which
        # bus to send it on.
        raw_message = moteus.Command()
        raw_message.raw = True
        raw_message.arbitration_id = 0x0405
        raw_message.bus = 5
        raw_message.data = b'1234'
        raw_message.reply_required = False

        # A single 'transport.cycle' call's message list can contain a
        # mix of "raw" frames and those generated from
        # 'moteus.Controller'.
        #
        # If you want to listen on a CAN bus without having sent a
        # command with 'reply_required' set, you can use the
        # 'force_can_check' optional parameter.  It is a 1-indexed
        # bitfield listing which additional CAN buses should be
        # listened to.

        results = await transport.cycle([
            raw_message,
            controller.make_query(),
        ], force_can_check = (1 << 5))

        # If any raw CAN frames are present, the result list will be a
        # mix of moteus.Result elements and can.Message elements.
        # They each have the 'bus', 'arbitration_id', and 'data'
        # fields.
        #
        # moteus.Result elements additionally have an 'id' field which
        # is the moteus servo ID and a 'values' field which reports
        # the decoded response.
        for result in results:
            if hasattr(result, 'id'):
                # This is a moteus structure.
                print(f"{time.time():.3f} MOTEUS {result}")
            else:
                # This is a raw structure.
                print(f"{time.time():.3f} BUS {result.bus}  " +
                      f"ID {result.arbitration_id:x}  DATA {result.data.hex()}")

        await asyncio.sleep(1.0)


if __name__ == '__main__':
    asyncio.run(main())
