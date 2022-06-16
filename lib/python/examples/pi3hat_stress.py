#!/usr/bin/python3

# Copyright 2022 Josh Pieper, jjp@pobox.com.
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

'''Test CAN performance.  Assumes that a loopback cable is connected
between bus 1 and 3.'''


import argparse
import asyncio
import moteus
import moteus_pi3hat
import time
import sys


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--count', '-c', type=int, default=2)

    moteus.make_transport_args(parser)

    args = parser.parse_args()

    transport = moteus_pi3hat.Pi3HatFactory()(args)

    test_count = 0

    while True:
        print(f'{test_count}')
        test_count += 1

        raw_message = moteus.Command()
        raw_message.raw = True
        raw_message.arbitration_id = 0x0405
        raw_message.bus = 1
        raw_message.reply_required = False

        received = []

        for i in range(args.count):
            raw_message.data = bytes(list(range(i + 2, i + 50)))

            results = await transport.cycle(
                [raw_message],
                force_can_check = (1 << 3),
            )

            for result in results:
                assert not hasattr(result, 'id')
                received.append(result)

        start = time.time()

        while len(received) < args.count:
            results = await transport.cycle([], force_can_check = (1 << 3))
            for result in results:
                assert not hasattr(result, 'id')
                received.append(result)

            if (time.time() - start) > 1.0:
                break

        if len(received) < args.count:
            print()
            print("Mismatch!")
            print(f"received: {received}")
            sys.exit(1)

        time.sleep(0.02)


if __name__ == '__main__':
    asyncio.run(main())
