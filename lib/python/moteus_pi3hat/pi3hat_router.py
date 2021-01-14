# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

import asyncio
import pathlib
import sys

here = str(pathlib.Path(__file__).parent.resolve())
sys.path.insert(0, here)

import _pi3hat_router


def _set_future(future, output):
    if not future.cancelled():
        future.set_result(output)


class Pi3HatRouter:
    """Permits communication using the pi3hat CAN interfaces.  This
    requires a dedicated Raspberry PI CPU to operate the hardware.  It
    is recommended that you configure the selected CPU using isolcpus
    so that no other processes use it.

    At construction time, you must provide a list of which servo IDs
    are present on which pi3hat bus.  Communication with servos not in
    this list will use bus 1.
    """

    def __init__(self,
                 cpu = 3,
                 spi_speed_hz = 10000000,
                 servo_bus_map = None):
        """Args:

          cpu: The device interface will run on this CPU
          spi_speed_hz: How fast to run the SPI bus
          servo_bus_map: A list of tuples, (bus, [list, of, ids])
        """

        self.servo_bus_map = servo_bus_map or {}

        options = _pi3hat_router.Pi3HatRouterOptions()
        options.cpu = cpu
        options.spi_speed_hz = spi_speed_hz

        self._impl = _pi3hat_router.Pi3HatRouter(options)

    def _find_bus(self, destination):
        for key, value in self.servo_bus_map.items():
            if destination in value:
                return key
        return 1

    def _make_single_can(self, command):
        result = _pi3hat_router.SingleCan()
        result.arbitration_id = command.destination | (0x8000 if command.reply_required else 0x0000)
        result.data = command.data
        result.bus = self._find_bus(command.destination)
        result.expect_reply = command.reply_required
        return result

    async def _cycle(self, input):
        loop = asyncio.get_event_loop()
        future = asyncio.Future(loop=loop)

        def handle_output(output):
            if loop.is_closed():
                print("LOOP UNEXPECTEDLY CLOSED")
            else:
                loop.call_soon_threadsafe(_set_future, future, output)

        self._impl.cycle(input, handle_output)
        # We are forbidden to call "cycle" multiple times, thus we
        # have to actually wait for it to finish before we can let any
        # timeouts or other cancellations propagate up.
        result = await asyncio.shield(future)

        return result

    async def cycle(self, commands):
        input = _pi3hat_router.Input()

        input.tx_can = [self._make_single_can(command) for command in commands]

        output = await self._cycle(input)

        result = []
        for single_rx in output.rx_can:
            maybe_command = [x for x in commands if
                             x.destination == (single_rx.arbitration_id) >> 8]
            if maybe_command:
                command = maybe_command[0]
                result.append(command.parse(single_rx))
        return result

    async def write(self, command):
        input = _pi3hat_router.Input()

        input.tx_can = [self._make_single_can(command)]
        await self._cycle(input)
        return []

    async def read(self):
        input = _pi3hat_router.Input()
        input.force_can_check = 0x3f
        input.max_rx = 1

        output = await self._cycle(input)

        return None if not output.rx_can else output.rx_can[0]
