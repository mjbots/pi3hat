# Copyright 2020-2021 Josh Pieper, jjp@pobox.com.
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


CanRateOverride = _pi3hat_router.CanRateOverride
CanConfiguration = _pi3hat_router.CanConfiguration


class CanAttitudeWrapper:
    def __init__(self, attitude):

        # Set some variables so we look a little bit like a CAN
        # response.
        self.id = -1
        self.arbitration_id = -1
        self.bus = -1
        self.values = []

        self.attitude = attitude.attitude
        self.rate_dps = attitude.rate_dps
        self.accel_mps2 = attitude.accel_mps2
        self.euler_rad = attitude.euler_rad


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
                 mounting_deg = None,
                 attitude_rate_hz = None,
                 can = None,
                 servo_bus_map = None):
        """Initialize.

        :param cpu: The device interface will run on this CPU

        :param spi_speed_hz: How fast to run the SPI bus

        :param servo_bus_map: A map of buses to servo ids: {bus:
          [list, of, ids, ...})
        """

        self.servo_bus_map = servo_bus_map or {}

        options = _pi3hat_router.Options()
        options.cpu = cpu
        options.spi_speed_hz = spi_speed_hz

        if mounting_deg:
            options.mounting_deg.pitch = mounting_deg['pitch']
            options.mounting_deg.roll = mounting_deg['roll']
            options.mounting_deg.yaw = mounting_deg['yaw']

        if attitude_rate_hz:
            options.attitude_rate_hz = attitude_rate_hz

        if can:
            # pybind11 arrays are returned by copy, thus element
            # modifications are not possible.  Thus we just create a
            # full array to assign.
            options.can = [can[i] if i in can else CanConfiguration()
                           for i in range(1, 6)]

        self._impl = _pi3hat_router.Pi3HatRouter(options)

    def _find_bus(self, destination):
        for key, value in self.servo_bus_map.items():
            if destination in value:
                return key
        return 1

    def _make_single_can(self, command):
        result = _pi3hat_router.SingleCan()

        if getattr(command, 'raw', False):
            result.arbitration_id = command.arbitration_id
            result.bus = command.bus
        else:
            result.bus = self._find_bus(command.destination)
            result.arbitration_id = command.destination | (0x8000 if command.reply_required else 0x0000)
        result.data = command.data
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

    async def cycle(self, commands,
                    force_can_check=0,
                    max_rx=-1,
                    timeout_ns=1000000,
                    min_tx_wait_ns=1000000,
                    rx_extra_wait_ns=40000,
                    request_attitude=False):
        '''Operate one CAN cycle of the pi3hat

        :param commands: A list of moteus.Command structures

        :param force_can_check: A bitmask list CAN channels which
          should be listened to, even if no commands on that bus were
          marked as expecting a reply.

        :param max_rx: The maximum number of receive packets to
          return, the default -1, means return as many as possible.

        :param timeout_ns: If specified, require waiting at least this
        long for replies.

        :param min_tx_wait_ns: If specified, change the amount of time
        waited after the last transmit on a given bus if replies are
        expected.

        :param rx_extra_wait_ns: After a successful receipt, wait this
        much longer for more replies.

        :param request_attitude: If True, then the attitude will be
        queried in the same cycle, and returned as a message with
        id=-1/bus=-1 and the type moteus_pi3hat.CanAttitudeWrapper.

        '''
        input = _pi3hat_router.Input()

        input.tx_can = [self._make_single_can(command) for command in commands]
        input.force_can_check = force_can_check
        input.max_rx = max_rx
        input.timeout_ns = timeout_ns
        input.min_tx_wait_ns = min_tx_wait_ns
        input.rx_extra_wait_ns = rx_extra_wait_ns
        input.request_attitude = request_attitude

        output = await self._cycle(input)

        result = []
        for single_rx in output.rx_can:
            # For commands that were raw, we can't associate them with
            # a response.  They will just get returned as a python-can
            # style class instead of a parsed structure.
            maybe_command = [x for x in commands if
                             (not getattr(x, 'raw', False) and
                              x.destination == (single_rx.arbitration_id) >> 8)
                             ]
            if maybe_command:
                command = maybe_command[0]
                result.append(command.parse(single_rx))
            else:
                # We didn't associate this with a command, so just
                # return it raw.
                result.append(single_rx)

        if output.attitude_present:
            # For now, we return this as a "pseudo-can" message, with
            # a negative ID.
            result.append(CanAttitudeWrapper(output.attitude))

        return result

    async def write(self, command):
        '''Write a single message.'''
        input = _pi3hat_router.Input()

        input.tx_can = [self._make_single_can(command)]
        input.max_rx = 0

        await self._cycle(input)
        return []

    async def read(self):
        '''Read at most one message from any BUS.'''
        input = _pi3hat_router.Input()
        input.force_can_check = 0x3f
        input.max_rx = 1

        output = await self._cycle(input)

        return None if not output.rx_can else output.rx_can[0]

    async def attitude(self):
        '''Return the current IMU value

        The return object will have the following fields:
         - attitude: a Quaternion with w, x, y, z fields
         - rate_dps: a Point3D with x, y, z fields
         - accel_mps2: a Point3D with x, y, z fields
         - euler_rad: a Euler with roll, pitch, yaw fields (redundant with attitude)
        '''

        input = _pi3hat_router.Input()
        input.request_attitude = True

        output = await self._cycle(input)

        return output.attitude if output.attitude_present else None
