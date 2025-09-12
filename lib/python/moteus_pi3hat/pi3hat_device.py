# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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
import itertools
import moteus
import pathlib
import sys
import time
import typing

from moteus import Frame, FrameFilter, TransportDevice

here = str(pathlib.Path(__file__).parent.resolve())
sys.path.insert(0, here)

import _pi3hat_device


def _set_future(future, output):
    if not future.cancelled():
        future.set_result(output)


async def _await_mandatory(aw: typing.Awaitable[typing.Any]) -> typing.Any:
    future = asyncio.ensure_future(aw)
    try:
        return await asyncio.shield(future)
    except asyncio.CancelledError:
        # We've had a request for cancellation.  We still need to wait
        # for the future to finish before allowing it.
        await asyncio.shield(future)

        # Now we can let the cancellation proceed.
        raise


CanRateOverride = _pi3hat_device.CanRateOverride
CanConfiguration = _pi3hat_device.CanConfiguration


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


class Pi3HatDevice(TransportDevice):
    """Permits communication using the pi3hat CAN interfaces.  This
    requires a dedicated Raspberry PI CPU to operate the hardware.  It
    is recommended that you configure the selected CPU using isolcpus
    so that no other processes use it.

    At construction time, you may provide a list of which servo IDs
    are present on which pi3hat bus.  Communication with non-broadcast
    servos not in this list will use bus 1.

    """

    def __init__(self,
                 cpu = 3,
                 spi_speed_hz = 10000000,
                 mounting_deg = None,
                 attitude_rate_hz = None,
                 enable_aux = True,
                 disable_brs = False,
                 can = None,
                 servo_bus_map = None,
                 debug_log = None,
                 **kwargs):
        """Initialize.

        :param cpu: The device interface will run on this CPU

        :param spi_speed_hz: How fast to run the SPI bus

        :param servo_bus_map: A map of buses to servo ids: {bus:
          [list, of, ids, ...})
        """

        super(Pi3HatDevice, self).__init__(**kwargs)

        self._lock = asyncio.Lock()
        self._children = {}
        self.servo_bus_map = servo_bus_map or {}
        self._debug_log = debug_log

        options = _pi3hat_device.Options()
        options.cpu = cpu
        options.spi_speed_hz = spi_speed_hz
        options.enable_aux = enable_aux

        if mounting_deg:
            options.mounting_deg.pitch = mounting_deg['pitch']
            options.mounting_deg.roll = mounting_deg['roll']
            options.mounting_deg.yaw = mounting_deg['yaw']

        if attitude_rate_hz:
            options.attitude_rate_hz = attitude_rate_hz


        def update_brs(x):
            if disable_brs:
                x.bitrate_switch = False
            return x

        # pybind11 arrays are returned by copy, thus element
        # modifications are not possible.  Thus we just create a
        # full array to assign.
        options.can = [update_brs(can[i] if (can and i in can) else
                                  CanConfiguration())
                       for i in range(1, 6)]

        self._impl = _pi3hat_device.Pi3HatDevice(options)

        device_info = self._impl.device_info()
        self._empty_bus_tx_safe = device_info.can_unknown_address_safe

    def __repr__(self):
        return 'pi3hat()'

    def empty_bus_tx_safe(self):
        return self._empty_bus_tx_safe

    async def receive_frame(self) -> Frame:
        '''Read one frame from any port.'''
        while True:
            # Until we have something in the receive queue, keep waiting.
            if self._receive_queue:
                return self._receive_queue.pop(0)

            # We have nothing.  Attempt to read something
            # for it and try again.
            request = TransportDevice.Request(
                frame=None,
                frame_filter=None)

            await self.transaction(
                [request],
                force_can_check=62, # 2 | 4 | 8 | 16 | 32
                max_rx=16)

    async def send_frame(self, frame: Frame):
        raise NotImplementedError()

    def _make_pi3hat_from_request(self, request):
        assert request.child_device is not None

        result = _pi3hat_device.SingleCan()
        frame = request.frame

        result.arbitration_id = frame.arbitration_id
        result.bus = request.child_device.bus()

        result.data = frame.data
        result.expect_reply = request.frame_filter is not None
        result.expected_reply_size = request.expected_reply_size

        return result

    def _make_frame_from_pi3hat(self, message):
        if isinstance(message, CanAttitudeWrapper):
            return message
        return Frame(
            arbitration_id=message.arbitration_id,
            data=message.data,
            dlc=message.dlc,
            is_extended_id=message.is_extended_id,
            is_fd=message.is_fd,
            bitrate_switch=message.bitrate_switch,
            channel=self._children[message.bus])

    async def _async_cycle(self, input):
        if self._debug_log:
            for tx in input.tx_can:
                self._write_log(f'{tx.bus} > {tx.arbitration_id:04X} {tx.data.hex().upper()}'.encode('latin1'))

        loop = asyncio.get_event_loop()
        future = asyncio.Future()

        def handle_output(output):
            loop.call_soon_threadsafe(_set_future, future, output)

        self._impl.cycle(input, handle_output)

        result = await _await_mandatory(future)

        if self._debug_log:
            for rx in result.rx_can:
                self._write_log(f'{rx.bus} < {rx.arbitration_id:04X} {rx.data.hex().upper()}'.encode('latin1'))

        return result

    async def transaction(
            self,
            requests: typing.List[TransportDevice.Request],
            force_can_check=0,
            max_rx=-1,
            request_attitude=False,
            **kwargs):
        async with self._lock:
            # We know the transaction itself will not take long.  Thus
            # we don't want to let it be cancelled, and potentially
            # lose frames.
            await self._transaction(
                requests,
                force_can_check=force_can_check,
                max_rx=max_rx,
                request_attitude=request_attitude)

            # It doesn't matter so much we are cancelled ourselves, as
            # we don't return anything via 'return' anyway.

    async def _transaction(
            self,
            requests: typing.List[TransportDevice.Request],
            force_can_check,
            max_rx,
            request_attitude):
        input = _pi3hat_device.Input()

        input.tx_can = [
            self._make_pi3hat_from_request(request)
            for request in requests
            if request.frame is not None
        ]

        input.max_rx = max_rx
        input.request_attitude = request_attitude

        def make_subscription(request):
            future = asyncio.Future()

            async def handler(frame, request=request, future=future):
                if future.done():
                    # Stick it in our receive queue so that it
                    # isn't lost.
                    self._receive_queue.append(frame)

                    return

                request.responses.append(frame)
                # While we may receive more than one frame for a given
                # request, we only wait for one.
                future.set_result(None)

            filter_wrapper = (
                lambda frame: (request.child_device is None
                               or request.child_device == frame.channel)
                and request.frame_filter(frame))

            return self._subscribe(filter_wrapper, handler), future

        subscriptions = [
            make_subscription(request)
            for request in requests
            if request.frame_filter is not None
        ]

        try:
            while True:
                remaining_busses_to_check = set(
                    [request.child_device.bus() for request in requests
                     if (request.child_device and
                         request.frame_filter and
                         len(request.responses) == 0)])
                input.force_can_check = (
                    (force_can_check or 0) + sum(
                        [1 << x for x in remaining_busses_to_check]))

                output = await self._async_cycle(input)

                output_frames = output.rx_can

                if output.attitude_present:
                    attitude_frame = CanAttitudeWrapper(output.attitude)
                    # Give it to any request that has no frame, but
                    # has a frame_filter.
                    for request in requests:
                        if (not request.frame
                            and request.frame_filter and
                            request.frame_filter(attitude_frame)):


                            output_frames.append(attitude_frame)


                # Associate the replies we have received so far with
                # requests.
                for single_rx in output_frames:
                    frame = self._make_frame_from_pi3hat(single_rx)
                    await self._handle_received_frame(frame)

                # Give everything else a chance to run and potentially
                # mark themselves as done.
                await asyncio.sleep(0)

                # Need to iterate until we are either cancelled, or until
                # all the requests that need replies have them.

                # If all subscriptions have been processed, then we
                # are done.
                if all([x[1].done() for x in subscriptions]):
                    return

                # Remove all tx messages and attitude requests for
                # subsequent iterations.
                input.tx_can = []
                input.request_attitude = False

                # On subsequent attempts, always try to read
                # something.
                input.max_rx = 16
        finally:
            for x in subscriptions:
                x[0].cancel()

    async def attitude(self):
        async with self._lock:
            input = _pi3hat_device.Input()
            input.request_attitude = True

            output = await self._async_cycle(input)

            return output.attitude if output.attitude_present else None

    def close(self):
        self._impl = None

    def _write_log(self, output: bytes):
        assert self._debug_log is not None
        self._debug_log.write(f'{time.time():.6f} '.encode('latin1') + output + b'\n')

class Pi3HatChildDevice(TransportDevice):
    def __init__(self, parent, bus):
        self._parent = parent
        self._bus = bus

        self._parent._children[bus] = self

    def __repr__(self):
        return f"pi3hat('JC{self._bus}')"

    def parent(self):
        return self._parent

    def empty_bus_tx_safe(self):
        # Legacy pi3hat's treated JC1 as special and always assumed
        # there was something there.
        if self._bus == 1:
            return True
        return self._parent._empty_bus_tx_safe

    def bus(self):
        return self._bus

    async def send_frame(self, frame: Frame):
        request = TransportDevice.Request(
            frame=frame,
            child_device=self,
            frame_filter=None)

        return await self._parent.transaction([request])

    async def receive_frame(self) -> Frame:
        while True:
            # Until our parent has something in the receive queue for this
            # bus, keep waiting.
            for i, frame in enumerate(self._parent._receive_queue):
                if frame.channel == self:
                    # This is ours, we can return it.
                    self._parent._receive_queue = (
                        self._parent._receive_queue[0:i] +
                        self._parent._receive_queue[i+1:])
                    return frame

            # We have nothing for this bus.  Attempt to read something
            # for it and try again.
            request = TransportDevice.Request(
                frame=None,
                child_device=self,
                frame_filter=None)

            await self._parent.transaction(
                [request],
                force_can_check=(1 << self._bus),
                max_rx=2)

    async def transaction(
            self,
            requests: typing.List[TransportDevice.Request]):
        for request in requests:
            request.child_device = this
        return await self._parent.transaction(requests)
