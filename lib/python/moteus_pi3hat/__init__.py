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

"""Classes and functions for interoperating with the moteus brushless
controller."""

import argparse

from moteus_pi3hat.pi3hat_device import (
    Pi3HatDevice, Pi3HatChildDevice, CanAttitudeWrapper, CanConfiguration, CanRateOverride)

from moteus import DeviceAddress, TransportWrapper

class Pi3HatFactory():
    PRIORITY = 5

    name = 'pi3hat'

    def add_args(self, parser):
        try:
            parser.add_argument('--can-disable-brs', action='store_true',
                                help='do not set BRS')
        except argparse.ArgumentError:
            # It must already be set.
            pass

        parser.add_argument('--pi3hat-cpu', type=int, metavar='CPU',
                            help='CPU used for busy-looping on pi3hat')
        parser.add_argument('--pi3hat-spi-hz', type=int, metavar='HZ',
                            help='SPI speed to use')
        parser.add_argument('--pi3hat-cfg', metavar='CFG',
                            help='1=ID1,ID2;N=IDX,IDY...')
        parser.add_argument('--pi3hat-disable-aux', action='store_true',
                            help='Prevent use of the IMU/JC5')

    def is_args_set(self, args):
        return args and (args.pi3hat_cfg or args.pi3hat_cpu or args.pi3hat_spi_hz)

    def __call__(self, args):
        kwargs = {}
        if args:
            if args.pi3hat_cfg:
                servo_bus_map = {}
                buses = args.pi3hat_cfg.split(';')
                for bus in buses:
                    name, ids = bus.split('=')
                    servo_bus_map[int(name)] = [int(x) for x in ids.split(',')]
                kwargs['servo_bus_map'] = servo_bus_map
            if args.pi3hat_cpu:
                kwargs['cpu'] = args.pi3hat_cpu
            if args.pi3hat_spi_hz:
                kwargs['spi_speed_hz'] = args.pi3hat_spi_hz
            if args.pi3hat_disable_aux:
                kwargs['enable_aux'] = False
            if args.can_disable_brs:
                kwargs['disable_brs'] = True
            if args.can_debug:
                kwargs['debug_log'] = args.can_debug

        parent = Pi3HatDevice(**kwargs)
        return [Pi3HatChildDevice(parent, bus) for bus in [1, 2, 3, 4, 5]]


class Pi3HatRouter(TransportWrapper):
    def __init__(self, *args, **kwargs):
        # The new "transport" mechanism automatically detects which
        # devices are on which bus in most cases and uses the
        # 'routing_table' argument when we don't want that to be the
        # case.  Map the legacy 'servo_bus_map' onto that mechanism.
        servo_bus_map = kwargs.pop('servo_bus_map', {})

        self._parent = Pi3HatDevice(**kwargs)
        children = { bus: Pi3HatChildDevice(self._parent, bus)
                     for bus in [1, 2, 3, 4, 5] }
        self._devices = list(children.values())

        routing_table = {}
        for bus, id_list in servo_bus_map.items():
            for id in id_list:
                routing_table[DeviceAddress(can_id=id)] = children[bus]

        super().__init__(self._devices, routing_table=routing_table)

    async def attitude(self):
        return await self._parent.attitude()


__all__ = [
    'Pi3HatDevice',
    'Pi3HatChildDevice',
    'Pi3HatFactory',
    'Pi3HatRouter',
    'CanAttitudeWrapper',
    'CanConfiguration',
    'CanRateOverride',
]
