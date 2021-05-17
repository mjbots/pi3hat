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

"""Classes and functions for interoperating with the moteus brushless
controller."""

from moteus_pi3hat.pi3hat_router import (
    Pi3HatRouter, CanAttitudeWrapper, CanConfiguration, CanRateOverride)

class Pi3HatFactory():
    PRIORITY = 5

    name = 'pi3hat'

    def add_args(self, parser):
        parser.add_argument('--pi3hat-cpu', type=int, metavar='CPU',
                            help='CPU used for busy-looping on pi3hat')
        parser.add_argument('--pi3hat-spi-hz', type=int, metavar='HZ',
                            help='SPI speed to use')
        parser.add_argument('--pi3hat-cfg', metavar='CFG',
                            help='1=ID1,ID2;N=IDX,IDY...')

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

        return Pi3HatRouter(**kwargs)

__all__ = [
    'Pi3HatRouter',
    'Pi3HatFactory',
    'CanAttitudeWrapper',
    'CanConfiguration',
    'CanRateOverride',
]
