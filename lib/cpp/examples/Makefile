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


# This is a simple Makefile to demonstrate compiling the pi3hat
# examples in-situ on a Raspberry Pi.

# On Ubuntu, you may need to:
#  sudo apt install libraspberrypi-dev

EXAMPLES = \
	simple \
	diagnostic_protocol \
	multiple_cycle \
	simple_teleop \
	nlateral_teleop \
	timeout_debug \
	bandwidth_test \
	imu_transport

AUX_SOURCES = pi3hat.cc pi3hat_moteus_transport_register.cc

% :: %.cc $(AUX_SOURCES) *.h
	g++ -O2 \
		-mcpu=cortex-a53 \
		-g -Wall -Werror -std=c++17 -Wno-psabi \
		-I /opt/vc/include \
		-L /opt/vc/lib \
		-o $@ \
		pi3hat.cc \
		pi3hat_moteus_transport_register.cc \
		$< \
		-lpthread


all : $(EXAMPLES)
