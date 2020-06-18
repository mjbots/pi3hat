# mjbots pi3hat #

This contains designs, firmware, and client side libraries for the
mjbots pi3hat.  It is a daughterboard for Raspberry Pi (tm) boards and
similar to provide:

* 4x 5Mbps CAN-FD ports
* 1x 125kbps CAN port
* 1kHz IMU with attitude reference

# LICENSE #

All files contained in this repository, unless otherwise noted, are
available under an Apache 2.0 License
https://www.apache.org/licenses/LICENSE-2.0

# Directory Structure #

* hw/ - Eagle schematic
* fw/ - Firmware
* docs/ - Reference documentation
* mjbots/pi3hat/ - Client side C++ library
* tools/ - bazel build configuration

# FAQ #

## How do I get the best timing performance? ##

A: We recommend using the following configuration:
 * `isolcpus` and `sched_setaffinity` used to keep linux from running
   on the processor interfacing with the pi3hat
 * `chrt 99` to run the process at the maximum real time priority
 * `mlockall` to prevent linux from swapping any part of the process

## Will this work on a Raspberry Pi 4? ##

A: Despite the name `pi3hat`, yes it works just fine on a Raspberry Pi
4 too.

## Will this work on **insert random board here**? ##

A: Maybe?  If it has the same pinout as a Raspberry Pi, can can be
powered through the GPIO header with 5V @ 2.5A then there is a chance.

## Can I use the Raspberry Pi Camera? ##

A: If you install a higher GPIO riser then it is possible.  Samtec
ESW-120-12-G-D works.  You can install one yourself, or contact
info@mjbots.com to get one fitted at the factory.
