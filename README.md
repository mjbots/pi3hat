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

* How do I get the best timing performance?

A: We recommend using the following configuration:
 * A `preempt_rt` linux kernel (
http://unofficialpi.org/Distros/RealtimePi/nightly/old/2019-05-14_2019-04-08-realtimepi-stretch-lite-0.4.0.zip
   from https://github.com/guysoft/RealtimePi/releases is known to
   work pretty well)
 * `isolcpus` and `sched_setaffinity` used to keep linux from running
   on the processor interfacing with the pi3hat
 * `chrt 99` to run the process at the maximum real time priority
 * `mlockall` to prevent linux from swapping any part of the process

* Will this work on a Raspberry Pi 4?

A: Likely, however, to achieve optimal performance you will want to
use the `preempt_rt` kernel.  The only pre-built `preempt_rt` kernels
for the pi 4 are 4.19 or later with buster.  Currently those images
have undiagnosed timing or throttling issues and aren't recommended.

This will be updated when those issues have been resolved and testing
has been done on a pi 4.

* Will this work on **insert random board here**?

A: Maybe?  If it has the same pinout as a Raspberry Pi, can can be
powered through the GPIO header with 5V 2.5A then there is a chance.
