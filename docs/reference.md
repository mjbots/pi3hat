# mjbots pi3 hat #

This is the reference documentation for the mjbots pi3 hat r4.4b.
What is it?

The mjbots pi3hat is a daughterboard for a Raspberry Pi (tm), which
provides the following:

 * Power input from 8-44V, provides 5V for Raspberry Pi
 * 5x independent 5Mbps CAN-FD buses
 * A 1kHz IMU and attitude reference system
 * A spread spectrum nrf24l01 interface

# Pinouts #

## J1/J2 ##

XT30-M power input/output.  The chamfered side of the connector is
negative.

## JC1/JC2/JC3/JC4/JC5 ##

These are JST PH-3, 5Mbps CAN-FD connectors with 12V common mode
range.  Each has a physical 100 ohm terminator permanently installed.

* Pin 1: CANH
* Pin 2: CANL
* Pin 3: Ground

## J3 ##

This 8 pin connector allows a nrf24l01 module to be connected.

Pin 1 is closest to the J3 label.

* Pin 1: IRQ
* Pin 2: CIPO
* Pin 3: COPI
* Pin 4: SCK
* Pin 5: CS
* Pin 6: CE
* Pin 7: 3.3V
* Pin 8: GND

# Orientation #

For the purposes of the IMU, in the absence of any configured rotation
the reported axes are as follows:

* +x - towards the CAN connectors or HDMI connectors on the RPI
* +y - towards the USB ports on the rpi
* +z - up from the rpi towards the pi3hat

# Usage with client-side tools #

## tview / moteus_tool ##

First, install the python library:

```
sudo pip3 install moteus-pi3hat
```

Then both `moteus_tool` and `tview` will be aware of the pi3hat, and
will use its port JC1 by default if no fdcanusb is present.

To use other ports, you need to tell the tool which IDs are present on
which port using the `--pi3hat-cfg` command line option.  It has the
following syntax:

```
--pi3hat-cfg 'BUS1=ID1,ID2,ID3;BUS2=ID4,ID5,ID6
```

So, an example where servo ID 1 and 6 are present on port JC1 (bus 1),
and 3 and 7 are present on port JC2 (bus 2), would look like:

```
sudo python3 -m moteus_gui.tview --pi3hat-cfg '1=1,6;2=3,7' -t 1,3,6,7
```

The same `--pi3hat-cfg` works for `moteus_tool` as for `tview`.

## Python ##

Simply perform:

```
sudo pip3 install moteus-pi3hat
```

Then use the same API documented at:
https://github.com/mjbots/moteus/tree/main/lib/python

Alternately, the pi3hat transport can be constructed directly using:

```
import moteus
import moteus_pi3hat

transport = moteus_pi3hat.Pi3HatRouter(
    servo_bus_map={...},
)
controller = moteus.Controller(id=1, transport=transport)
```


## C++ ##

To use the pi3hat with C++, a library is provided at:
 https://github.com/mjbots/pi3hat/blob/master/lib/cpp/mjbots/pi3hat/pi3hat.h

It consists of a single standalone C++11 header and source file, which
depends upon nothing but the standard library.  The library provides a
blocking interface that allows all the ports and features to be used
in an efficient manner.  Internally, it bypasses the linux kernel
drivers (which requires it to be run as root), and sometimes
busy-waits the CPU, in order to achieve tight timing performance.

There is a sample user of this library, `pi3hat_tool` in the same
directory which provides a simple command line program that exercises
all of its functionality.

## Board level documentation ##

If you wish to use the existing linux kernel SPI drivers, or write a
custom driver, the board theory of operation and SPI register mappings
are described here.

Internally, the board has 3 separate STM32 processors, each attached
to a SPI bus on the Raspberry Pi:

* *Processor 1* The first processor provides the high speed CAN-FD
  interfaces for CAN connectors JC1 and JC2.  It is connected to the
  Raspberry Pi SPI1 bus, CS0.  Those are the following pins on the 40
  pin GPIO header:

  * Pin 35: CIPO (BCM 19)
  * Pin 38: COPI (BCM 20)
  * Pin 40: SCLK (BCM 21)
  * Pin 12: CS   (BCM 18)
  * Pin 37: IRQ  (BCM 26)

* *Processor 2*: This processor provides the CAN-FD interfaces for JC3
  and JC4.  It is connected to the Raspberry Pi SPI1 bus, CS1.  Pins:

  * Pin 35: CIPO (BCM 19)
  * Pin 38: COPI (BCM 20)
  * Pin 40: SCLK (BCM 21)
  * Pin 11: CS   (BCM 17)
  * Pin 29: IRQ  (BCM 5)

* *Processor 3*: This processor provides the low-speed CAN interface,
  the IMU, and the nrf24l01 logic.  It is connected to the Raspberry
  Pi SPI0 bus, CS0.

  * Pin 21: CIPO (BCM 9)
  * Pin 19: COPI (BCM 10)
  * Pin 23: SCLK (BCM 11)
  * Pin 24: CS   (BCM 8)
  * Pin 15: IRQ  (BCM 22)
  * Pin 16: IRQ  (BCM 23)
  * Pin 18: IRQ  (BCM 24)

# Debugging connectors #

## JP5 ##

This connector directly breaks out 3 pins from the Raspberry Pi:

 * 1: Pin 32 (BCM 12)
 * 2: Pin 33 (BCM 13)
 * 3: Ground

## JP4 ##

Reserved debugging connector for processor 1.

## JP3 ##

Reserved debugging connector for processor 2

## JP2 ##

Reserved debugging connector for processor 3

# SPI Protocol #

All of the processors use the same basic SPI protocol, with unique
registers for each of the various functions.

Maximum frequency: 10MHz

The following sequence is used for all SPI transactions (write or
read).

1. CS is pulled low
2. A >= 3 us hold time
3. An 8 byte address is written MSB first
4. A >= 3 us hold time
5. One or more bytes can now be read or written.  When reading, 0's
   should be clocked out.  When writing, the return value can be
   ignored.  Each "address" logically consists of sequence of bytes.
6. A >= 3 us hold time
7. CS is pulled high
8. A minimum 3us turnaround time is required before CS can be pulled
   low again.

## CAN Register Mapping ##

These addresses are present on processor 1, 2, and 3.

* *0* Protocol version: A constant byte 0x02
* *1* Interface type: A constant byte 0x01
* *2* Receive status
  * byte 0-5: Size of up to 6 received frames.  0 means no frame is
    available.  This size includes header information, so is the total
    number of bytes that must be read from address 3 to get a complete
    frame.
* *3* Received frame
  * Reading this address consumes exactly one frame from the received
    queue.
  * byte 0: (0x00 means no data)
    * bit 7: which port this was received on 0=JC1/JC3/JC5 1=JC2/JC4
    * bit 6-0: size of payload + 1 (1-65)
  * byte 1-4: CAN ID, MSB first
  * byte 5+: payload
* *4* Transmit frame with 4 byte ID
  * Writing to this address enqueues one CAN frame to be sent.
  * byte 0:
    * bit 7: which port to send on 0=JC1/JC3/JC5 1=JC2/JC4
    * bit 6-0: size of payload
  * byte 1-4: CAN ID, MSB first
  * byte 5+: payload
* *5* Transmit frame with 2 byte ID
  * Writing to this address enqueues one CAN frame to be sent.
  * byte 0:
    * bit 7: which port to send to 0=JC1/JC3/JC5 1=JC2/JC4
    * bit 6-0: size of payload
  * byte 1-2: CAN ID, MSB first
  * byte 3+: payload
* *6* Error status (read only)
  * Port 0 (JC1/JC3/JC5)
    * byte 0
      * bit 7-6 activity
      * bit 5-3 data last error code
      * bit 2-0 last error code
    * byte 1
      * bit 3 protocol exception
      * bit 2 bus_off
      * bit 1 warning status
      * bit 0 error passive
    * byte 2 - rx error count
    * byte 3 - tx error count
    * byte 4 - reset count
    * byte 5 - reserved
  * Port 1 (JC2/JC4)
    * byte 6
      * bit 7-6 activity
      * bit 5-3 data last error code
      * bit 2-0 last error code
    * byte 7
      * bit 3 protocol exception
      * bit 2 bus_off
      * bit 1 warning status
      * bit 0 error passive
    * byte 8 - rx error count
    * byte 9 - tx error count
    * byte 10 - reset count
    * byte 11 - reserved
* *7* Read configuration Port 1 (JC1/JC3/JC5)
  * byte 0-3: int32 standard bitrate
  * byte 4-7: int32 CAN-FD bitrate
  * byte 8: generate CAN-FD frames
  * byte 9: switch bitrates
  * byte 10: enable automatic retransmission
  * byte 11: restricted mode
  * byte 12: bus monitor mode
  * byte 13: std prescaler
  * byte 14: std sync_jump_width
  * byte 15: std time_seg1
  * byte 16: std time_seg2
  * byte 17: std prescaler
  * byte 18: std sync_jump_width
  * byte 19: std time_seg1
  * byte 20: std time_seg2
* *8* Read configuration Port 2 (JC2/JC4)
  * Same as for 7
* *9* Write configuration Port 1 (JC1/JC3/JC5)
  * Same as for 7
* *10* Write configuration Port 2 (JC2/JC4)
  * Same as for 7


## IMU Register Mapping ##

These addresses are present only on processor 3.

* *32* Protocol version: A constant byte 0x20
* *33* Raw IMU data
  * uint16 _present_
  * float _gx dps_
  * float _gy dps_
  * float _gz dps_
  * float _ax mps2_
  * float _ay mps2_
  * float _az mps2_
* *34* Attitude Data
  * uint8 _present_
  * uint8 _reserved_
  * float _w_
  * float _x_
  * float _y_
  * float _z_
  * float _x dps_
  * float _y dps_
  * float _z dps_
  * float _ax mps2_
  * float _ay mps2_
  * float _az mps2_
  * float _bias x dps_
  * float _bias y dps_
  * float _bias z dps_
  * float _uncertainty w_
  * float _uncertainty x_
  * float _uncertainty y_
  * float _uncertainty z_
  * float _uncertainty bias x dps_
  * float _uncertainty bias y dps_
  * float _uncertainty bias z dps_
  * uint32 _error count_
  * uint32 _last error_
* *35* Read configuration
  * float _roll deg_
  * float _pitch deg_
  * float _yaw deg_
  * uint32t _rate Hz_
* *36* Write configuration
  * The same structure as for address 35.

## RF Register Mapping ##

If a nrf24l01 module is attached, the pi3hat implements a spread
spectrum receiver.  This receiver must be configured with the same ID
as the transmitter for communication to take place.

Each "slot" is transmitted independently and repeatedly according to
the given bitmask schedule.  The individual RF frames occur at 50Hz,
and each bit in the bitmask controls whether the data should be sent
in that particular frame.  So 0xaaaaaaaa would result in the data
being sent every other frame.

* *48* Protocol version: A constant byte 0x11
* *49* Read ID
  * byte 0-3: Common RF ID
* *50* Write ID
* *51* Set data for transmission
  * byte 0: Slot number (0-15)
  * byte 1-4: Transmission schedule (little endian)
  * byte 5+: Data
* *52* Read status
  * byte 0-3: uint32 rx slot counter, 2 bits per slot (little endian)
    * each time new data is received from the transmitter, the 2 bit
      counter for that slot is incremented
  * byte 4-7: uint32 age since last lock from tx (little endian)
* *64-79* Read data from remote transmitter
  * byte 0-3: uint32 age in ms (little endian)
  * byte 4: size (0-16)
  * byte 5-20: data


# Flashing new firmware #

To flash new firmware, openocd 0.11.0 or newer is required.  You can find binaries for many platforms at: https://xpack.github.io/openocd/releases/

Flashing can be performed using the `flash.py` script, for instance to flash a newly compiled image:

```
./flash.py
```

Or to flash a specific .elf file

```
./flash.py 20211129-pi3hat-71c002e71ce804959e681dc7043f6fb5d8147cac.elf
```

Each of the three processors needs to be flashed with the same image.

# Optional Raspberry Pi SD card image #

The optional Raspberry Pi with SD card comes with a non-GUI image
flashed on it from Raspberry Pi.  Additionally, it is configured to
present as a 5GHz wifi access point.  The SSID is mjbots-xxxxxx, and
the password is "WalkingRobots".  The user name and password are the
default for the rpi, pi/raspberry

The wifi IP address is 192.168.16.47, you can ssh to it from linux:

```
ssh pi@192.168.16.47
```

The ethernet address is 192.168.17.47.

In the home directory is a copy of the compiled `pi3hat_tool`, you can
run it:

```
pi@realtimepi:~ $  sudo bash

root@realtimepi:/home/pi# ./pi3hat_tools/pi3hat_tool --read-att
```

Which will then display the current attitude information.
