// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! The core pi3hat driver.
//!
//! This is a direct translation of `lib/cpp/mjbots/pi3hat/pi3hat.cc`.
//! It accesses the BCM2835/6/7/2711 SPI0 and AUX SPI peripherals
//! directly through `/dev/mem`, so it requires root.  The relevant
//! kernel SPI drivers must not be in active use.

// Like the C++ library, register values are written out with one
// (commented) term per field, including the zero ones.
#![allow(clippy::identity_op, clippy::eq_op)]

use std::fs::{File, OpenOptions};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use crate::error::{Error, Result};

/// A single CAN frame, sent or received on one of the pi3hat buses.
#[derive(Clone, Copy, Debug)]
pub struct CanFrame {
    pub id: u32,
    pub data: [u8; 64],
    /// The number of valid bytes in `data`, at most 64.  A larger
    /// value is a caller bug and panics during [`Pi3Hat::cycle`]
    /// (the C++ library silently sends garbage instead).
    pub size: u8,

    /// Bus 1, 2, 3, 4, 5 are the buses labeled JC1 through JC5.
    /// Bus 5 is serviced by the auxiliary processor (see
    /// [`Configuration::enable_aux`]).
    pub bus: i32,

    /// If true, then a reply will be expected for this frame on the
    /// same bus.
    pub expect_reply: bool,

    /// If set, this is used as a hint to increase delays.  If unset,
    /// then the minimum delay may need to be increased.
    pub expected_reply_size: u8,
}

impl Default for CanFrame {
    fn default() -> Self {
        Self {
            id: 0,
            data: [0u8; 64],
            size: 0,
            bus: 0,
            expect_reply: false,
            expected_reply_size: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Euler {
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[non_exhaustive]
pub struct Attitude {
    pub attitude: Quaternion,
    pub rate_dps: Point3D,
    pub accel_mps2: Point3D,

    pub bias_dps: Point3D,
    pub attitude_uncertainty: Quaternion,
    pub bias_uncertainty_dps: Point3D,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RfSlot {
    pub slot: u8,
    pub priority: u32,
    /// The number of valid bytes in `data`, at most 16.  A larger
    /// value in a transmitted slot is a caller bug and panics during
    /// [`Pi3Hat::cycle`]; in received slots the driver clamps it.
    pub size: u8,
    pub data: [u8; 16],
    pub age_ms: u32,
}

/// If any of these fields are non-negative, then they are used
/// instead of the "bitrate" options in the CAN configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanRateOverride {
    pub prescaler: i32,
    pub sync_jump_width: i32,
    pub time_seg1: i32,
    pub time_seg2: i32,
}

impl Default for CanRateOverride {
    fn default() -> Self {
        Self {
            prescaler: -1,
            sync_jump_width: -1,
            time_seg1: -1,
            time_seg2: -1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CanConfiguration {
    pub slow_bitrate: i32,
    pub fast_bitrate: i32,
    pub fdcan_frame: bool,
    pub bitrate_switch: bool,
    pub automatic_retransmission: bool,

    /// NOTE: This field is currently ignored.  Like the C++ library,
    /// the driver does not transmit `restricted_mode` to the device,
    /// so setting it has no effect.
    pub restricted_mode: bool,

    pub bus_monitor: bool,

    pub std_rate: CanRateOverride,
    pub fd_rate: CanRateOverride,

    pub cancel_all_ms: u32,
}

impl Default for CanConfiguration {
    fn default() -> Self {
        Self {
            slow_bitrate: 1000000,
            fast_bitrate: 5000000,
            fdcan_frame: true,
            bitrate_switch: true,
            automatic_retransmission: true,
            restricted_mode: false,
            bus_monitor: false,
            std_rate: CanRateOverride::default(),
            fd_rate: CanRateOverride::default(),
            cancel_all_ms: 50,
        }
    }
}

/// This struct (like the other input structs) is deliberately
/// exhaustive so it can be built with functional-update syntax
/// (`Configuration { spi_speed_hz, ..Default::default() }`); the
/// trade-off is that new fields tracking the C++ library are
/// semver-major additions.
#[derive(Clone, Debug)]
pub struct Configuration {
    pub spi_speed_hz: i32,

    /// All attitude data will be transformed by this mounting angle.
    pub mounting_deg: Euler,

    /// Only a fixed set of rates are achievable.  Valid values are
    /// 100, 200, 400, 1000.  Selecting a higher rate than you need to
    /// sample at will result in more noise.
    pub attitude_rate_hz: u32,

    /// RF communication will be with a transmitter having this ID.
    pub rf_id: u32,

    pub enable_aux: bool,

    pub can: [CanConfiguration; 5],

    /// If true, nothing is guaranteed to work but `Pi3Hat::read_spi`.
    pub raw_spi_only: bool,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            spi_speed_hz: 10000000,
            mounting_deg: Euler::default(),
            attitude_rate_hz: 400,
            rf_id: 5678,
            enable_aux: true,
            can: [CanConfiguration::default(); 5],
            raw_spi_only: false,
        }
    }
}

/// Input to a single [`Pi3Hat::cycle`] operation.
///
/// Deliberately exhaustive; see [`Configuration`] for the semver
/// trade-off.
pub struct Input<'a> {
    pub tx_can: &'a [CanFrame],
    pub tx_rf: &'a [RfSlot],

    /// When waiting for CAN replies, the call will return all data
    /// that is available (possibly more than was requested).  If no
    /// data is available, and the given timeout has been exceeded,
    /// then it will return anyway.
    ///
    /// The timeout is measured from when the reading phase begins.
    /// This is not super precise, as the writing process has various
    /// queues, so it will need to encompass some amount of the time
    /// spent writing as well.
    pub timeout_ns: u32,

    /// When waiting for CAN replies, guarantee to wait at least this
    /// many nanoseconds after the final transmission is sent over SPI
    /// (not necessarily over the CAN bus).
    pub min_tx_wait_ns: u32,

    /// In addition to the absolute min_tx_wait_ns, there is a
    /// parallel calculation that attempts to estimate the total delay
    /// path for command-response pairs to set a minimum delay.  This
    /// value controls the non-calculated part of that estimate.  It
    /// should be the worst case response latency for a single device.
    ///
    /// Note, by default this is much longer than a device should
    /// actually take, so as to handle a wide range of possible host
    /// configurations.  i.e. including non-isolcpus or non-chrt
    /// operation.  If you have a properly configured system running
    /// on an isolcpu, this can potentially be as small as 200us.
    pub rx_baseline_wait_ns: u32,

    /// After each successful receipt, wait this much longer for more.
    pub rx_extra_wait_ns: u32,

    pub request_attitude: bool,

    /// If true, then the bias and uncertainty information will be
    /// filled in.  This increases the overall cycle time.
    pub request_attitude_detail: bool,

    /// If no new attitude is available, wait for it.
    pub wait_for_attitude: bool,

    pub request_rf: bool,

    /// A bitmask indicating CAN buses to check for data even if no
    /// replies are expected.
    ///  * bit 0 is unused, so the used bits start from 1
    pub force_can_check: u32,

    // These are data to store results in.
    pub rx_can: &'a mut [CanFrame],
    pub rx_rf: &'a mut [RfSlot],
    /// Where to store the attitude sample.  When this is `None`, no
    /// IMU SPI traffic occurs at all even if `request_attitude` (or
    /// `wait_for_attitude`) is set, so it cannot be used purely as a
    /// cycle pacing mechanism.
    pub attitude: Option<&'a mut Attitude>,
}

impl<'a> Default for Input<'a> {
    fn default() -> Self {
        Self {
            tx_can: &[],
            tx_rf: &[],
            timeout_ns: 0,
            min_tx_wait_ns: 200000,
            rx_baseline_wait_ns: 1000000,
            rx_extra_wait_ns: 0,
            request_attitude: false,
            request_attitude_detail: false,
            wait_for_attitude: false,
            request_rf: false,
            force_can_check: 0,
            rx_can: &mut [],
            rx_rf: &mut [],
            attitude: None,
        }
    }
}

/// Result of a single [`Pi3Hat::cycle`] operation.
#[derive(Clone, Copy, Debug, Default)]
#[non_exhaustive]
pub struct Output {
    pub attitude_present: bool,
    pub rx_can_size: usize,
    pub rx_rf_size: usize,

    /// This will only be updated if `Input::request_rf` is true.
    pub rf_lock_age_ms: u32,
}

#[derive(Clone, Copy, Debug, Default)]
#[non_exhaustive]
pub struct ProcessorInfo {
    pub git_hash: [u8; 20],
    pub dirty: bool,
    pub serial_number: [u8; 12],
}

#[derive(Clone, Copy, Debug, Default)]
#[non_exhaustive]
pub struct DeviceInfo {
    pub can1: ProcessorInfo,
    pub can2: ProcessorInfo,
    pub aux: ProcessorInfo,

    /// Is it safe to send frames that may never be acknowledged?
    /// Some older firmware versions became effectively "frozen" if a
    /// CAN frame was sent that was never acknowledged, either because
    /// no device whatsoever was connected to the bus or because
    /// nothing acknowledged the given ID.
    pub can_unknown_address_safe: bool,
}

#[derive(Clone, Copy, Debug, Default)]
#[non_exhaustive]
pub struct PerformanceInfo {
    pub cycles_per_ms: u32,
    pub min_cycles_per_ms: u32,
}

#[derive(Clone, Copy, Debug, Default)]
#[non_exhaustive]
pub struct DevicePerformance {
    pub can1: PerformanceInfo,
    pub can2: PerformanceInfo,
    pub aux: PerformanceInfo,
}

///////////////////////////////////////////////
// Random utility functions

fn round_up_dlc(value: usize) -> usize {
    match value {
        0..=8 => value,
        9..=12 => 12,
        13..=16 => 16,
        17..=20 => 20,
        21..=24 => 24,
        25..=32 => 32,
        33..=48 => 48,
        49..=64 => 64,
        _ => 0,
    }
}

// The casts are required on 32-bit targets, where time_t and c_long
// are narrower than i64.
#[allow(clippy::unnecessary_cast)]
fn get_now() -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC_RAW, &mut ts);
    }
    (ts.tv_sec as i64) * 1_000_000_000 + (ts.tv_nsec as i64)
}

/// A full barrier on either side of our waits, to ensure that setup
/// and hold times are properly enforced.  Allowing data stores and
/// loads to be re-ordered around the wait would defeat their purpose.
#[inline(always)]
fn barrier() {
    #[cfg(target_arch = "aarch64")]
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::asm!("dsb", options(nostack, preserves_flags));
    }
    // The C++ library refuses to compile on non-ARM architectures.
    // We instead fall back to a compiler/CPU fence so that the crate
    // can at least be compiled and unit tested on development hosts.
    // The hardware will never be detected there.
    #[cfg(not(any(target_arch = "aarch64", target_arch = "arm")))]
    std::sync::atomic::fence(std::sync::atomic::Ordering::SeqCst);
}

fn busy_wait_us(us: i64) {
    barrier();

    let start = get_now();
    let end = start + us * 1000;
    while get_now() <= end {}

    barrier();
}

fn read_contents(filename: &str) -> String {
    std::fs::read_to_string(filename).unwrap_or_default()
}

fn parse_rpi_version(model: &str) -> Result<i32> {
    for prefix in ["Raspberry Pi Compute Module ", "Raspberry Pi "] {
        if let Some(rest) = model.strip_prefix(prefix) {
            let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
            if !digits.is_empty() {
                return digits
                    .parse::<i32>()
                    .map_err(|_| Error::message(format!("Unknown model: {}", model)));
            }
        }
    }
    Err(Error::message(format!(
        "Unable to parse rpi version: {}",
        model
    )))
}

fn host_get_peripheral_address() -> Result<u64> {
    let model = std::fs::read_to_string("/proc/device-tree/model")
        .map_err(|_| Error::message("Unable to read /proc/device-tree/model"))?;

    let version = parse_rpi_version(&model)?;

    // Return hard-coded values for known versions.
    match version.cmp(&4) {
        std::cmp::Ordering::Equal => Ok(0xfe000000),
        // Pi 3, 2, 1, Zero, Model A/B
        std::cmp::Ordering::Less => Ok(0x3f000000),
        std::cmp::Ordering::Greater => Err(Error::message(format!(
            "Unsupported Raspberry Pi: {}",
            model
        ))),
    }
}

/// Returns true if a pi3hat is attached, as determined from the hat
/// EEPROM data exposed through the device tree.
pub fn pi3hat_present() -> bool {
    let product_code = read_contents("/sys/firmware/devicetree/base/hat/product");
    product_code.starts_with("mjbots quad pi3 hat")
}

///////////////////////////////////////////////
// Random utility classes

/// Manages ownership of an mmap'ed region of a given file descriptor.
struct SystemMmap {
    ptr: *mut libc::c_void,
    size: usize,
}

impl SystemMmap {
    fn new(fd: i32, size: usize, offset: u64) -> Result<Self> {
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                offset as libc::off_t,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err(Error::errno("pi3hat: mmap failed"));
        }
        Ok(Self { ptr, size })
    }

    fn ptr(&self) -> *mut libc::c_void {
        self.ptr
    }
}

impl Drop for SystemMmap {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.ptr, self.size);
        }
    }
}

// The mmap'ed pointers refer to MMIO regions.  Moving them between
// threads is safe; concurrent use is prevented by requiring `&mut
// Pi3Hat` for all operations with side effects on shared state.
unsafe impl Send for SystemMmap {}

#[inline(always)]
fn reg_read(base: *mut u32, index: usize) -> u32 {
    unsafe { base.add(index).read_volatile() }
}

#[inline(always)]
fn reg_write(base: *mut u32, index: usize, value: u32) {
    unsafe { base.add(index).write_volatile(value) }
}

///////////////////////////////////////////////
// Drivers for the Raspberry Pi hardware

const GPIO_BASE: u64 = 0x00200000;

const GPIO_OUTPUT: u32 = 1;
const GPIO_ALT_0: u32 = 4;
const GPIO_ALT_4: u32 = 3;

struct Rpi3Gpio {
    _mmap: SystemMmap,
    gpio: *mut u32,
}

unsafe impl Send for Rpi3Gpio {}

impl Rpi3Gpio {
    fn new(dev_mem_fd: i32) -> Result<Self> {
        let mmap = SystemMmap::new(dev_mem_fd, 4096, host_get_peripheral_address()? + GPIO_BASE)?;
        let gpio = mmap.ptr() as *mut u32;
        Ok(Self { _mmap: mmap, gpio })
    }

    fn set_gpio_mode(&self, gpio: u32, function: u32) {
        let reg_offset = (gpio / 10) as usize;
        let bit = (gpio % 10) * 3;
        let value = reg_read(self.gpio, reg_offset);
        reg_write(
            self.gpio,
            reg_offset,
            (value & !(0x7 << bit)) | ((function & 0x7) << bit),
        );
    }

    fn set_gpio_output(&self, gpio: u32, value: bool) {
        if value {
            let reg_offset = (gpio / 32 + 7) as usize;
            reg_write(self.gpio, reg_offset, 1 << (gpio % 32));
        } else {
            let reg_offset = (gpio / 32 + 10) as usize;
            reg_write(self.gpio, reg_offset, 1 << (gpio % 32));
        }
    }
}

/// Drives the given GPIO low for as long as this value is alive.
struct ActiveLow<'a> {
    parent: &'a Rpi3Gpio,
    gpio: u32,
}

impl<'a> ActiveLow<'a> {
    fn new(parent: &'a Rpi3Gpio, gpio: u32) -> Self {
        parent.set_gpio_output(gpio, false);
        Self { parent, gpio }
    }
}

impl Drop for ActiveLow<'_> {
    fn drop(&mut self) {
        self.parent.set_gpio_output(self.gpio, true);
    }
}

const SPI0_CS: [u32; 2] = [8, 7];

const SPI_BASE: u64 = 0x204000;
const SPI_CS_TA: u32 = 1 << 7;
const SPI_CS_DONE: u32 = 1 << 16;
const SPI_CS_RXD: u32 = 1 << 17;
const SPI_CS_TXD: u32 = 1 << 18;

// Register indices (of u32 words) within the SPI0 peripheral.
const SPI_REG_CS: usize = 0;
const SPI_REG_FIFO: usize = 1;
const SPI_REG_CLK: usize = 2;

#[derive(Clone, Copy, Debug)]
struct SpiOptions {
    speed_hz: i32,
    cs_hold_us: i64,
    address_hold_us: i64,
}

impl Default for SpiOptions {
    fn default() -> Self {
        Self {
            speed_hz: 10000000,
            cs_hold_us: 3,
            address_hold_us: 3,
        }
    }
}

/// This struct interacts with the SPI0 device on a raspberry pi using
/// the BCM2835/6/7's registers directly.  The kernel driver must not
/// be active (it can be loaded, as long as you're not using it), and
/// this must be run as root or otherwise have access to `/dev/mem`.
struct PrimarySpi {
    options: SpiOptions,
    _file: File,
    _spi_mmap: SystemMmap,
    spi: *mut u32,
    gpio: Rpi3Gpio,
}

unsafe impl Send for PrimarySpi {}

impl PrimarySpi {
    fn new(options: SpiOptions) -> Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_SYNC)
            .open("/dev/mem")
            .map_err(|e| Error::os("pi3hat: could not open /dev/mem", e))?;
        let fd = file.as_raw_fd();

        let spi_mmap = SystemMmap::new(fd, 4096, host_get_peripheral_address()? + SPI_BASE)?;
        let spi = spi_mmap.ptr() as *mut u32;

        let gpio = Rpi3Gpio::new(fd)?;

        gpio.set_gpio_output(SPI0_CS[0], true);
        gpio.set_gpio_output(SPI0_CS[1], true);

        gpio.set_gpio_mode(SPI0_CS[0], GPIO_OUTPUT); // We'll do CS in SW
        gpio.set_gpio_mode(SPI0_CS[1], GPIO_OUTPUT);
        gpio.set_gpio_mode(9, GPIO_ALT_0);
        gpio.set_gpio_mode(10, GPIO_ALT_0);
        gpio.set_gpio_mode(11, GPIO_ALT_0);

        reg_write(
            spi,
            SPI_REG_CS,
            0
                | (0 << 25) // LEn_LONG
                | (0 << 24) // DMA_LEN
                | (0 << 23) // CSPOL2
                | (0 << 22) // CSPOL1
                | (0 << 21) // CSPOL0
                | (0 << 13) // LEN
                | (0 << 12) // REN
                | (0 << 11) // ADCS
                | (0 << 10) // INTR
                | (0 << 9) // INTD
                | (0 << 8) // DMAEN
                | (0 << 7) // TA
                | (0 << 6) // CSPOL
                | (0 << 4) // CLEAR
                | (0 << 3) // CPOL
                | (0 << 2) // CPHA
                | (0 << 0), // CS
        );

        // Configure the SPI peripheral.
        let clkdiv = (400000000 / options.speed_hz).clamp(0, 65535) as u32;
        reg_write(spi, SPI_REG_CLK, clkdiv);

        Ok(Self {
            options,
            _file: file,
            _spi_mmap: spi_mmap,
            spi,
            gpio,
        })
    }

    fn write(&self, cs: usize, address: u8, data: &[u8]) {
        busy_wait_us(self.options.cs_hold_us);
        let _cs_holder = ActiveLow::new(&self.gpio, SPI0_CS[cs]);
        busy_wait_us(self.options.cs_hold_us);

        // TA, and clear FIFOs.
        reg_write(
            self.spi,
            SPI_REG_CS,
            reg_read(self.spi, SPI_REG_CS) | SPI_CS_TA | (3 << 4),
        );

        reg_write(self.spi, SPI_REG_FIFO, address as u32);

        // We are done when we have received one byte back.
        while reg_read(self.spi, SPI_REG_CS) & SPI_CS_RXD == 0 {}
        let _ = reg_read(self.spi, SPI_REG_FIFO);

        if !data.is_empty() {
            // Wait our address hold time.
            busy_wait_us(self.options.address_hold_us);

            let mut offset = 0;
            while offset < data.len() {
                while reg_read(self.spi, SPI_REG_CS) & SPI_CS_TXD == 0 {}
                reg_write(self.spi, SPI_REG_FIFO, data[offset] as u32);
                offset += 1;
            }

            // Wait until we are no longer busy.
            while reg_read(self.spi, SPI_REG_CS) & SPI_CS_DONE == 0 {
                if reg_read(self.spi, SPI_REG_CS) & SPI_CS_RXD != 0 {
                    let _ = reg_read(self.spi, SPI_REG_FIFO);
                }
            }
        }

        reg_write(
            self.spi,
            SPI_REG_CS,
            reg_read(self.spi, SPI_REG_CS) & !SPI_CS_TA,
        );
    }

    fn read(&self, cs: usize, address: u8, data: &mut [u8]) {
        busy_wait_us(self.options.cs_hold_us);
        let _cs_holder = ActiveLow::new(&self.gpio, SPI0_CS[cs]);
        busy_wait_us(self.options.cs_hold_us);

        // TA, and clear FIFOs.
        reg_write(
            self.spi,
            SPI_REG_CS,
            reg_read(self.spi, SPI_REG_CS) | SPI_CS_TA | (3 << 4),
        );

        reg_write(self.spi, SPI_REG_FIFO, address as u32);

        // We are done when we have received one byte back.
        while reg_read(self.spi, SPI_REG_CS) & SPI_CS_RXD == 0 {}
        let _ = reg_read(self.spi, SPI_REG_FIFO);

        if !data.is_empty() {
            // Wait our address hold time.
            busy_wait_us(self.options.address_hold_us);

            // Now we write out dummy values, reading values in.
            let mut remaining_read = data.len();
            let mut remaining_write = remaining_read;
            let mut offset = 0;
            while remaining_read > 0 {
                // Make sure we don't write more than we have read
                // spots remaining so that we can never overflow the
                // RX fifo.
                let can_write = (remaining_read - remaining_write) < 16;
                if can_write
                    && remaining_write > 0
                    && reg_read(self.spi, SPI_REG_CS) & SPI_CS_TXD != 0
                {
                    reg_write(self.spi, SPI_REG_FIFO, 0);
                    remaining_write -= 1;
                }

                if reg_read(self.spi, SPI_REG_CS) & SPI_CS_RXD != 0 {
                    data[offset] = (reg_read(self.spi, SPI_REG_FIFO) & 0xff) as u8;
                    offset += 1;
                    remaining_read -= 1;
                }
            }
        }

        reg_write(
            self.spi,
            SPI_REG_CS,
            reg_read(self.spi, SPI_REG_CS) & !SPI_CS_TA,
        );
    }
}

const AUX_BASE: u64 = 0x00215000;
const SPI1_CS: [u32; 3] = [18, 17, 16];

const AUXSPI_STAT_TX_FULL: u32 = 1 << 10;
const AUXSPI_STAT_TX_EMPTY: u32 = 1 << 9;
const AUXSPI_STAT_RX_EMPTY: u32 = 1 << 7;
const AUXSPI_STAT_BUSY: u32 = 1 << 6;

// Register indices (of u32 words) within the AUX SPI1 peripheral,
// which itself starts at byte offset 0x80 within the AUX block.
const AUXSPI_REG_CNTL0: usize = 0;
const AUXSPI_REG_CNTL1: usize = 1;
const AUXSPI_REG_STAT: usize = 2;
const AUXSPI_REG_IO: usize = 8;
const AUXSPI_REG_TXHOLD: usize = 12;

/// This struct interacts with the AUX SPI1 device on a raspberry pi
/// using the BCM2835/6/7's registers directly.  The kernel driver
/// must not be active, and this must be run as root or otherwise have
/// access to `/dev/mem`.
struct AuxSpi {
    options: SpiOptions,
    _file: File,
    _spi_mmap: SystemMmap,
    spi: *mut u32,
    gpio: Rpi3Gpio,
}

unsafe impl Send for AuxSpi {}

const AUXSPI_PACK: usize = 3;

impl AuxSpi {
    fn new(options: SpiOptions) -> Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_SYNC)
            .open("/dev/mem")
            .map_err(|e| Error::os("rpi3_aux_spi: could not open /dev/mem", e))?;
        let fd = file.as_raw_fd();

        let spi_mmap = SystemMmap::new(fd, 4096, host_get_peripheral_address()? + AUX_BASE)?;
        let auxenb = unsafe { (spi_mmap.ptr() as *mut u8).add(0x04) } as *mut u32;
        let spi = unsafe { (spi_mmap.ptr() as *mut u8).add(0x80) } as *mut u32;

        let gpio = Rpi3Gpio::new(fd)?;

        gpio.set_gpio_output(SPI1_CS[0], true);
        gpio.set_gpio_output(SPI1_CS[1], true);
        gpio.set_gpio_output(SPI1_CS[2], true);

        gpio.set_gpio_mode(SPI1_CS[0], GPIO_OUTPUT); // We'll do CS in SW
        gpio.set_gpio_mode(SPI1_CS[1], GPIO_OUTPUT);
        gpio.set_gpio_mode(SPI1_CS[2], GPIO_OUTPUT);
        gpio.set_gpio_mode(19, GPIO_ALT_4);
        gpio.set_gpio_mode(20, GPIO_ALT_4);
        gpio.set_gpio_mode(21, GPIO_ALT_4);

        // Start by disabling it to try and get to a known good state.
        reg_write(auxenb, 0, reg_read(auxenb, 0) & !0x02);

        busy_wait_us(10);

        // Enable the SPI peripheral.
        reg_write(auxenb, 0, reg_read(auxenb, 0) | 0x02); // SPI1 enable

        reg_write(spi, AUXSPI_REG_CNTL1, 0);
        reg_write(spi, AUXSPI_REG_CNTL0, 1 << 9); // clear fifos

        // Configure the SPI peripheral.
        let clkdiv = (250000000 / 2 / options.speed_hz - 1).clamp(0, 4095) as u32;
        reg_write(
            spi,
            AUXSPI_REG_CNTL0,
            (clkdiv << 20)
                | (0 << 17) // chip select defaults
                | (0 << 16) // post-input mode
                | (0 << 15) // variable CS
                | (1 << 14) // variable width
                | (2 << 12) // DOUT hold time
                | (1 << 11) // enable
                | (1 << 10) // in rising?
                | (0 << 9) // clear fifos
                | (0 << 8) // out rising
                | (0 << 7) // invert SPI CLK
                | (1 << 6) // MSB first
                | (0 << 0), // shift length
        );

        reg_write(
            spi,
            AUXSPI_REG_CNTL1,
            (0 << 8) // CS high time
                | (0 << 7) // tx empty IRQ
                | (0 << 6) // done IRQ
                | (1 << 1) // shift in MS first
                | (0 << 0), // keep input
        );

        Ok(Self {
            options,
            _file: file,
            _spi_mmap: spi_mmap,
            spi,
            gpio,
        })
    }

    fn write(&self, cs: usize, address: u8, data: &[u8]) {
        busy_wait_us(self.options.cs_hold_us);
        let _cs_holder = ActiveLow::new(&self.gpio, SPI1_CS[cs]);
        busy_wait_us(self.options.cs_hold_us);

        let value: u32 = 0
            | (0 << 29) // CS
            | (8 << 24) // data width
            | ((address as u32) << 16); // data

        if !data.is_empty() {
            reg_write(self.spi, AUXSPI_REG_TXHOLD, value);
        } else {
            reg_write(self.spi, AUXSPI_REG_IO, value);
        }

        while reg_read(self.spi, AUXSPI_REG_STAT) & AUXSPI_STAT_TX_EMPTY == 0 {}

        if data.is_empty() {
            return;
        }

        // Wait our address hold time.
        busy_wait_us(self.options.address_hold_us);

        let mut offset = 0;
        while offset < data.len() {
            while reg_read(self.spi, AUXSPI_REG_STAT) & AUXSPI_STAT_TX_FULL != 0 {}

            let remaining = data.len() - offset;
            let to_write = remaining.min(AUXSPI_PACK);

            // The Auxiliary SPI controller inserts a small dead time
            // between each FIFO entry, even if the FIFO is all full
            // up.  Thus, we work to minimize this by using all 3
            // available bytes of each FIFO entry when possible.
            let payload: u32 = match to_write {
                1 => (data[offset] as u32) << 16,
                2 => ((data[offset] as u32) << 16) | ((data[offset + 1] as u32) << 8),
                3 => {
                    ((data[offset] as u32) << 16)
                        | ((data[offset + 1] as u32) << 8)
                        | (data[offset + 2] as u32)
                }
                _ => unreachable!(),
            };
            let data_value: u32 = 0
                | (0 << 29) // CS
                | (((to_write as u32) * 8) << 24) // data width
                | payload; // data

            if offset + to_write == data.len() {
                reg_write(self.spi, AUXSPI_REG_IO, data_value);
            } else {
                reg_write(self.spi, AUXSPI_REG_TXHOLD, data_value);
            }
            offset += to_write;
        }

        // Discard anything in the RX fifo.
        while reg_read(self.spi, AUXSPI_REG_STAT) & AUXSPI_STAT_RX_EMPTY == 0 {
            let _ = reg_read(self.spi, AUXSPI_REG_IO);
        }

        // Wait until we are no longer busy.
        while reg_read(self.spi, AUXSPI_REG_STAT) & AUXSPI_STAT_BUSY != 0 {}
    }

    fn read(&self, cs: usize, address: u8, data: &mut [u8]) {
        busy_wait_us(self.options.cs_hold_us);
        let _cs_holder = ActiveLow::new(&self.gpio, SPI1_CS[cs]);
        busy_wait_us(self.options.cs_hold_us);

        let value: u32 = 0
            | (0 << 29) // CS
            | (8 << 24) // data width
            | ((address as u32) << 16); // data

        if !data.is_empty() {
            reg_write(self.spi, AUXSPI_REG_TXHOLD, value);
        } else {
            reg_write(self.spi, AUXSPI_REG_IO, value);
        }

        loop {
            let stat = reg_read(self.spi, AUXSPI_REG_STAT);
            if stat & AUXSPI_STAT_BUSY == 0 && stat & AUXSPI_STAT_TX_EMPTY != 0 {
                break;
            }
        }

        if data.is_empty() {
            return;
        }

        // Wait our address hold time.
        busy_wait_us(self.options.address_hold_us);

        // Discard the rx fifo.
        while reg_read(self.spi, AUXSPI_REG_STAT) & AUXSPI_STAT_RX_EMPTY == 0 {
            let _ = reg_read(self.spi, AUXSPI_REG_IO);
        }

        // Now we write out dummy values, reading values in.
        let mut remaining_read = data.len();
        let mut remaining_write = remaining_read;
        let mut offset = 0;
        while remaining_read > 0 {
            // Make sure we don't write more than we have read spots
            // remaining so that we can never overflow the RX fifo.
            let can_write = (remaining_read - remaining_write) < 3 * AUXSPI_PACK;
            let cur_stat = reg_read(self.spi, AUXSPI_REG_STAT);
            let tx_full = cur_stat & AUXSPI_STAT_TX_FULL != 0;

            if can_write && remaining_write > 0 && !tx_full {
                let to_read = remaining_write.min(AUXSPI_PACK);
                let to_write: u32 = 0
                    | (0 << 29) // CS
                    | ((8 * to_read as u32) << 24) // data width
                    | 0; // data
                remaining_write -= to_read;
                if remaining_write == 0 {
                    reg_write(self.spi, AUXSPI_REG_IO, to_write);
                } else {
                    reg_write(self.spi, AUXSPI_REG_TXHOLD, to_write);
                }
            }

            if reg_read(self.spi, AUXSPI_REG_STAT) & AUXSPI_STAT_RX_EMPTY == 0 {
                let value = reg_read(self.spi, AUXSPI_REG_IO);

                let byte_count = remaining_read.min(AUXSPI_PACK);
                match byte_count {
                    3 => {
                        data[offset] = ((value >> 16) & 0xff) as u8;
                        data[offset + 1] = ((value >> 8) & 0xff) as u8;
                        data[offset + 2] = (value & 0xff) as u8;
                    }
                    2 => {
                        data[offset] = ((value >> 8) & 0xff) as u8;
                        data[offset + 1] = (value & 0xff) as u8;
                    }
                    1 => {
                        data[offset] = (value & 0xff) as u8;
                    }
                    _ => unreachable!(),
                }
                offset += byte_count;
                remaining_read -= byte_count;
            }
        }
    }
}

/// The common interface between [`PrimarySpi`] and [`AuxSpi`].
trait Spi {
    fn write(&self, cs: usize, address: u8, data: &[u8]);
    fn read(&self, cs: usize, address: u8, data: &mut [u8]);
}

impl Spi for PrimarySpi {
    fn write(&self, cs: usize, address: u8, data: &[u8]) {
        PrimarySpi::write(self, cs, address, data)
    }

    fn read(&self, cs: usize, address: u8, data: &mut [u8]) {
        PrimarySpi::read(self, cs, address, data)
    }
}

impl Spi for AuxSpi {
    fn write(&self, cs: usize, address: u8, data: &[u8]) {
        AuxSpi::write(self, cs, address, data)
    }

    fn read(&self, cs: usize, address: u8, data: &mut [u8]) {
        AuxSpi::read(self, cs, address, data)
    }
}

///////////////////////////////////////////////
// Structures exchanged with the pi3hat over SPI
//
// The C++ library reads and writes these as packed in-memory
// structures.  Here we serialize them explicitly; the wire format is
// little endian, which is the native byte order on all supported
// hosts.

fn f32_at(buf: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(buf[offset..offset + 4].try_into().unwrap())
}

fn u32_at(buf: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(buf[offset..offset + 4].try_into().unwrap())
}

/// This is the format exported by register 34 on the hat.  86 bytes,
/// of which the first 42 contain everything except bias and
/// uncertainty information.
const ATTITUDE_DETAIL_SIZE: usize = 86;
const ATTITUDE_BASIC_SIZE: usize = 42;

/// `DeviceImuConfiguration` in the C++ library: roll, pitch, and yaw
/// in degrees as f32, followed by the rate in Hz as u32.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct DeviceImuConfiguration {
    roll_deg: f32,
    pitch_deg: f32,
    yaw_deg: f32,
    rate_hz: u32,
}

impl DeviceImuConfiguration {
    const SIZE: usize = 16;

    fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut result = [0u8; Self::SIZE];
        result[0..4].copy_from_slice(&self.roll_deg.to_le_bytes());
        result[4..8].copy_from_slice(&self.pitch_deg.to_le_bytes());
        result[8..12].copy_from_slice(&self.yaw_deg.to_le_bytes());
        result[12..16].copy_from_slice(&self.rate_hz.to_le_bytes());
        result
    }

    fn from_bytes(buf: &[u8; Self::SIZE]) -> Self {
        Self {
            roll_deg: f32_at(buf, 0),
            pitch_deg: f32_at(buf, 4),
            yaw_deg: f32_at(buf, 8),
            rate_hz: u32_at(buf, 12),
        }
    }
}

/// `DeviceCanConfigurationV4` in the C++ library.  The V3 variant is
/// the first 21 bytes; V4 appends `cancel_all_ms`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct DeviceCanConfiguration {
    slow_bitrate: i32,
    fast_bitrate: i32,
    fdcan_frame: i8,
    bitrate_switch: i8,
    automatic_retransmission: i8,
    restricted_mode: i8,
    bus_monitor: i8,
    std_rate: [i8; 4],
    fd_rate: [i8; 4],
    cancel_all_ms: u32,
}

impl Default for DeviceCanConfiguration {
    fn default() -> Self {
        Self {
            slow_bitrate: 1000000,
            fast_bitrate: 5000000,
            fdcan_frame: 1,
            bitrate_switch: 1,
            automatic_retransmission: 0,
            restricted_mode: 0,
            bus_monitor: 0,
            std_rate: [-1; 4],
            fd_rate: [-1; 4],
            cancel_all_ms: 50,
        }
    }
}

impl DeviceCanConfiguration {
    const V3_SIZE: usize = 21;
    const V4_SIZE: usize = 25;

    fn to_bytes(self) -> [u8; Self::V4_SIZE] {
        let mut result = [0u8; Self::V4_SIZE];
        result[0..4].copy_from_slice(&self.slow_bitrate.to_le_bytes());
        result[4..8].copy_from_slice(&self.fast_bitrate.to_le_bytes());
        result[8] = self.fdcan_frame as u8;
        result[9] = self.bitrate_switch as u8;
        result[10] = self.automatic_retransmission as u8;
        result[11] = self.restricted_mode as u8;
        result[12] = self.bus_monitor as u8;
        for i in 0..4 {
            result[13 + i] = self.std_rate[i] as u8;
            result[17 + i] = self.fd_rate[i] as u8;
        }
        result[21..25].copy_from_slice(&self.cancel_all_ms.to_le_bytes());
        result
    }

    fn from_bytes(buf: &[u8; Self::V4_SIZE]) -> Self {
        let mut std_rate = [0i8; 4];
        let mut fd_rate = [0i8; 4];
        for i in 0..4 {
            std_rate[i] = buf[13 + i] as i8;
            fd_rate[i] = buf[17 + i] as i8;
        }
        Self {
            slow_bitrate: u32_at(buf, 0) as i32,
            fast_bitrate: u32_at(buf, 4) as i32,
            fdcan_frame: buf[8] as i8,
            bitrate_switch: buf[9] as i8,
            automatic_retransmission: buf[10] as i8,
            restricted_mode: buf[11] as i8,
            bus_monitor: buf[12] as i8,
            std_rate,
            fd_rate,
            cancel_all_ms: u32_at(buf, 21),
        }
    }

    fn eq_v3(&self, rhs: &Self) -> bool {
        self.to_bytes()[..Self::V3_SIZE] == rhs.to_bytes()[..Self::V3_SIZE]
    }
}

fn get_processor_info(spi: &dyn Spi, cs: usize) -> ProcessorInfo {
    // DeviceDeviceInfo: git_hash[20], dirty u8, serial_number[12].
    let mut buf = [0u8; 33];
    spi.read(cs, 97, &mut buf);

    let mut result = ProcessorInfo::default();
    result.git_hash.copy_from_slice(&buf[0..20]);
    result.dirty = buf[20] != 0;
    result.serial_number.copy_from_slice(&buf[21..33]);
    result
}

fn get_performance(spi: &dyn Spi, cs: usize) -> PerformanceInfo {
    let mut buf = [0u8; 8];
    spi.read(cs, 100, &mut buf);

    PerformanceInfo {
        cycles_per_ms: u32_at(&buf, 0),
        min_cycles_per_ms: u32_at(&buf, 4),
    }
}

fn read_byte(spi: &dyn Spi, cs: usize, address: u8) -> u8 {
    let mut data = [0u8; 1];
    spi.read(cs, address, &mut data);
    data[0]
}

///////////////////////////////////////////////
// The main driver

/// Encode a CAN frame in the SPI format used by the pi3hat
/// processors.  Returns the SPI address and the encoded length within
/// `buf`.
fn encode_can_packet(cpu_bus: i32, can_frame: &CanFrame, buf: &mut [u8; 70]) -> (u8, usize) {
    let size = round_up_dlc(can_frame.size as usize);
    let frame_size = can_frame.size as usize;

    buf[0] = (if cpu_bus == 1 { 0x80u8 } else { 0x00u8 }) | (size as u8 & 0x7f);

    if can_frame.id <= 0xffff {
        // We'll use the 2 byte ID formulation, cmd 5
        buf[1] = ((can_frame.id >> 8) & 0xff) as u8;
        buf[2] = (can_frame.id & 0xff) as u8;
        buf[3..3 + frame_size].copy_from_slice(&can_frame.data[..frame_size]);
        for item in buf.iter_mut().take(3 + size).skip(3 + frame_size) {
            *item = 0x50;
        }
        (5, 3 + size)
    } else {
        // 4 byte formulation, cmd 4
        buf[1] = ((can_frame.id >> 24) & 0xff) as u8;
        buf[2] = ((can_frame.id >> 16) & 0xff) as u8;
        buf[3] = ((can_frame.id >> 8) & 0xff) as u8;
        buf[4] = (can_frame.id & 0xff) as u8;
        buf[5..5 + frame_size].copy_from_slice(&can_frame.data[..frame_size]);
        for item in buf.iter_mut().take(5 + size).skip(5 + frame_size) {
            *item = 0x50;
        }
        (4, 5 + size)
    }
}

#[derive(Default)]
struct ExpectedReply {
    count: [i32; 6],

    // How long we expect each bus to take to send the frames.
    send_ns: [i64; 6],

    // How long we expect each bus to take to receive any responses.
    receive_ns: [i64; 6],
}

/// Holds an advisory lock for as long as it is alive, so that at most
/// one process drives the pi3hat at a time.
struct LockFile {
    _file: File,
}

impl LockFile {
    fn new() -> Result<Self> {
        // Since we directly poke at /dev/mem, nothing good can come
        // of multiple instances of this running at once on the same
        // system.  Thus, we use a lock to ensure that at most one
        // copy runs at a time.
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .mode(0o660)
            .open("/tmp/.pi3hat-lock")
            .map_err(|e| Error::os("pi3hat: could not open lock file", e))?;

        let mut lock: libc::flock = unsafe { std::mem::zeroed() };
        lock.l_type = libc::F_WRLCK as libc::c_short;
        lock.l_whence = libc::SEEK_SET as libc::c_short;
        lock.l_start = 0;
        lock.l_len = 0;
        lock.l_pid = -1;

        let ret = unsafe { libc::fcntl(file.as_raw_fd(), libc::F_SETLK, &lock) };
        if ret < 0 {
            return Err(Error::errno(
                "pi3hat: could not acquire lock, is another process running?",
            ));
        }

        Ok(Self { _file: file })
    }
}

/// This provides the top level interface to an application which
/// wants to use all the features of the mjbots pi3hat in an
/// integrated fashion at high rate.
///
/// The primary operations are blocking and busy-loop the CPU.  For
/// best timing performance, the following steps should be taken:
///
///  * The CPU this runs on should be isolated from all other OS
///    operations.  This can be accomplished through the isolcpus
///    mechanism and sched_setaffinity.
///  * The thread this is running in should be set to realtime
///    priority (see [`crate::realtime::configure_realtime`]).
pub struct Pi3Hat {
    config: Configuration,

    _lock_file: LockFile,

    primary_spi: PrimarySpi,
    aux_spi: AuxSpi,

    // This is a member variable purely so that in steady state we
    // don't have to allocate memory.
    //
    // It is 1 indexed to match the bus naming.
    can_packets: [Vec<usize>; 6],

    // To keep track of which RF slots we have processed.
    last_bitfield: u32,
}

impl Pi3Hat {
    /// Construct a driver for an attached pi3hat.
    ///
    /// This requires access to `/dev/mem` (i.e. root), and will fail
    /// if no pi3hat is attached or another process is using it.
    pub fn new(configuration: &Configuration) -> Result<Self> {
        // First, look to see if we have a pi3hat attached by looking
        // for the eeprom data.  This prevents us from stomping on the
        // SPI registers if it isn't ours.  (The C++ library performs
        // this check after initializing the SPI peripherals; here we
        // can do it first.)
        if !pi3hat_present() {
            return Err(Error::message("No pi3hat detected"));
        }

        // Also deliberately before any hardware setup: a second
        // process then fails cleanly without having perturbed the
        // registers of the one holding the lock.  (The C++ library
        // acquires the lock only after its SPI construction has
        // already reconfigured GPIO and SPI state.)
        let lock_file = LockFile::new()?;

        let spi_options = SpiOptions {
            speed_hz: configuration.spi_speed_hz,
            ..SpiOptions::default()
        };
        let primary_spi = PrimarySpi::new(spi_options)?;
        let aux_spi = AuxSpi::new(spi_options)?;

        let mut result = Self {
            config: configuration.clone(),
            _lock_file: lock_file,
            primary_spi,
            aux_spi,
            can_packets: Default::default(),
            last_bitfield: 0,
        };

        if result.config.raw_spi_only {
            return Ok(result);
        }

        // Verify the versions of all peripherals we will use.
        result.verify_versions()?;

        if result.config.enable_aux {
            result.configure_aux()?;
        }
        result.configure_can()?;

        Ok(result)
    }

    fn configure_aux(&mut self) -> Result<()> {
        // See if we need to update the IMU configuration.
        let mut buf = [0u8; DeviceImuConfiguration::SIZE];
        self.primary_spi.read(0, 35, &mut buf);
        let original_imu_configuration = DeviceImuConfiguration::from_bytes(&buf);

        let desired_imu = DeviceImuConfiguration {
            yaw_deg: self.config.mounting_deg.yaw as f32,
            pitch_deg: self.config.mounting_deg.pitch as f32,
            roll_deg: self.config.mounting_deg.roll as f32,
            rate_hz: self.config.attitude_rate_hz.min(1000),
        };

        if desired_imu != original_imu_configuration {
            self.primary_spi.write(0, 36, &desired_imu.to_bytes());

            // Give it some time to work.
            std::thread::sleep(std::time::Duration::from_micros(1000));
            let mut verify_buf = [0u8; DeviceImuConfiguration::SIZE];
            self.primary_spi.read(0, 35, &mut verify_buf);
            let config_verify = DeviceImuConfiguration::from_bytes(&verify_buf);
            if desired_imu != config_verify {
                return Err(Error::message(format!(
                    "IMU config not set properly ({},{},{}) {} != ({},{},{}) {}",
                    desired_imu.yaw_deg,
                    desired_imu.pitch_deg,
                    desired_imu.roll_deg,
                    desired_imu.rate_hz,
                    config_verify.yaw_deg,
                    config_verify.pitch_deg,
                    config_verify.roll_deg,
                    config_verify.rate_hz
                )));
            }
        }

        // Configure our RF id if necessary.
        let mut id_buf = [0u8; 4];
        self.primary_spi.read(0, 49, &mut id_buf);
        let original_id = u32_at(&id_buf, 0);
        if original_id != self.config.rf_id {
            self.primary_spi
                .write(0, 50, &self.config.rf_id.to_le_bytes());
            // Changing the ID takes at least a few milliseconds.
            std::thread::sleep(std::time::Duration::from_micros(10000));
            let mut verify_buf = [0u8; 4];
            self.primary_spi.read(0, 49, &mut verify_buf);
            let id_verify = u32_at(&verify_buf, 0);
            if self.config.rf_id != id_verify {
                return Err(Error::message(format!(
                    "RF Id not set properly ({:08x} != {:08x})",
                    self.config.rf_id, id_verify
                )));
            }
        }

        Ok(())
    }

    fn update_can_config(
        spi: &dyn Spi,
        cs: usize,
        canbus: usize,
        can_config: &CanConfiguration,
    ) -> Result<()> {
        // If the version is before 3, then we can't config anything.
        let version = read_byte(spi, cs, 0);
        if version < 3 {
            return Ok(());
        }

        // Populate what we want our config to look like.
        let out = DeviceCanConfiguration {
            slow_bitrate: can_config.slow_bitrate,
            fast_bitrate: can_config.fast_bitrate,
            fdcan_frame: if can_config.fdcan_frame { 1 } else { 0 },
            bitrate_switch: if can_config.bitrate_switch { 1 } else { 0 },
            automatic_retransmission: if can_config.automatic_retransmission {
                1
            } else {
                0
            },
            bus_monitor: if can_config.bus_monitor { 1 } else { 0 },
            // NOTE: `restricted_mode` is intentionally not copied here;
            // it is left at its default, matching the C++ library.
            std_rate: [
                can_config.std_rate.prescaler as i8,
                can_config.std_rate.sync_jump_width as i8,
                can_config.std_rate.time_seg1 as i8,
                can_config.std_rate.time_seg2 as i8,
            ],
            fd_rate: [
                can_config.fd_rate.prescaler as i8,
                can_config.fd_rate.sync_jump_width as i8,
                can_config.fd_rate.time_seg1 as i8,
                can_config.fd_rate.time_seg2 as i8,
            ],
            cancel_all_ms: can_config.cancel_all_ms,
            ..DeviceCanConfiguration::default()
        };

        let spi_size = if version <= 3 {
            DeviceCanConfiguration::V3_SIZE
        } else {
            DeviceCanConfiguration::V4_SIZE
        };

        // Check to see if this is what is already there.
        let read_config = |spi: &dyn Spi| {
            let mut buf = [0u8; DeviceCanConfiguration::V4_SIZE];
            spi.read(cs, if canbus != 0 { 8 } else { 7 }, &mut buf[..spi_size]);
            DeviceCanConfiguration::from_bytes(&buf)
        };

        let original_config = read_config(spi);
        let matches = |a: &DeviceCanConfiguration, b: &DeviceCanConfiguration| {
            if version <= 3 {
                a.eq_v3(b)
            } else {
                a == b
            }
        };
        if matches(&original_config, &out) {
            // We have nothing to do, so just bail early.
            return Ok(());
        }

        // Update the configuration on the device.
        spi.write(
            cs,
            if canbus != 0 { 10 } else { 9 },
            &out.to_bytes()[..spi_size],
        );

        // Give it some time to work.
        std::thread::sleep(std::time::Duration::from_micros(100));

        let verify = read_config(spi);
        if !matches(&out, &verify) {
            return Err(Error::message("Could not set CAN configuration properly"));
        }

        Ok(())
    }

    fn configure_can(&mut self) -> Result<()> {
        Self::update_can_config(&self.aux_spi, 0, 0, &self.config.can[0])?;
        Self::update_can_config(&self.aux_spi, 0, 1, &self.config.can[1])?;
        Self::update_can_config(&self.aux_spi, 1, 0, &self.config.can[2])?;
        Self::update_can_config(&self.aux_spi, 1, 1, &self.config.can[3])?;
        if self.config.enable_aux {
            Self::update_can_config(&self.primary_spi, 0, 0, &self.config.can[4])?;
        }
        Ok(())
    }

    fn test_can(spi: &dyn Spi, cs: usize, name: &str) -> Result<()> {
        let version = read_byte(spi, cs, 0);
        if version != 2 && version != 3 && version != 4 {
            return Err(Error::message(format!(
                "Processor '{}' has incorrect CAN SPI version {} != [2,3,4]",
                name, version
            )));
        }
        Ok(())
    }

    fn verify_versions(&mut self) -> Result<()> {
        const ATTITUDE_VERSION: u8 = 0x20;
        const RF_VERSION: u8 = 0x10;

        if self.config.enable_aux {
            Self::test_can(&self.primary_spi, 0, "aux")?;
        }
        Self::test_can(&self.aux_spi, 0, "can1")?;
        Self::test_can(&self.aux_spi, 1, "can2")?;

        if self.config.enable_aux {
            let attitude_version = read_byte(&self.primary_spi, 0, 32);
            if attitude_version != ATTITUDE_VERSION {
                return Err(Error::message(format!(
                    "Incorrect attitude version {} != {}",
                    attitude_version, ATTITUDE_VERSION
                )));
            }

            let rf_version = read_byte(&self.primary_spi, 0, 48);
            if rf_version != RF_VERSION {
                return Err(Error::message(format!(
                    "Incorrect RF version {} != {}",
                    rf_version, RF_VERSION
                )));
            }
        }

        Ok(())
    }

    /// Query the firmware version and serial number information from
    /// each of the on-board processors.
    pub fn device_info(&mut self) -> DeviceInfo {
        // Verify all the CAN protocols for "unknown address safety".
        let can1_can_protocol = read_byte(&self.aux_spi, 0, 0);
        let can2_can_protocol = read_byte(&self.aux_spi, 1, 0);
        let aux_can_protocol = if self.config.enable_aux {
            read_byte(&self.primary_spi, 0, 0)
        } else {
            can1_can_protocol
        };

        // Now get the device information from all three processors.
        DeviceInfo {
            can1: get_processor_info(&self.aux_spi, 0),
            can2: get_processor_info(&self.aux_spi, 1),
            aux: if self.config.enable_aux {
                get_processor_info(&self.primary_spi, 0)
            } else {
                ProcessorInfo::default()
            },
            can_unknown_address_safe: can1_can_protocol > 3
                && can2_can_protocol > 3
                && aux_can_protocol > 3,
        }
    }

    /// Query the current processor utilization of each of the
    /// on-board processors.
    pub fn device_performance(&mut self) -> DevicePerformance {
        DevicePerformance {
            can1: get_performance(&self.aux_spi, 0),
            can2: get_performance(&self.aux_spi, 1),
            aux: if self.config.enable_aux {
                get_performance(&self.primary_spi, 0)
            } else {
                PerformanceInfo::default()
            },
        }
    }

    /// Read raw SPI data.
    pub fn read_spi(&mut self, spi_bus: i32, address: u8, data: &mut [u8]) {
        if spi_bus == 0 {
            self.aux_spi.read(0, address, data);
        } else if spi_bus == 1 {
            self.aux_spi.read(1, address, data);
        } else if spi_bus == 2 {
            self.primary_spi.read(0, address, data);
        }
    }

    fn get_attitude(&mut self, output: &mut Attitude, wait: bool, detail: bool) -> bool {
        // Busy loop until we get something.
        if wait {
            let mut buf = [0u8; 2];
            loop {
                self.primary_spi.read(0, 96, &mut buf);
                if buf[1] == 1 {
                    break;
                }
                // If we spam the STM32 too hard, then it doesn't have
                // any cycles left to actually work on the IMU.
                busy_wait_us(20);
            }
        }

        let read_size = if detail {
            ATTITUDE_DETAIL_SIZE
        } else {
            ATTITUDE_BASIC_SIZE
        };
        let device_attitude = loop {
            let mut buf = [0u8; ATTITUDE_DETAIL_SIZE];
            self.primary_spi.read(0, 34, &mut buf[..read_size]);
            if !(wait && buf[0] & 0x01 == 0) {
                break buf;
            }
        };

        if device_attitude[0] & 0x01 == 0 {
            return false;
        }

        let da = &device_attitude;
        output.attitude = Quaternion {
            w: f32_at(da, 2) as f64,
            x: f32_at(da, 6) as f64,
            y: f32_at(da, 10) as f64,
            z: f32_at(da, 14) as f64,
        };
        output.rate_dps = Point3D {
            x: f32_at(da, 18) as f64,
            y: f32_at(da, 22) as f64,
            z: f32_at(da, 26) as f64,
        };
        output.accel_mps2 = Point3D {
            x: f32_at(da, 30) as f64,
            y: f32_at(da, 34) as f64,
            z: f32_at(da, 38) as f64,
        };
        output.bias_dps = Point3D {
            x: f32_at(da, 42) as f64,
            y: f32_at(da, 46) as f64,
            z: f32_at(da, 50) as f64,
        };
        output.attitude_uncertainty = Quaternion {
            w: f32_at(da, 54) as f64,
            x: f32_at(da, 58) as f64,
            y: f32_at(da, 62) as f64,
            z: f32_at(da, 66) as f64,
        };
        output.bias_uncertainty_dps = Point3D {
            x: f32_at(da, 70) as f64,
            y: f32_at(da, 74) as f64,
            z: f32_at(da, 78) as f64,
        };

        true
    }

    fn send_can_packet(&self, can_frame: &CanFrame) {
        let mut buf = [0u8; 70];
        let (spi_address, spi_size, spi, cs): (u8, usize, &dyn Spi, usize) = match can_frame.bus {
            1 => {
                let (a, s) = encode_can_packet(0, can_frame, &mut buf);
                (a, s, &self.aux_spi, 0)
            }
            2 => {
                let (a, s) = encode_can_packet(1, can_frame, &mut buf);
                (a, s, &self.aux_spi, 0)
            }
            3 => {
                let (a, s) = encode_can_packet(0, can_frame, &mut buf);
                (a, s, &self.aux_spi, 1)
            }
            4 => {
                let (a, s) = encode_can_packet(1, can_frame, &mut buf);
                (a, s, &self.aux_spi, 1)
            }
            5 => {
                if !self.config.enable_aux {
                    return;
                }
                let (a, s) = encode_can_packet(0, can_frame, &mut buf);
                (a, s, &self.primary_spi, 0)
            }
            _ => return,
        };
        spi.write(cs, spi_address, &buf[..spi_size]);
    }

    fn calculate_expected_reply(&self, tx_can: &[CanFrame]) -> ExpectedReply {
        let mut result = ExpectedReply::default();

        for frame in tx_can {
            let bus = frame.bus;
            // Guard against out-of-range bus numbers; the C++ library
            // indexes its per-bus arrays unchecked here (UB for a bad
            // `CanFrame::bus`).  Such frames are silently skipped, in
            // both this accounting and in send_can below.
            if !(1..=5).contains(&bus) {
                continue;
            }
            let busi = bus as usize;
            let can_config = &self.config.can[busi - 1];

            let arbitration_bitrate = can_config.slow_bitrate as i64;
            let data_bitrate = if can_config.bitrate_switch {
                can_config.fast_bitrate as i64
            } else {
                can_config.slow_bitrate as i64
            };

            let can_header_size_bits: i64 = (if frame.id >= 2048 { 32 } else { 16 }) + 8;
            let can_data_size_bits = frame.size as i64 * 8 + 28;
            let tx_spi_bits = (frame.size as i64 + 5) * 8;
            let can_send_ns = 1_000_000_000 * can_header_size_bits / arbitration_bitrate
                + 1_000_000_000 * can_data_size_bits / data_bitrate
                + 1_000_000_000 * tx_spi_bits / self.config.spi_speed_hz as i64;

            result.send_ns[busi] += can_send_ns;

            if frame.expect_reply {
                let rx_header_size_bits: i64 = 32;
                let rx_data_size_bits = frame.expected_reply_size as i64 * 8 + 28;
                let rx_spi_bits = (frame.expected_reply_size as i64 + 5) * 8;
                let rx_ns = 1_000_000_000 * rx_header_size_bits / arbitration_bitrate
                    + 1_000_000_000 * rx_data_size_bits / data_bitrate
                    + 1_000_000_000 * rx_spi_bits / self.config.spi_speed_hz as i64;

                result.receive_ns[busi] += rx_ns;

                result.count[busi] += 1;
            }
        }
        result
    }

    fn send_can(&mut self, tx_can: &[CanFrame]) {
        // We try to send packets on alternating buses if possible, so
        // we can reduce the average latency before the first data
        // goes out on any bus.
        for bus_packets in self.can_packets.iter_mut() {
            bus_packets.clear();
        }

        for (i, frame) in tx_can.iter().enumerate() {
            let bus = frame.bus;
            if (1..=5).contains(&bus) {
                self.can_packets[bus as usize].push(i);
            }
        }

        let mut bus_offset = [0usize; 6];
        loop {
            // We try to send out packets to buses in this order to
            // minimize latency.
            let mut any_sent = false;
            for bus in [1usize, 3, 5, 2, 4] {
                let offset = &mut bus_offset[bus];
                if *offset >= self.can_packets[bus].len() {
                    continue;
                }
                let can_packet = &tx_can[self.can_packets[bus][*offset]];
                *offset += 1;

                self.send_can_packet(can_packet);
                any_sent = true;
            }

            if !any_sent {
                break;
            }
        }
    }

    fn send_rf(&self, slots: &[RfSlot]) {
        if !self.config.enable_aux {
            return;
        }

        const HEADER_SIZE: usize = 5;
        const MAX_DATA_SIZE: usize = 16;
        let mut buf = [0u8; HEADER_SIZE + MAX_DATA_SIZE];

        for slot in slots {
            buf[0] = slot.slot;
            buf[1..5].copy_from_slice(&slot.priority.to_le_bytes());
            let size = slot.size as usize;
            buf[HEADER_SIZE..HEADER_SIZE + size].copy_from_slice(&slot.data[..size]);

            self.primary_spi.write(0, 51, &buf[..HEADER_SIZE + size]);
        }
    }

    fn read_rf(&mut self, rx_rf: &mut [RfSlot], output: &mut Output) {
        if !self.config.enable_aux {
            return;
        }

        let mut status_buf = [0u8; 8];
        self.primary_spi.read(0, 52, &mut status_buf);
        let bitfield = u32_at(&status_buf, 0);
        let lock_age_ms = u32_at(&status_buf, 4);

        output.rf_lock_age_ms = lock_age_ms;

        let bitfield_delta = bitfield ^ self.last_bitfield;
        if bitfield_delta == 0 {
            return;
        }

        for i in 0..15 {
            if output.rx_rf_size >= rx_rf.len() {
                // No more room.
                return;
            }

            if bitfield_delta & (3 << (i * 2)) == 0 {
                continue;
            }

            self.last_bitfield ^= bitfield_delta & (3 << (i * 2));

            // DeviceSlotData: age_ms u32, size u8, data[16].
            let mut slot_buf = [0u8; 21];
            self.primary_spi.read(0, 64 + i, &mut slot_buf);

            let output_slot = &mut rx_rf[output.rx_rf_size];
            output.rx_rf_size += 1;
            output_slot.slot = i;
            output_slot.age_ms = u32_at(&slot_buf, 0);
            // Clamp a malformed device-reported size to the slot
            // capacity so that both this copy and any later
            // `data[..size]` slicing by the caller stay in bounds.
            // (The C++ library copies unclamped, a stack overflow on
            // such input.)
            output_slot.size = slot_buf[4].min(16);
            let size = output_slot.size as usize;
            output_slot.data[..size].copy_from_slice(&slot_buf[5..5 + size]);
        }
    }

    fn read_can_frames(
        spi: &dyn Spi,
        cs: usize,
        bus_start: i32,
        rx_can: &mut [CanFrame],
        output: &mut Output,
    ) -> i32 {
        // Is there any room?
        if output.rx_can_size >= rx_can.len() {
            return 0;
        }

        let mut count = 0;

        let mut buf = [0u8; 70];

        // Read until no more frames are available or until the
        // output buffer is full.
        let mut queue_sizes = [0u8; 6];
        spi.read(cs, 2, &mut queue_sizes);

        for &queue_size in queue_sizes.iter() {
            if output.rx_can_size >= rx_can.len() {
                // We're full and can't read any more.
                break;
            }

            if queue_size == 0 {
                continue;
            }
            let size = if queue_size as usize > 64 + 5 {
                // This is malformed.  Lets just set it to the maximum
                // size for now.
                64 + 5
            } else {
                queue_size as usize
            };

            spi.read(cs, 3, &mut buf[..size]);

            if buf[0] == 0 {
                // Hmmm, this shouldn't happen, but indicates there
                // isn't really a frame here.
                continue;
            }

            if size < 5 {
                // A real frame always carries the 5-byte header (flags
                // byte plus 4-byte id), so anything shorter is
                // malformed.  We have already popped it from the device
                // queue with the read above, so just drop it rather
                // than underflowing `size - 5` below.
                continue;
            }

            let output_frame = &mut rx_can[output.rx_can_size];
            output.rx_can_size += 1;
            count += 1;

            output_frame.bus = bus_start + (if buf[0] & 0x80 != 0 { 1 } else { 0 });
            output_frame.id = ((buf[1] as u32) << 24)
                | ((buf[2] as u32) << 16)
                | ((buf[3] as u32) << 8)
                | (buf[4] as u32);
            output_frame.size = (size - 5) as u8;
            output_frame.data[..size - 5].copy_from_slice(&buf[5..size]);
        }

        count
    }

    fn flush_read_can(
        &self,
        force_can_check: u32,
        expected_replies: &ExpectedReply,
        rx_can: &mut [CanFrame],
        output: &mut Output,
    ) {
        let can1_expected = (expected_replies.count[1] + expected_replies.count[2]) != 0
            || force_can_check & 0x06 != 0;
        if can1_expected {
            Self::read_can_frames(&self.aux_spi, 0, 1, rx_can, output);
        }

        let can2_expected = (expected_replies.count[3] + expected_replies.count[4]) != 0
            || force_can_check & 0x18 != 0;
        if can2_expected {
            Self::read_can_frames(&self.aux_spi, 1, 3, rx_can, output);
        }

        let aux_expected = (expected_replies.count[5] != 0 || force_can_check & 0x20 != 0)
            && self.config.enable_aux;
        if aux_expected {
            Self::read_can_frames(&self.primary_spi, 0, 5, rx_can, output);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn read_can(
        &self,
        timeout_ns: u32,
        min_tx_wait_ns: u32,
        rx_baseline_wait_ns: u32,
        rx_extra_wait_ns: u32,
        force_can_check: u32,
        expected_replies: &ExpectedReply,
        rx_can: &mut [CanFrame],
        output: &mut Output,
    ) {
        let mut bus_replies = [
            expected_replies.count[1] + expected_replies.count[2],
            expected_replies.count[3] + expected_replies.count[4],
            expected_replies.count[5],
        ];

        let min_rx_timeout_ns: i64 = {
            let mut biggest: i64 = 0;
            for bus in 1..=5usize {
                let this_ns = if expected_replies.count[bus] != 0 {
                    expected_replies.send_ns[bus] + expected_replies.receive_ns[bus]
                } else {
                    0
                };
                if this_ns > biggest {
                    biggest = this_ns;
                }
            }
            biggest + rx_baseline_wait_ns as i64
        };

        let to_check = [
            bus_replies[0] != 0 || force_can_check & 0x06 != 0,
            bus_replies[1] != 0 || force_can_check & 0x18 != 0,
            bus_replies[2] != 0 || force_can_check & 0x20 != 0,
        ];

        let start_now = get_now();
        let mut last_reply = start_now;

        loop {
            let mut any_found = false;
            // Then check for CAN responses as necessary.
            if to_check[0] {
                let count = Self::read_can_frames(&self.aux_spi, 0, 1, rx_can, output);
                bus_replies[0] -= count;
                if count != 0 {
                    last_reply = get_now();
                    any_found = true;
                }
            }
            if to_check[1] {
                let count = Self::read_can_frames(&self.aux_spi, 1, 3, rx_can, output);
                bus_replies[1] -= count;
                if count != 0 {
                    last_reply = get_now();
                    any_found = true;
                }
            }
            if to_check[2] && self.config.enable_aux {
                let count = Self::read_can_frames(&self.primary_spi, 0, 5, rx_can, output);
                bus_replies[2] -= count;
                if count != 0 {
                    last_reply = get_now();
                    any_found = true;
                }
            }

            if output.rx_can_size >= rx_can.len() {
                // Our buffer is full, so no more frames could have
                // been returned.
                return;
            }

            let cur_now = get_now();
            let delta_ns = cur_now - start_now;
            let since_last_ns = cur_now - last_reply;

            if bus_replies[0] <= 0
                && bus_replies[1] <= 0
                && bus_replies[2] <= 0
                && delta_ns > min_tx_wait_ns as i64
                && since_last_ns > rx_extra_wait_ns as i64
            {
                // We've read all the replies we are expecting and
                // have polled everything at least once if requested.
                return;
            }

            if delta_ns > timeout_ns as i64
                && !(delta_ns < min_tx_wait_ns as i64
                    || delta_ns < min_rx_timeout_ns
                    || since_last_ns < rx_extra_wait_ns as i64)
            {
                // The timeout has expired.
                return;
            }

            if !any_found {
                // Give the controllers a chance to rest.
                busy_wait_us(10);
            }
        }
    }

    /// Do some or all of the following:
    ///  * Send the given CAN frames
    ///  * Wait for at least the requested number of replies (on a
    ///    per-bus basis), or the given timeout
    ///  * Read the current ARS result
    ///  * Send any desired RF slots
    ///  * Return any RF slots that may have been received
    pub fn cycle(&mut self, input: Input<'_>) -> Output {
        let Input {
            tx_can,
            tx_rf,
            timeout_ns,
            min_tx_wait_ns,
            rx_baseline_wait_ns,
            rx_extra_wait_ns,
            request_attitude,
            request_attitude_detail,
            wait_for_attitude,
            request_rf,
            force_can_check,
            rx_can,
            rx_rf,
            attitude,
        } = input;

        let mut result = Output::default();

        let expected_replies = self.calculate_expected_reply(tx_can);

        // First, ensure there aren't any receive frames sitting
        // around before we start for CAN busses we expect to have a
        // reply for.
        self.flush_read_can(force_can_check, &expected_replies, rx_can, &mut result);

        // Send off all our CAN data to all buses.
        self.send_can(tx_can);

        // While those are sending, do our other work.
        if !tx_rf.is_empty() {
            self.send_rf(tx_rf);
        }

        if request_rf {
            self.read_rf(rx_rf, &mut result);
        }

        if request_attitude {
            if let Some(attitude) = attitude {
                result.attitude_present =
                    self.get_attitude(attitude, wait_for_attitude, request_attitude_detail);
            }
        }

        self.read_can(
            timeout_ns,
            min_tx_wait_ns,
            rx_baseline_wait_ns,
            rx_extra_wait_ns,
            force_can_check,
            &expected_replies,
            rx_can,
            &mut result,
        );

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_round_up_dlc() {
        let cases = [
            (0usize, 0usize),
            (1, 1),
            (7, 7),
            (8, 8),
            (9, 12),
            (12, 12),
            (13, 16),
            (17, 20),
            (21, 24),
            (25, 32),
            (33, 48),
            (49, 64),
            (64, 64),
            (65, 0),
        ];
        for (input, expected) in cases {
            assert_eq!(round_up_dlc(input), expected, "input {}", input);
        }
    }

    #[test]
    fn test_parse_rpi_version() {
        assert_eq!(
            parse_rpi_version("Raspberry Pi 4 Model B Rev 1.4").unwrap(),
            4
        );
        assert_eq!(parse_rpi_version("Raspberry Pi 3 Model B Plus").unwrap(), 3);
        assert_eq!(
            parse_rpi_version("Raspberry Pi Compute Module 4").unwrap(),
            4
        );
        assert!(parse_rpi_version("Banana Pi").is_err());
    }

    #[test]
    fn test_encode_can_packet_short_id() {
        let frame = CanFrame {
            id: 0x8001,
            data: {
                let mut data = [0u8; 64];
                data[0] = 0xaa;
                data[1] = 0xbb;
                data[2] = 0xcc;
                data
            },
            size: 3,
            bus: 1,
            expect_reply: true,
            expected_reply_size: 8,
        };

        let mut buf = [0u8; 70];
        let (address, size) = encode_can_packet(0, &frame, &mut buf);
        assert_eq!(address, 5);
        assert_eq!(size, 6);
        assert_eq!(&buf[..6], &[0x03, 0x80, 0x01, 0xaa, 0xbb, 0xcc]);

        // The same frame on the second cpu bus sets the high bit of
        // the first byte.
        let (address, _size) = encode_can_packet(1, &frame, &mut buf);
        assert_eq!(address, 5);
        assert_eq!(buf[0], 0x83);
    }

    #[test]
    fn test_encode_can_packet_long_id() {
        let frame = CanFrame {
            id: 0x12345678,
            data: {
                let mut data = [0u8; 64];
                for (i, item) in data.iter_mut().enumerate().take(9) {
                    *item = i as u8;
                }
                data
            },
            size: 9,
            bus: 1,
            ..CanFrame::default()
        };

        let mut buf = [0u8; 70];
        let (address, size) = encode_can_packet(0, &frame, &mut buf);
        assert_eq!(address, 4);
        // DLC rounds 9 up to 12.
        assert_eq!(size, 5 + 12);
        assert_eq!(&buf[..5], &[0x0c, 0x12, 0x34, 0x56, 0x78]);
        assert_eq!(&buf[5..14], &[0, 1, 2, 3, 4, 5, 6, 7, 8]);
        // Padding bytes.
        assert_eq!(&buf[14..17], &[0x50, 0x50, 0x50]);
    }

    #[test]
    fn test_device_can_configuration_roundtrip() {
        let config = DeviceCanConfiguration {
            slow_bitrate: 125000,
            fast_bitrate: 1000000,
            fdcan_frame: 0,
            bitrate_switch: 0,
            automatic_retransmission: 1,
            restricted_mode: 0,
            bus_monitor: 1,
            std_rate: [10, 1, 2, 3],
            fd_rate: [-1, -1, -1, -1],
            cancel_all_ms: 100,
        };
        let bytes = config.to_bytes();
        assert_eq!(bytes.len(), DeviceCanConfiguration::V4_SIZE);
        let parsed = DeviceCanConfiguration::from_bytes(&bytes);
        assert_eq!(config, parsed);
    }

    #[test]
    fn test_device_imu_configuration_roundtrip() {
        let config = DeviceImuConfiguration {
            roll_deg: 1.5,
            pitch_deg: -2.5,
            yaw_deg: 90.0,
            rate_hz: 400,
        };
        let parsed = DeviceImuConfiguration::from_bytes(&config.to_bytes());
        assert_eq!(config, parsed);
    }
}
