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

//! Client library for the mjbots pi3hat, a Raspberry Pi hat providing
//! CAN-FD and an attitude reference system.
//!
//! This is a direct translation of the C++ library in
//! `lib/cpp/mjbots/pi3hat`.  Like the C++ library, it drives the
//! Raspberry Pi SPI peripherals through `/dev/mem`, so it must run as
//! root, and only one process may use the pi3hat at a time.
//!
//! There are two levels of API:
//!
//! * [`Pi3Hat`] - the raw device interface, giving access to all CAN
//!   buses, the IMU, and the RF transceiver in a single blocking
//!   [`Pi3Hat::cycle`] operation.
//! * [`transport`] - an implementation of the `moteus` crate's
//!   transport layer, so that moteus controllers attached to a pi3hat
//!   can be used with the standard `moteus` APIs.  With the `tokio`
//!   feature enabled, the async transport is provided as well.
//!
//! # Using the moteus transport
//!
//! Call [`transport::register()`] once at startup (the Rust equivalent
//! of linking `pi3hat_moteus_transport_register.cc` in C++), then use
//! the `moteus` crate normally:
//!
//! ```no_run
//! use moteus::{BlockingController, TransportOptions};
//!
//! fn main() -> Result<(), moteus::Error> {
//!     moteus_pi3hat::transport::register();
//!
//!     let opts = TransportOptions::new().force_transport("pi3hat");
//!     let mut c = BlockingController::with_options(1, &opts)?;
//!     c.set_stop()?;
//!     Ok(())
//! }
//! ```

mod error;
mod pi3hat;
pub mod realtime;
pub mod transport;

pub use error::{Error, Result};
pub use pi3hat::{
    pi3hat_present, Attitude, CanConfiguration, CanFrame, CanRateOverride, Configuration,
    DeviceInfo, DevicePerformance, Euler, Input, Output, PerformanceInfo, Pi3Hat, Point3D,
    ProcessorInfo, Quaternion, RfSlot,
};
