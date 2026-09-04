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

//! Commands a single servo with ID 1 attached to a pi3hat to hold the
//! current position indefinitely, and prints the state of the servo to
//! the console.  This is the blocking counterpart to `simple_async`.
//!
//! Because the pi3hat transport accesses /dev/mem, it must be run as
//! root:
//!
//!   cargo build --example simple
//!   sudo ./target/debug/examples/simple

use moteus::command::PositionCommand;
use moteus::{BlockingController, TransportOptions};
use std::time::Duration;

fn main() -> Result<(), moteus::Error> {
    // Make the pi3hat transport available.
    moteus_pi3hat::transport::register();

    // Once registered, the pi3hat participates in transport
    // auto-detection, so `force_transport` is only needed to
    // disambiguate it from other attached transports.  If your
    // application is fine with all detected CAN-FD transports being
    // used simultaneously, this is not necessary.
    let opts = TransportOptions::new().force_transport("pi3hat");
    let mut c = BlockingController::with_options(1, &opts)?;

    // Clear any faults by sending a stop command.
    c.set_stop()?;

    loop {
        // Hold the current position.  A NaN position with unset
        // velocity (0.0) means "hold where you are".
        let state = c.set_position(PositionCommand::new().position(f32::NAN))?;
        println!("{:?}", state);
        std::thread::sleep(Duration::from_millis(20));
    }
}
