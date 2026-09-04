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

//! The async variant of the `simple` example: commands a single
//! servo with ID 1 attached to a pi3hat to hold the current position
//! indefinitely, and prints the state of the servo to the console.
//!
//! Because the pi3hat transport accesses /dev/mem, it must be run as
//! root:
//!
//!   cargo build --features tokio --example simple_async
//!   sudo ./target/debug/examples/simple_async

use moteus::command::PositionCommand;
use moteus::{AsyncController, TransportOptions};
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), moteus::Error> {
    // Make the pi3hat transport available.  With the tokio feature
    // enabled, this registers both the blocking and the async
    // factories.
    moteus_pi3hat::transport::register();

    let opts = TransportOptions::new().force_transport("pi3hat");
    let mut c = AsyncController::with_options(1, &opts).await?;

    // Clear any faults by sending a stop command.
    c.set_stop().await?;

    loop {
        // Hold the current position.  A NaN position with unset
        // velocity (0.0) means "hold where you are".
        let state = c
            .set_position(PositionCommand::new().position(f32::NAN))
            .await?;
        println!("{:?}", state);
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}
