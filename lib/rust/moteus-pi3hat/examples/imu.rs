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

//! This example reads the attitude from the pi3hat IMU using the raw
//! `Pi3Hat` interface and prints it to the console.
//!
//! Because the pi3hat driver accesses /dev/mem, it must be run as
//! root:
//!
//!   cargo build --example imu
//!   sudo ./target/debug/examples/imu

use moteus_pi3hat::{Attitude, Configuration, Input, Pi3Hat};

fn main() -> Result<(), moteus_pi3hat::Error> {
    let mut pi3hat = Pi3Hat::new(&Configuration::default())?;

    loop {
        let mut attitude = Attitude::default();
        let output = pi3hat.cycle(Input {
            request_attitude: true,
            wait_for_attitude: true,
            attitude: Some(&mut attitude),
            ..Input::default()
        });

        if output.attitude_present {
            let q = attitude.attitude;
            let r = attitude.rate_dps;
            println!(
                "quat=({:7.4},{:7.4},{:7.4},{:7.4})  rate_dps=({:7.2},{:7.2},{:7.2})",
                q.w, q.x, q.y, q.z, r.x, r.y, r.z
            );
        }
    }
}
