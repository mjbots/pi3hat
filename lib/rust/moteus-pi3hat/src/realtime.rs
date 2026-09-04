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

//! Helpers for configuring realtime scheduling, a translation of
//! `lib/cpp/mjbots/pi3hat/realtime.h`.

use crate::error::{Error, Result};

/// Pin the calling thread to the given CPU and switch it to the
/// SCHED_RR realtime scheduler.
///
/// This requires elevated privileges (i.e. root).
pub fn configure_realtime(cpu: usize) -> Result<()> {
    unsafe {
        let mut cpuset: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut cpuset);
        libc::CPU_SET(cpu, &mut cpuset);

        let r = libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &cpuset);
        if r < 0 {
            return Err(Error::errno("Error setting CPU affinity"));
        }
    }

    unsafe {
        // Zero-initialize and set only sched_priority, like the C++
        // `struct sched_param params = {}`.  A struct literal would
        // fail to compile on libc variants (e.g. musl) whose
        // `sched_param` carries the optional POSIX sporadic-server
        // fields.
        let mut params: libc::sched_param = std::mem::zeroed();
        params.sched_priority = 10;
        let r = libc::sched_setscheduler(0, libc::SCHED_RR, &params);
        if r < 0 {
            // musl does not implement sched_setscheduler (it returns
            // ENOSYS); the CPU affinity set above is what keeps the
            // worker off the kernel's general CPUs, so tolerate that
            // one case and fail on anything else (e.g. EPERM).
            let err = std::io::Error::last_os_error();
            if err.raw_os_error() != Some(libc::ENOSYS) {
                return Err(Error::os(
                    "Error setting realtime scheduler, try running as root (use sudo)",
                    err,
                ));
            }
        }
    }

    Ok(())
}
