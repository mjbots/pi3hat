# -*- python -*-

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

# Rust client library for the mjbots pi3hat.
# See moteus-pi3hat/ for the library crate.

load("@rules_rust//rust:defs.bzl", "rust_binary", "rustfmt_test")
load("@moteus//lib/rust:examples.bzl", "MOTEUS_RUST_EXAMPLES")

package(default_visibility = ["//visibility:public"])

# Aggregate of all host Rust checks, wired into //:host for CI.
test_suite(
    name = "host",
    tests = [
        "//lib/rust/moteus-pi3hat:moteus-pi3hat-test",
        "//lib/rust/moteus-pi3hat:moteus-pi3hat-test-tokio",
        "//lib/rust/moteus-pi3hat:moteus-pi3hat-doc-test",
        "//lib/rust/moteus-pi3hat:moteus-pi3hat-doc-test-tokio",
        ":clippy",
        ":examples",
        ":rustdoc",
        ":fmt",
    ],
)

# Clippy lint checks.  Wrapped in sh_test so they run as part of the
# test suite (the data deps must build for the test to be set up).
sh_test(
    name = "clippy",
    srcs = ["clippy_test.sh"],
    data = [
        "//lib/rust/moteus-pi3hat:moteus-pi3hat-clippy",
        "//lib/rust/moteus-pi3hat:moteus-pi3hat-tokio-clippy",
    ],
    size = "small",
)

# Formatting check using the bazel Rust toolchain's rustfmt.
rustfmt_test(
    name = "fmt",
    targets = [
        "//lib/rust/moteus-pi3hat:moteus_pi3hat",
    ],
    size = "small",
)

# Rustdoc lint checks (warnings as errors).
sh_test(
    name = "rustdoc",
    srcs = ["clippy_test.sh"],
    data = [
        "//lib/rust/moteus-pi3hat:moteus-pi3hat-doc",
    ],
    size = "small",
)

# Ensure all example binaries build: the pi3hat-specific ones plus the
# full set of moteus examples rebuilt over the pi3hat transport.
sh_test(
    name = "examples",
    srcs = ["clippy_test.sh"],
    data = [
        ":imu",
        ":simple",
        ":simple_async",
    ] + ["//lib/rust/moteus_examples:" + x for x in MOTEUS_RUST_EXAMPLES],
    size = "small",
)

# Example: stream IMU data (raw driver).
rust_binary(
    name = "imu",
    srcs = ["//lib/rust/moteus-pi3hat:examples/imu.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus-pi3hat:moteus_pi3hat",
    ],
)

# Example: hold position on a single servo (blocking).
rust_binary(
    name = "simple",
    srcs = ["//lib/rust/moteus-pi3hat:examples/simple.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus-pi3hat:moteus_pi3hat",
        "@moteus//lib/rust/moteus:moteus",
    ],
)

# Example: hold position on a single servo (async).
rust_binary(
    name = "simple_async",
    srcs = ["//lib/rust/moteus-pi3hat:examples/simple_async.rs"],
    edition = "2021",
    deps = [
        "//lib/rust/moteus-pi3hat:moteus_pi3hat_tokio",
        "@crate_index//:tokio",
        "@moteus//lib/rust/moteus:moteus_tokio",
    ],
)
