# -*- python -*-

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

workspace(name = "com_github_mjbots_quad_pi3_hat")

BAZEL_VERSION = "7.4.1"
BAZEL_VERSION_SHA = "c97f02133adce63f0c28678ac1f21d65fa8255c80429b588aeeba8a1fac6202b"

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

load("@rpi_bazel//tools/workspace:default.bzl",
     rpi_bazel_add = "add_default_repositories",
     rpi_bazel_register = "add_default_toolchains")
rpi_bazel_add()

# Register rpi_bazel's Raspberry Pi cross-compilation toolchains so they
# participate in Bazel's platform-based toolchain resolution (selected
# via --config=pi / --config=pi64, i.e. --platforms=@rpi_bazel//:...).
rpi_bazel_register()

load("@com_github_mjbots_rules_mbed//:rules.bzl", mbed_register = "mbed_register")

mbed_register(
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE",
        "mbed_config": {
            "MBED_CONF_RTOS_PRESENT": "0",
            "MBED_CONF_TARGET_LSE_AVAILABLE": "0",
            "DEVICE_STDIO_MESSAGES": "0",
            "NDEBUG": "1",
        },
    },
)

load("@com_github_mjbots_bazel_deps//tools/workspace:default.bzl",
     bazel_deps_add = "add_default_repositories")
bazel_deps_add()

load("@moteus//tools/workspace:default.bzl",
     moteus_add = "add_default_repositories")
moteus_add()

load("@com_github_mjbots_mjlib//tools/workspace:default.bzl",
     mjlib_add = "add_default_repositories")
mjlib_add()

load("@rules_pkg//:deps.bzl", "rules_pkg_dependencies")
rules_pkg_dependencies()

# Rust toolchain via rules_rust.  The rules_rust repository itself is
# brought in by moteus_add() above.  We mirror the moteus repository's
# Rust setup so that the @moteus//lib/rust/... Bazel targets (which the
# moteus-pi3hat crate depends on) resolve against a single shared
# @crate_index.
load("@rules_rust//rust:repositories.bzl", "rules_rust_dependencies", "rust_register_toolchains")

rules_rust_dependencies()

rust_register_toolchains(
    edition = "2021",
    # The extra target triples provide std for cross-compiling the Rust
    # example binaries to the Raspberry Pi: armv7 via --config=pi
    # (--platforms=@rpi_bazel//:armeabihf) and aarch64 via --config=pi64
    # (--platforms=@rpi_bazel//:aarch64).  Host (x86_64) is registered by
    # default for the test suite.
    extra_target_triples = [
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
    versions = ["1.82.0"],
)

load("@rules_rust//crate_universe:repositories.bzl", "crate_universe_dependencies")

crate_universe_dependencies()

load("@rules_rust//crate_universe:defs.bzl", "crate", "crates_repository")

# Third-party crates for the Rust client library and examples: the
# specs moteus itself builds with (exported from its crates.bzl, so
# they cannot drift from moteus's own crate_index) plus moteus-pi3hat's
# own dependency (libc).
load("@moteus//lib/rust:crates.bzl", "moteus_crate_packages")

crates_repository(
    name = "crate_index",
    cargo_lockfile = "//lib/rust:Cargo.lock",
    lockfile = "//lib/rust:Cargo.bazel.lock",
    # Triples the third-party crates must be rendered for: the host (for
    # the test suite and proc-macro/build-script execution) plus the two
    # Raspberry Pi cross-compilation targets used by the example
    # binaries.
    supported_platform_triples = [
        "x86_64-unknown-linux-gnu",
        "armv7-unknown-linux-gnueabihf",
        "aarch64-unknown-linux-gnu",
    ],
    packages = dict(
        moteus_crate_packages(),
        libc = crate.spec(version = "0.2"),
    ),
)

load("@crate_index//:defs.bzl", "crate_repositories")

crate_repositories()
