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

load("@rules_pkg//pkg:mappings.bzl", "pkg_files", "pkg_attributes")
load("@rules_pkg//pkg:tar.bzl", "pkg_tar")
load("@moteus//lib/rust:examples.bzl", "MOTEUS_RUST_EXAMPLES")

filegroup(
    name = "target",
    srcs = [
        "//fw:pi3_hat.bin",
    ],
)

filegroup(
    name = "pi_target",
    srcs = [
        "//lib/cpp/mjbots/pi3hat",
        "examples",
    ],
)

EXAMPLES = [
    "simple",
    "diagnostic_protocol",
    "multiple_cycle",
    "simple_teleop",
    "nlateral_teleop",
    "timeout_debug",
    "bandwidth_test",
    "imu_transport",
]

pkg_files(
    name = "examples",
    srcs = ["//lib/cpp/examples:" + x for x in EXAMPLES],
    prefix = "examples",
    attributes = pkg_attributes(
        mode = "0755",
    ),
)

# Rust example binaries, cross-compiled for the same architecture as
# the rest of the bundle (armv7 via --config=pi, aarch64 via
# --config=pi64).  They go under examples/rust/ to avoid colliding with
# the like-named C++ examples (e.g. "simple").  These are the
# pi3hat-specific examples; the moteus examples (rebuilt over the pi3hat
# transport) are added flat alongside them below.
RUST_EXAMPLES = [
    "imu",
    "simple_async",
]

pkg_files(
    name = "rust_examples",
    srcs = ["//lib/rust:" + x for x in RUST_EXAMPLES],
    prefix = "examples/rust",
    attributes = pkg_attributes(
        mode = "0755",
    ),
)

# The full set of moteus examples rebuilt over the pi3hat transport
# (//lib/rust/moteus_examples), generated automatically from moteus's
# exported MOTEUS_RUST_EXAMPLES list.  Same examples/rust prefix; the
# names do not collide with the pi3hat-specific examples above.
pkg_files(
    name = "moteus_rust_examples",
    srcs = ["//lib/rust/moteus_examples:" + x for x in MOTEUS_RUST_EXAMPLES],
    prefix = "examples/rust",
    attributes = pkg_attributes(
        mode = "0755",
    ),
)

pkg_files(
    name = "example_srcs",
    srcs = [
        "@moteus//lib/cpp/mjbots/moteus:src",
        "@moteus//lib/cpp/examples:src",
        "//lib/cpp/examples:src",
        "//lib/cpp/mjbots/pi3hat:src",
    ],
    prefix = "src",
)

# Source for the Rust examples, shipped alongside the binaries for
# reference (the crate itself is published as moteus-pi3hat).  Unlike
# the binary list above, "simple" is included here: only the binary
# name collides with the moteus example, not the source path.
pkg_files(
    name = "rust_example_srcs",
    srcs = [
        "//lib/rust/moteus-pi3hat:examples/" + x + ".rs"
        for x in RUST_EXAMPLES + ["simple"]
    ],
    prefix = "src/rust",
)


pkg_tar(
    name = "pi3hat_tools",
    extension = "tar.bz2",
    package_dir = "pi3hat_tools",
    srcs = [
        "//lib/cpp/mjbots/pi3hat:pi3hat_tool",
        "//lib/python:bdist_wheel",
        ":examples",
        ":rust_examples",
        ":moteus_rust_examples",
        ":example_srcs",
        ":rust_example_srcs",
    ],
)

test_suite(
    name = "host",
    tests = [
        "//lib/rust:host",
    ],
)

[config_setting(
    name = "python{py_nodot_ver}_{arch}".format(
        py_nodot_ver=pyver.replace('.',''),
        arch=arch),
    values = {
        "define": "PYTHON={pyver}".format(pyver=pyver),
        "cpu": arch,
    },
 )
 for pyver in ['3.7', '3.9', '3.10', '3.11', '3.12', '3.13']
 for arch in ['armeabihf', 'aarch64']
]

config_setting(
    name = "python37",
    values = {
        "define": "PYTHON=3.7",
    },
)

config_setting(
    name = "python39",
    values = {
        "define": "PYTHON=3.9",
    },
)

config_setting(
    name = "python310",
    values = {
        "define": "PYTHON=3.10",
    },
)

config_setting(
    name = "python311",
    values = {
        "define": "PYTHON=3.11",
    },
)

config_setting(
    name = "python312",
    values = {
        "define": "PYTHON=3.12",
    },
)

config_setting(
    name = "python313",
    values = {
        "define": "PYTHON=3.13",
    },
)
