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
    ],
)

EXAMPLES = [
    "simple",
    "diagnostic_protocol",
    "multiple_cycle",
    "simple_teleop",
    "nlateral_teleop",
    "timeout_debug",
]

pkg_files(
    name = "examples",
    srcs = ["//lib/cpp/examples:" + x for x in EXAMPLES],
    prefix = "examples",
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


pkg_tar(
    name = "pi3hat_tools",
    extension = "tar.bz2",
    package_dir = "pi3hat_tools",
    srcs = [
        "//lib/cpp/mjbots/pi3hat:pi3hat_tool",
        "//lib/python:bdist_wheel",
        ":examples",
        ":example_srcs",
    ],
)

test_suite(
    name = "host",
    tests = [
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
 for pyver in ['3.7', '3.9', '3.10', '3.11']
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
