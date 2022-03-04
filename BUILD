# -*- python -*-

# Copyright 2020-2022 Josh Pieper, jjp@pobox.com.
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

load("@rules_pkg//:pkg.bzl", "pkg_tar")

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

pkg_tar(
    name = "pi3hat_tools",
    extension = "tar.bz2",
    package_dir = "pi3hat_tools",
    srcs = [
        "//lib/cpp/mjbots/pi3hat:pi3hat_tool",
        "//lib/cpp/mjbots/moteus:moteus_control_example",
        "//lib/python:bdist_wheel",
    ],
)

test_suite(
    name = "host",
    tests = [
        "//lib/cpp/mjbots/moteus:test",
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
 for pyver in ['3.7', '3.9']
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
