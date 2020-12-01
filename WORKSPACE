# -*- python -*-

# Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

BAZEL_VERSION = "3.4.1"
BAZEL_VERSION_SHA = "1a64c807716e10c872f1618852d95f4893d81667fe6e691ef696489103c9b460"

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

load("@rpi_bazel//tools/workspace:default.bzl",
     rpi_bazel_add = "add_default_repositories")
rpi_bazel_add()

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
