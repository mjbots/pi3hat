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

workspace(name = "com_github_jpieper_mjmech_pi3_hat")

BAZEL_VERSION = "0.25.2"
BAZEL_VERSION_SHA = "c6b9f3a3350419521461c6d9bdbfd09b4f745af564e1728e2fbf9d4d343f93b6"

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

load("@com_github_mjbots_rules_mbed//:rules.bzl", mbed_register = "mbed_register")

mbed_register(
    config = {
        "mbed_target": "targets/TARGET_STM/TARGET_STM32F0/TARGET_DISCO_F051R8",
        "mbed_config": {
            "MBED_CONF_RTOS_PRESENT": "0",
            "DEVICE_STDIO_MESSAGES": "0",
            "NDEBUG": "1",
        },
    },
)
