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

load("@com_github_ARMmbed_mbed-os//:rules.bzl", "mbed_binary")

mbed_binary(
    name = "pi3_hat",
    srcs = [
        "fdcan.cc",
        "fdcan.h",
        "millisecond_timer.h",
        "pi3_hat.cc",
    ],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:inplace_function",
        "@com_github_mjbots_mjlib//mjlib/base:string_span",
        "@com_github_mjbots_mjlib//mjlib/micro:callback_table",
    ],
)

OCD = (
    "openocd " +
    "-f interface/stlink.cfg " +
    "-f target/stm32g4x.cfg "
)

genrule(
    name = "flash",
    tags = ["manual"],
    srcs = ["pi3_hat.bin"],
    outs = ["flash.stamp"],
    cmd = (OCD + "-c init -c \"reset_config none separate; program $(location pi3_hat.bin) verify 0x8000000 reset exit 0x8000000\" && touch $@"),
)
