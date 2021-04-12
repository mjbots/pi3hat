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

package(default_visibility = ["//visibility:public"])

COPTS = [
    "-Werror",
    "-Wdouble-promotion",
]

cc_library(
    name = "git_info",
    hdrs = ["git_info.h"],
    srcs = ["git_info.cc"],
    linkstamp = "git_info_linkstamp.cc",
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:visitor",
    ],
    copts = COPTS,
)

mbed_binary(
    name = "pi3_hat",
    srcs = [
        "attitude_reference.h",
        "bmi088.cc",
        "bmi088.h",
        "can_bridge.h",
        "cpu_meter.h",
        "device_info.h",
        "euler.h",
        "fdcan.cc",
        "fdcan.h",
        "icm42688.cc",
        "icm42688.h",
        "imu_data.h",
        "imu.h",
        "imu.cc",
        "math_util.h",
        "millisecond_timer.h",
        "nrf24l01.cc",
        "nrf24l01.h",
        "pi3_hat.cc",
        "point3d.h",
        "quaternion.h",
        "register_spi_slave.h",
        "register_spi_slave.cc",
        "rf_transceiver.h",
        "slot_rf_protocol.h",
        "slot_rf_protocol.cc",
        "stm32_pwm_out.h",
        "ukf_filter.h",
    ],
    features = [
        "speedopt",
    ],
    deps = [
        ":git_info",
        "@com_github_mjbots_mjlib//mjlib/base:inplace_function",
        "@com_github_mjbots_mjlib//mjlib/base:string_span",
        "@com_github_mjbots_mjlib//mjlib/base:visitor",
        "@com_github_mjbots_mjlib//mjlib/micro:async_types",
        "@com_github_mjbots_mjlib//mjlib/micro:callback_table",
        "@com_github_mjbots_mjlib//mjlib/micro:pool_ptr",
        "@com_github_mjbots_mjlib//mjlib/micro:static_ptr",
        "@com_github_mjbots_mjlib//mjlib/micro:static_vector",
        "@eigen",
    ],
    copts = COPTS,
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
