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

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "src",
    srcs = [
        "pi3hat.h",
        "pi3hat.cc",
        "pi3hat_moteus_transport.h",
        "realtime.h",
    ],
)

cc_library(
    name = "headers",
    hdrs = ["pi3hat.h"],
    include_prefix = "mjbots/pi3hat",
)

cc_library(
    name = "libpi3hat",
    hdrs = ["pi3hat.h"],
    srcs = ["pi3hat.cc"],
    deps = [":headers", "@raspberrypi-firmware//:bcm_host"],
)

cc_binary(
    name = "pi3hat_tool",
    srcs = ["pi3hat_tool.cc"],
    deps = [
        ":libpi3hat",
        "@org_llvm_libcxx//:libcxx",
    ],
)

filegroup(
    name = "pi3hat",
    srcs = [
        ":libpi3hat",
        ":pi3hat_tool",
    ],
)

cc_library(
    name = "realtime",
    hdrs = ["realtime.h"],
)

cc_library(
    name = "transport",
    hdrs = [
        "pi3hat_moteus_transport.h",
    ],
    deps = [
        ":realtime",
        ":libpi3hat",
        "@moteus//lib/cpp/mjbots/moteus:moteus",
    ],
    includes = ["."],
)
