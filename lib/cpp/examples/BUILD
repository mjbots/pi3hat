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

EXAMPLES = [
    'simple',
    'diagnostic_protocol',
    'multiple_cycle',
    'simple_teleop',
    'nlateral_teleop',
    'timeout_debug',
]

filegroup(
    name = "src",
    srcs = [
        "Makefile",
        "pi3hat_moteus_transport_register.cc",
    ]
)


[cc_binary(
    name = x,
    srcs = [
        "@moteus//lib/cpp/examples:" + x + ".cc",
        "pi3hat_moteus_transport_register.cc",
    ],
    copts = [
        "-Iexternal/moteus/lib/cpp/mjbots/moteus",
    ],
    deps = [
        "//lib/cpp/mjbots/pi3hat:transport",
        "@org_llvm_libcxx//:libcxx",
    ],
)
 for x in EXAMPLES
]
