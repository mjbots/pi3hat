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

config_setting(
    name = "armeabihf",
    values = {"cpu" : "armeabihf"},
)

config_setting(
    name = "aarch64",
    values = {"cpu" : "aarch64"},
)

# Some distros, like Ubuntu, only have libbcm_host.so without a .0,
# however the official raspberry pi package expects the aarch64
# version to be libbcm_host.so.0.
#
# Make things work on both of them by manually patching the soname
# before linking.  This will result in any libraries that link against
# this only requiring "libbcm_host.so".
genrule(
    name = "soless_bcm_host",
    outs = ["libbcm_host.so"],
    srcs = ["aarch64/usr/lib/aarch64-linux-gnu/libbcm_host.so"],
    cmd = "cp $< $@ && patchelf --set-soname libbcm_host.so $@",
)


cc_library(
    name = "bcm_host",
    hdrs = select({
        ":armeabihf" : [
            "hardfp/opt/vc/include/bcm_host.h",
        ] + glob([
            "hardfp/opt/vc/include/interface/**/*.h",
            "hardfp/opt/vc/include/vcinclude/**/*.h",
        ]),
        ":aarch64" : [] + glob([
            "aarch64/usr/include/**/*.h",
        ]),
        "//conditions:default" : [],
    }),
    srcs = select({
        ":armeabihf" : ["hardfp/opt/vc/lib/libbcm_host.so"],
        ":aarch64" : [":soless_bcm_host"],
    }),
    includes = select({
        ":armeabihf" : [
            "hardfp/opt/vc/include",
        ],
        "aarch64" : [
            "aarch64/usr/include",
        ],
    }),
)

cc_library(
    name = "raspberrypi-firmware",
    hdrs = glob(["hardfp/opt/vc/include/**/*.h"]),
    srcs = glob(["hardfp/opt/vc/lib/*.so"]),
    includes = [
        "hardfp/opt/vc/include",
        "hardfp/opt/vc/include/interface/vcos",
        "hardfp/opt/vc/include/interface/mmal",
    ],
)
