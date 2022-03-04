# -*- python -*-

# Copyright 2018-2022 Josh Pieper, jjp@pobox.com.
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

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _impl(repository_ctx):
    FWVER = '1.20220120'

    repository_ctx.download_and_extract(
        url = [
            "https://github.com/raspberrypi/firmware/archive/{}.tar.gz".format(FWVER),
        ],
        sha256 = "67c49b6f2fbf4ee612536b3fc24e44ab3fa9584c78224865699f1cbc1b8eea3c",
        stripPrefix = "firmware-{}".format(FWVER),
    )

    repository_ctx.extract(
        archive = repository_ctx.attr.aarch64_sysroot,
        output = 'aarch64',
    )

    repository_ctx.template(
        "BUILD",
        repository_ctx.attr.build_file,
        substitutions = {},
    )


_raspberrypi_firmware_repository = repository_rule(
    implementation = _impl,
    attrs = {
        "build_file" : attr.label(allow_single_file=True),
        "aarch64_sysroot" : attr.label(allow_single_file=True),
    },
)

def raspberrypi_firmware_repository(name):
    _raspberrypi_firmware_repository(
        name = name,
        build_file = Label("//tools/workspace/raspberrypi-firmware:package.BUILD"),
        aarch64_sysroot = Label("//tools/workspace/raspberrypi-firmware:2022-03-04-firmware-aarch64-linux-gnu.tar.xz"),
    )
