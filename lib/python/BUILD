# -*- python -*-

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

package(default_visibility = ["//visibility:public"])

# It would be nice to pull this from the python source.  As it is, the
# below genrule will fail if they don't match, so at least there is
# some verification.
pkg_tar(
    name = "wheel_tar",
    extension = "tar",
    package_dir = "moteus",
    srcs = [
        "//lib/python/moteus:_pi3hat_router.so",
        "//lib/python/moteus",
        "@moteus//lib/python/moteus:libfiles",
    ],
)

VER="0.0.1"

genrule(
    name = "bdist_wheel",
    srcs = ["setup.py", "README.md", ":wheel_tar"],
    outs = [
        "moteus_pi3hat-{ver}-cp37-abi3-linux_armv7l.whl".format(ver=VER),
    ],
    cmd = (
        "OUTDIR=$$(dirname $(location moteus_pi3hat-{ver}-cp37-abi3-linux_armv7l.whl)) && " +
        "BUILDDIR=$$OUTDIR/build && " +
        "mkdir -p $$BUILDDIR && " +
        "tar xf $(location wheel_tar) -C $$BUILDDIR && " +
        "cp $(location setup.py) $(location README.md) $$BUILDDIR && " +
        "cd $$BUILDDIR && " +
        "python3 setup.py bdist_wheel " +
        "        --plat-name linux_armv7l " +
        "        --py-limited-api cp37 " +
        "        --dist-dir .. " +
        "        1>/dev/null" +
        ""
    ).format(ver=VER),
)
