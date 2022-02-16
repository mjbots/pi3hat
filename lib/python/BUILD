# -*- python -*-

# Copyright 2020-2021 Josh Pieper, jjp@pobox.com.
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

load("@rules_pkg//:pkg.bzl", "pkg_tar")
load("//tools/workspace:template_file.bzl", "template_file")

VERSION="0.3.18"

PYTHON_SHORT_VERSIONS = ['37', '39']
ABITAG = {
    '37': 'abi3',
    '39': 'cp39',
}


# It would be nice to pull this from the python source.  As it is, the
# below genrule will fail if they don't match, so at least there is
# some verification.
pkg_tar(
    name = "wheel_tar",
    extension = "tar",
    package_dir = "moteus_pi3hat",
    srcs = [
        "//lib/python/moteus_pi3hat:_pi3hat_router.so",
        "//lib/python/moteus_pi3hat",
    ],
)


[
    template_file(
        name = "setup_{pyshortver}.py".format(pyshortver=pyshortver),
        src = "setup.py.TPL",
        substitutions = {
            "{{VERSION}}" : '"{}"'.format(VERSION),
            "{{PYTHON_SHORT_VERSION}}" : '"{}"'.format(pyshortver),
        },
    )
    for pyshortver in PYTHON_SHORT_VERSIONS
]


[genrule(
    name = "bdist_wheel_{pyshortver}".format(pyshortver=pyshortver),
    srcs = ["setup_{pyshortver}.py".format(pyshortver=pyshortver),
            ":wheel_tar", "README.md"],
    outs = [
        "moteus_pi3hat-{ver}-cp{pyshortver}-{abitag}-linux_armv7l.whl".format(
            ver=VERSION, pyshortver=pyshortver, abitag=ABITAG[pyshortver]),
    ],
    cmd = (
        "OUTDIR=$$(dirname $(location moteus_pi3hat-{ver}-cp{pyshortver}-{abitag}-linux_armv7l.whl)) && " +
        "BUILDDIR=$$OUTDIR/build && " +
        "mkdir -p $$BUILDDIR && " +
        "tar xf $(location wheel_tar) -C $$BUILDDIR && " +
        "cp $(location setup_{pyshortver}.py) $(location README.md) $$BUILDDIR && " +
        "cd $$BUILDDIR && " +
        "python3.9 setup_{pyshortver}.py bdist_wheel " +
        "        --plat-name linux_armv7l " +
        "        --py-limited-api cp{pyshortver} " +
        "        --dist-dir .. " +
        "        1>/dev/null &&" +

        # In case the file doesn't change, we rename to a different
        # temporary name.

        "mv ../moteus_pi3hat-{ver}-cp{pyshortver}-abi3-linux_armv7l.whl ../a.whl && " +
        "mv ../a.whl ../moteus_pi3hat-{ver}-cp{pyshortver}-{abitag}-linux_armv7l.whl " +
        ""
    ).format(ver=VERSION,
             pyshortver=pyshortver,
             abitag=ABITAG[pyshortver],
    ),
 )
 for pyshortver in PYTHON_SHORT_VERSIONS
]

filegroup(
    name="bdist_wheel",
    srcs = select({
        "//:python37" : [":bdist_wheel_37"],
        "//:python39" : [":bdist_wheel_39"],
    }),
)
