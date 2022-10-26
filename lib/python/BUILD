# -*- python -*-

# Copyright 2020-2022 Josh Pieper, jjp@pobox.com.
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

VERSION="0.3.26"

PYTHON_SHORT_VERSIONS = ['37', '39']
ARCHITECTURES = ['armv7l', 'aarch64']
ABITAG = {
    '37': 'abi3',
    '39': 'cp39',
}
PTAG = {
    '37_armv7l': 'linux',
    '37_aarch64': 'manylinux_2_17',
    '39_armv7l': 'manylinux_2_17',
    '39_aarch64': 'manylinux_2_17',
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
    name = "bdist_wheel_{pyshortver}_{architecture}".format(
        pyshortver=pyshortver,
        architecture=architecture),
    srcs = ["setup_{pyshortver}.py".format(pyshortver=pyshortver),
            ":wheel_tar", "README.md"],
    outs = [
        "moteus_pi3hat-{ver}-cp{pyshortver}-{abitag}-{ptag}_{architecture}.whl".format(
            ver=VERSION,
            pyshortver=pyshortver,
            abitag=ABITAG[pyshortver],
            ptag=PTAG[pyshortver + '_' + architecture],
            architecture=architecture),
    ],
    cmd = (
        "OUTDIR=$$(dirname $(location moteus_pi3hat-{ver}-cp{pyshortver}-{abitag}-{ptag}_{architecture}.whl)) && " +
        "BUILDDIR=$$OUTDIR/build && " +
        "mkdir -p $$BUILDDIR && " +
        "tar xf $(location wheel_tar) -C $$BUILDDIR && " +
        "cp $(location setup_{pyshortver}.py) $(location README.md) $$BUILDDIR && " +
        "cd $$BUILDDIR && " +
        "python3.9 setup_{pyshortver}.py bdist_wheel " +
        "        --plat-name {ptag}_{architecture} " +
        "        --py-limited-api cp{pyshortver} " +
        "        --dist-dir .. " +
        "        1>/dev/null &&" +

        # In case the file doesn't change, we rename to a different
        # temporary name.

        "mv ../moteus_pi3hat-{ver}-cp{pyshortver}-abi3-{ptag}_{architecture}.whl ../a.whl && " +
        "mv ../a.whl ../moteus_pi3hat-{ver}-cp{pyshortver}-{abitag}-{ptag}_{architecture}.whl " +
        ""
    ).format(ver=VERSION,
             pyshortver=pyshortver,
             abitag=ABITAG[pyshortver],
             ptag=PTAG[pyshortver + '_' + architecture],
             architecture=architecture,
    ),
 )
 for pyshortver in PYTHON_SHORT_VERSIONS
 for architecture in ARCHITECTURES
]

filegroup(
    name="bdist_wheel",
    srcs = select({
        "//:python37_armeabihf" : [":bdist_wheel_37_armv7l"],
        "//:python37_aarch64" : [":bdist_wheel_37_aarch64"],
        "//:python39_armeabihf" : [":bdist_wheel_39_armv7l"],
        "//:python39_aarch64" : [":bdist_wheel_39_aarch64"],
    }),
)
