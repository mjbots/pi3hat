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

import setuptools
import pathlib

# Hacky monkeypatch to allow building cp313 wheels from Python 3.12.
# The wheel package validates tags against sys_tags(), which only
# includes tags for the running interpreter. This adds cp313 tags to
# allow cross-version builds.  Note: wheel uses its own vendored copy
# of packaging, so we must patch that.
from wheel.vendored.packaging import tags as _wheel_tags
_original_sys_tags = _wheel_tags.sys_tags
def _patched_sys_tags():
    for tag in _original_sys_tags():
        yield tag
    # Add cp313 abi3 tags for building 3.13 wheels from 3.12
    for plat in ['manylinux_2_17_aarch64', 'manylinux_2_17_armv7l', 'linux_armv7l']:
        yield _wheel_tags.Tag('cp313', 'abi3', plat)
_wheel_tags.sys_tags = _patched_sys_tags

here = pathlib.Path(__file__).parent.resolve()

# Get the long description from the README file
long_description = (here / 'README.md').read_text(encoding='utf-8')


class BinaryDistribution(setuptools.dist.Distribution):
    def has_ext_modules(foo):
        return True


setuptools.setup(
    name = 'moteus_pi3hat',
    version = {{VERSION}},
    description = 'moteus brushless controller library and tools',
    long_description = long_description,
    long_description_content_type = 'text/markdown',
    url = 'https://github.com/mjbots/pi3hat',
    author = 'mjbots Robotic Systems',
    author_email = 'info@mjbots.com',
    classifiers = [
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
    ],
    keywords='moteus, pi3hat',
    packages = [
        'moteus_pi3hat',
    ],
    entry_points = {
        'moteus.transports2': [ 'pi3hat=moteus_pi3hat:Pi3HatFactory' ],
    },
    python_requires = ">=3.7",
    install_requires = [
        'moteus>=0.3.91',
    ],
    include_package_data = True,
    package_data = {
        'moteus_pi3hat' : ['_pi3hat_device.so'],
    },
    distclass = BinaryDistribution,
)
