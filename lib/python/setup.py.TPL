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

import setuptools
import pathlib

here = pathlib.Path(__file__).parent.resolve()

# Get the long description from the README file
long_description = (here / 'README.md').read_text(encoding='utf-8')


class BinaryDistribution(setuptools.dist.Distribution):
    def has_ext_modules(foo):
        return True


setuptools.setup(
    name = 'moteus-pi3hat',
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
        'moteus.transports': [ 'pi3hat=moteus_pi3hat:Pi3HatFactory' ],
    },
    python_requires = ">=3.7",
    install_requires = [
        'moteus>=0.3.22',
    ],
    include_package_data = True,
    package_data = {
        'moteus_pi3hat' : ['_pi3hat_router.so'],
    },
    distclass = BinaryDistribution,
)
