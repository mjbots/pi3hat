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

load("//tools/workspace:github_archive.bzl", "github_archive")


def pybind11_repository(name):
    github_archive(
        name = name,
        repo = "pybind/pybind11",
        commit = "6d22dba82f1789f11a8eb2c2debbcbd4d2d8a969",
        sha256 = "ecbce06af56a59f0c6bac5241c655fd67d8a107c9aa46249e90ea4ed0fb36c98",
        build_file = Label("//tools/workspace/pybind11:package.BUILD"),
    )
