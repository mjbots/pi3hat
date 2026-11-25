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
        commit = "97bf890db679505a14dfe547a5e77bb2bd05dc90",
        sha256 = "dab8ba2506f3cb950329934b7456c0104c844e4adcc45e45e7452733bf7be4e8",
        build_file = Label("//tools/workspace/pybind11:package.BUILD"),
    )
