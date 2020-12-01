# -*- python -*-

# Copyright 2019 Josh Pieper, jjp@pobox.com.
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
        commit = "028812ae7eee307dca5f8f69d467af7b92cc41c8",
        sha256 = "d3f0bfb14b59838b6d42b2b53876007b5043f51c17cb189ae8fa2e2c88ada4f5",
        build_file = Label("//tools/workspace/pybind11:package.BUILD"),
    )
