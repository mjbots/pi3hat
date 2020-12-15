# -*- python -*-

# Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
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

def moteus_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/moteus",
        commit = "b94b1e64a23ef0919c8b7f0ffa8b9ecd47b2620b",
        sha256 = "ea496c8fc0fa18efce62ea0199ebc7b2b55011bba78ee79d9ee6da9198975497",
    )
