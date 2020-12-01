# -*- python -*-

# Copyright 2018 Josh Pieper, jjp@pobox.com.
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

load("//tools/workspace/bazel_deps:repository.bzl", "bazel_deps_repository")
load("//tools/workspace/pybind11:repository.bzl", "pybind11_repository")
load("//tools/workspace/rules_mbed:repository.bzl", "rules_mbed_repository")
load("//tools/workspace/mjlib:repository.bzl", "mjlib_repository")
load("//tools/workspace/moteus:repository.bzl", "moteus_repository")
load("//tools/workspace/raspberrypi-firmware:repository.bzl", "raspberrypi_firmware_repository")
load("//tools/workspace/rpi_bazel:repository.bzl", "rpi_bazel_repository")
load("//tools/workspace/rules_pkg:repository.bzl", "rules_pkg_repository")

def add_default_repositories(excludes = []):
    if "bazel_deps" not in excludes:
        bazel_deps_repository(name = "com_github_mjbots_bazel_deps")
    if "pybind11" not in excludes:
        pybind11_repository(name = "com_github_pybind_pybind11")
    if "com_github_mjbots_rules_bazel" not in excludes:
        rules_mbed_repository()
    if "mjlib" not in excludes:
        mjlib_repository(name = "com_github_mjbots_mjlib")
    if "moteus" not in excludes:
        moteus_repository(name = "moteus")
    if "raspberrypi-firmware" not in excludes:
        raspberrypi_firmware_repository(name = "raspberrypi-firmware")
    if "rpi_bazel" not in excludes:
        rpi_bazel_repository(name = "rpi_bazel")
    if "rules_pkg" not in excludes:
        rules_pkg_repository(name = "rules_pkg")
