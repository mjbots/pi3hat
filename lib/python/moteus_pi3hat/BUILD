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

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "_pi3hat_router.so",
    srcs = [
        "_pi3hat_router.cc",
    ],
    deps = [
        "//lib/cpp/mjbots/pi3hat:libpi3hat",
        "//lib/cpp/mjbots/pi3hat:realtime",
        "@org_llvm_libcxx//:libcxx",
    ] + select({
        "//conditions:default" : [],
        "//:python37" : ["@com_github_pybind_pybind11//:pybind11_37"],
        "//:python39" : ["@com_github_pybind_pybind11//:pybind11_39"],
        "//:python310" : ["@com_github_pybind_pybind11//:pybind11_310"],
        "//:python311" : ["@com_github_pybind_pybind11//:pybind11_311"],
    }),
    linkshared = True,
    copts = ["-Ilib/cpp"],
)

py_library(
    name = "moteus_pi3hat",
    srcs = [
        "__init__.py",
        "pi3hat_router.py",
    ],
)
