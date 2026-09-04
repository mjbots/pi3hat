# -*- python -*-

# Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

# The full set of moteus Rust examples, rebuilt to run over the pi3hat
# transport.  One binary per entry in moteus's exported
# MOTEUS_RUST_EXAMPLES list is generated automatically: a thin wrapper
# (from wrapper.rs.in) registers the pi3hat transport and then calls the
# shared example body in moteus::examples.  Adding an example upstream in
# moteus picks it up here with no changes.

load("@rules_rust//rust:defs.bzl", "rust_binary")
load("@moteus//lib/rust:examples.bzl", "MOTEUS_RUST_EXAMPLES")

package(default_visibility = ["//visibility:public"])

# Generate one wrapper source per moteus example by substituting the
# example name into the template.
[
    genrule(
        name = "gen_" + name,
        srcs = ["wrapper.rs.in"],
        outs = [name + ".rs"],
        cmd = "sed 's/@EXAMPLE@/" + name + "/g' $< > $@",
    )
    for name in MOTEUS_RUST_EXAMPLES
]

# Build each wrapper.  Both deps must reference the same moteus
# `moteus_examples` rlib (see moteus_pi3hat_examples) so rustc sees a
# single `moteus` crate.
[
    rust_binary(
        name = name,
        srcs = [":" + name + ".rs"],
        edition = "2021",
        deps = [
            "//lib/rust/moteus-pi3hat:moteus_pi3hat_examples",
            "@moteus//lib/rust/moteus:moteus_examples",
        ],
    )
    for name in MOTEUS_RUST_EXAMPLES
]
