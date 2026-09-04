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

# Rust client library and moteus transport for the mjbots pi3hat.
# Mirrors the structure of @moteus//lib/rust/moteus.

load("@rules_rust//rust:defs.bzl", "rust_clippy", "rust_doc", "rust_doc_test", "rust_library", "rust_test")

package(default_visibility = ["//visibility:public"])

# The example sources are part of the cargo package (so `cargo run
# --example` works and they ship in the published crate), but the
# rust_binary targets that build them live in //lib/rust.
exports_files(glob(["examples/*.rs"]))

# Default (blocking) library.  moteus-pi3hat depends on moteus with
# default-features disabled; we link the @moteus//...:moteus variant,
# whose extra serialport feature is harmless for the pi3hat crate.
rust_library(
    name = "moteus_pi3hat",
    srcs = glob(["src/**/*.rs"]),
    crate_name = "moteus_pi3hat",
    edition = "2021",
    rustc_flags = [
        "-D", "warnings",
    ],
    deps = [
        "@crate_index//:libc",
        "@moteus//lib/rust/moteus:moteus",
    ],
)

# Library with the async (tokio) transport enabled.
rust_library(
    name = "moteus_pi3hat_tokio",
    srcs = glob(["src/**/*.rs"]),
    crate_name = "moteus_pi3hat",
    edition = "2021",
    rustc_flags = [
        "--cfg=feature=\"tokio\"",
        "-D", "warnings",
    ],
    deps = [
        "@crate_index//:libc",
        "@crate_index//:tokio",
        "@moteus//lib/rust/moteus:moteus_tokio",
    ],
)

# Identical to moteus_pi3hat_tokio, but built against the moteus crate's
# `moteus_examples` variant.  The auto-generated moteus-example wrappers
# (//lib/rust/moteus_examples) link both this and moteus_examples, so
# both must reference the *same* `moteus` rlib or rustc sees two crates
# named `moteus`.
rust_library(
    name = "moteus_pi3hat_examples",
    srcs = glob(["src/**/*.rs"]),
    crate_name = "moteus_pi3hat",
    edition = "2021",
    rustc_flags = [
        "--cfg=feature=\"tokio\"",
        "-D", "warnings",
    ],
    deps = [
        "@crate_index//:libc",
        "@crate_index//:tokio",
        "@moteus//lib/rust/moteus:moteus_examples",
    ],
)

rust_test(
    name = "moteus-pi3hat-test",
    crate = ":moteus_pi3hat",
    edition = "2021",
    size = "small",
)

# rust_test does not inherit the crate's rustc_flags, so the feature
# cfgs must be repeated here or the tokio-gated tests are compiled out.
rust_test(
    name = "moteus-pi3hat-test-tokio",
    crate = ":moteus_pi3hat_tokio",
    crate_features = [
        "tokio",
    ],
    edition = "2021",
    size = "small",
)

rust_clippy(
    name = "moteus-pi3hat-clippy",
    deps = [":moteus_pi3hat"],
)

rust_clippy(
    name = "moteus-pi3hat-tokio-clippy",
    deps = [":moteus_pi3hat_tokio"],
)

rust_doc_test(
    name = "moteus-pi3hat-doc-test",
    crate = ":moteus_pi3hat",
    size = "small",
)

# rust_doc_test does not inherit rustc_flags, so the feature cfgs must
# be repeated or rustdoc silently skips the feature-gated doctests.
rust_doc_test(
    name = "moteus-pi3hat-doc-test-tokio",
    crate = ":moteus_pi3hat_tokio",
    crate_features = [
        "tokio",
    ],
    size = "small",
)

rust_doc(
    name = "moteus-pi3hat-doc",
    crate = ":moteus_pi3hat_tokio",
    crate_features = [
        "tokio",
    ],
    rustdoc_flags = ["-D", "warnings"],
)
