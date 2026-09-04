#!/bin/sh

set -ev

./tools/bazel test --config=host //:host
./tools/bazel build //:target
./tools/bazel build --config=pi --define PYTHON=3.9 -c opt //:pi3hat_tools

# Compile the rust crate in its *published* configuration.  Bazel
# builds it against the moteus git pin (which may be ahead of the
# crates.io release) with the serialport feature enabled; the crate as
# published resolves moteus from crates.io with default features off.
# Without this check, a change that quietly depends on unreleased
# moteus API stays green until `cargo publish` fails on release day.
(cd lib/rust && cargo publish --dry-run --locked -p moteus-pi3hat)
