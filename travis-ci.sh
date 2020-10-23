#!/bin/sh

set -ev

./tools/bazel test --config=host //:host
./tools/bazel build //:target
./tools/bazel build --config=pi -c opt //:pi3hat_tools
