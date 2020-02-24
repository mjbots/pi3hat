#!/bin/bash

openocd \
    -f interface/stlink.cfg \
    -f target/stm32g4x.cfg \
    -c "init" \
    -c "reset_config none separate; program bazel-out/stm32g4-opt/bin/fw/pi3_hat.bin verify 0x8000000 verify reset exit 0x8000000"
