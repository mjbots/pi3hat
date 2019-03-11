#!/bin/bash

openocd \
    -f /usr/share/openocd/scripts/interface/stlink-v2.cfg \
    -f /usr/share/openocd/scripts/target/stm32f0x_stlink.cfg \
    -c init -c "reset_config none separate; reset init"
