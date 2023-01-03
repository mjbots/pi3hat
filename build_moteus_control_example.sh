#!/bin/bash

g++ -O2 -g -Wall --std=c++11 \
    -mcpu=cortex-a53 \
    -Wno-psabi \
    -I lib/cpp -I /opt/vc/include -L /opt/vc/lib \
    -o moteus_control_example \
    lib/cpp/mjbots/moteus/moteus_control_example.cc \
    lib/cpp/mjbots/pi3hat/pi3hat.cc \
    -lbcm_host \
    -lpthread
