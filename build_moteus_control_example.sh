#!/bin/bash

cd lib/cpp

g++ -O2 -Wall --std=c++11 \
    -lpthread \
    -Wno-psabi \
    -I . -I /opt/vc/include -L /opt/vc/lib -l bcm_host \
    -o moteus_control_example \
    mjbots/moteus/moteus_control_example.cc \
    mjbots/pi3hat/pi3hat.cc
