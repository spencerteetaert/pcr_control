#!/usr/bin/env bash

sudo ip link set down can0

# Set up motor serial connections 
sudo ip link set can0 type can bitrate 1000000 sample-point 0.867

sudo ip link set up can0

# Set up Aurora permissions
sudo chmod a+rw /dev/ttyUSB0