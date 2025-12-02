#!/bin/bash

# Set pin mode to GPIO
config-pin "P2_02" gpio

# Export GPIO59
echo 59 > /sys/class/gpio/export

# Set direction to output
echo out > /sys/class/gpio/gpio59/direction

# Turn LED ON
echo 1 > /sys/class/gpio/gpio59/value

# Wait a moment
sleep 1

# Turn LED OFF
echo 0 > /sys/class/gpio/gpio59/value

# Unexport when done
echo 59 > /sys/class/gpio/unexport