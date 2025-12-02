#!/usr/bin/env python3
import Adafruit_BBIO.GPIO as GPIO

test_pin = "P2_02"
GPIO.setup(test_pin, GPIO.OUT)
GPIO.output(test_pin, GPIO.HIGH)
GPIO.output(test_pin, GPIO.LOW)
print(f"{test_pin} works")
