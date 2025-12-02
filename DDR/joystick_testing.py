#!/usr/bin/env python3
"""
--------------------------------------------------------------------------

License:   
Copyright 2025 Boyuan Zhang

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------
joystick testing, this only includes the Y axis but the same logic goes for X.
"""
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.GPIO as GPIO
import time

ADC_PIN = "P1_19"    # Your joystick Y-axis analog pin
SW_PIN  = "P2_10"    # Your joystick press button pin

LEFT_THRESH  = 0.35
RIGHT_THRESH = 0.65

def read_axis():
    """Return raw ADC 0–1.0 and direction -1/0/+1."""
    val = ADC.read(ADC_PIN)
    if val is None:
        return None, 0

    if val < LEFT_THRESH:
        direction = -1
    elif val > RIGHT_THRESH:
        direction = 1
    else:
        direction = 0

    return val, direction

def main():
    ADC.setup()
    GPIO.setup(SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print("\n=== Joystick Test ===")
    print("Move the stick and press the button.\n")
    print("Prints 20 times per second.\n")
    print("Columns:")
    print(" RAW     DIR     PRESSED\n")

    try:
        while True:
            raw, direction = read_axis()
            pressed = GPIO.input(SW_PIN) == 0

            if raw is None:
                print("ADC ERROR", end="\r")
                time.sleep(0.05)
                continue

            # Format nice output
            print(f" {raw:0.3f}     {direction:+d}        {pressed}", end="\r")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nExiting.")

if __name__ == "__main__":
    main()
