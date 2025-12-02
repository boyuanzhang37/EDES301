#!/usr/bin/env python3
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
