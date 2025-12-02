#!/usr/bin/env python3
import time, random, os
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import board, busio
import ht16k33 as HT16K33

# -------------------------
# Config
# -------------------------
BPM_OPTIONS = [60, 90, 120]
BPM_INDEX_DEFAULT = 1

# LEDs mapped to GPIOs
LEDS = [
    "P2_02",  # GPIO59
    "P2_04",  # GPIO58
    "P2_06",  # GPIO57
    "P2_08",  # GPIO60
    "P2_18",  # GPIO47
    "P2_20",  # GPIO64
    "P2_22",  # GPIO46
    "P2_24",  # GPIO44
]

JOYSTICK = {
    "left":  "P1_29",
    "right": "P1_28",
    "press": "P1_31",
}

# Buzzers
BUZZER_A = "P2_1"  # GPIO50
BUZZER_B = "P2_3"  # GPIO23

I2C_ADDR_A = 0x70
I2C_ADDR_B = 0x71

POINTS_PER_HIT = 5
ON_BEAT_TOLERANCE = 0.08
GAME_SECONDS = 60

# -------------------------
# Pin configuration
# -------------------------
def configure_pins():
    # LEDs as GPIO
    for pin in LEDS:
        os.system(f"config-pin {pin} gpio")

    # Joystick as GPIO
    for pin in JOYSTICK.values():
        os.system(f"config-pin {pin} gpio")

    # Buzzers as PWM
    os.system(f"config-pin {BUZZER_A} pwm")
    os.system(f"config-pin {BUZZER_B} pwm")

# -------------------------
# Setup
# -------------------------
def setup_gpio_and_displays():
    for pin in LEDS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    for _, pin in JOYSTICK.items():
        GPIO.setup(pin, GPIO.IN)
    PWM.start(BUZZER_A, 0.0, 1200.0)
    PWM.start(BUZZER_B, 0.0, 400.0)
    i2c = busio.I2C(board.SCL, board.SDA)
    dispA = segments.Seg7x4(i2c, address=I2C_ADDR_A)
    dispB = segments.Seg7x4(i2c, address=I2C_ADDR_B)
    dispA.brightness = 12
    dispB.brightness = 12
    dispA.fill(0)
    dispB.fill(0)
    return dispA, dispB

def cleanup():
    PWM.stop(BUZZER_A)
    PWM.stop(BUZZER_B)
    PWM.cleanup()
    GPIO.cleanup()

# -------------------------
# LED control
# -------------------------
def all_leds_off():
    for pin in LEDS:
        GPIO.output(pin, GPIO.LOW)

def set_led(index, on=True):
    GPIO.output(LEDS[index], GPIO.HIGH if on else GPIO.LOW)

def show_column_step(col, row):
    all_leds_off()
    index = row + (4 if col == 1 else 0)
    set_led(index, True)

# -------------------------
# Buzzers
# -------------------------
def tick_bpm():
    PWM.set_frequency(BUZZER_A, 1200.0)
    PWM.set_duty_cycle(BUZZER_A, 45.0)
    time.sleep(0.03)
    PWM.set_duty_cycle(BUZZER_A, 0.0)

def error_buzz():
    PWM.set_frequency(BUZZER_B, 400.0)
    PWM.set_duty_cycle(BUZZER_B, 60.0)
    time.sleep(0.15)
    PWM.set_duty_cycle(BUZZER_B, 0.0)

# -------------------------
# Game loop
# -------------------------
def run_game(dispA, dispB):
    bpm = BPM_OPTIONS[BPM_INDEX_DEFAULT]
    score = 0
    remaining = GAME_SECONDS
    end_time = time.time() + GAME_SECONDS
    beat_int = 60.0 / bpm
    next_beat = time.time() + 0.3
    col = random.randrange(0, 2)
    row = -1

    while True:
        now = time.time()
        remaining = max(0, int(end_time - now))
        dispA.print(f"{remaining:04d}")
        dispB.print(f"{score:04d}")

        if remaining <= 0:
            break

        if now >= next_beat:
            tick_bpm()
            row += 1
            if row == 0:
                col = random.randrange(0, 2)
            if row < 4:
                show_column_step(col, row)

            if row == 3:
                target = next_beat
                deadline = target + ON_BEAT_TOLERANCE
                hit = False
                while time.time() <= deadline:
                    if GPIO.input(JOYSTICK["left"]) and col == 0:
                        score += POINTS_PER_HIT
                        hit = True
                        break
                    elif GPIO.input(JOYSTICK["right"]) and col == 1:
                        score += POINTS_PER_HIT
                        hit = True
                        break
                    elif GPIO.input(JOYSTICK["left"]) or GPIO.input(JOYSTICK["right"]):
                        error_buzz()
                        hit = True
                        break
                    time.sleep(0.005)
                if not hit:
                    error_buzz()
                all_leds_off()
                row = -1
            next_beat += beat_int
        time.sleep(0.002)

    # End sequence
    all_leds_off()
    dispA.print("0000")
    time.sleep(2)
    dispB.print(f"{score:04d}")
    time.sleep(5)
    dispA.fill(0)
    dispB.fill(0)

# -------------------------
# Main
# -------------------------
def main():
    configure_pins()  # <-- NEW: set pinmux before anything else
    dispA, dispB = setup_gpio_and_displays()
    try:
        while True:
            print("Press joystick to start...")
            while not GPIO.input(JOYSTICK["press"]):
                time.sleep(0.05)
            run_game(dispA, dispB)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()

if __name__ == "__main__":
    main()