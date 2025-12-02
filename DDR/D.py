#!/usr/bin/env python3
"""
Simplified DDR-style game for PocketBeagle
- Single HT16K33 display (shows BPM before start, countdown timer during game)
- 4x2 LED grid for DDR arrows
- Joystick: VRy -> P1_19, SW -> P2_10
- Buzzers: P2_01 (tempo), P2_03 (error)
"""

import time, threading, random
from collections import deque
import os
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import smbus

# --------------------------
# Pin config
# --------------------------
LED_PINS = ["P2_02","P2_04","P2_06","P2_08","P2_18","P2_20","P2_22","P2_24"]
JOY_SW_PIN = "P2_10"
VR_ADC_CHANNEL = 0  # AIN0 / P1_19
BUZZER_A_PIN = "P2_01"
BUZZER_B_PIN = "P2_03"

HT16K33_ADDR = 0x70
HT16K33_BUS = 1

# --------------------------
# Game constants
# --------------------------
SLOW_BPM = 90
NORMAL_BPM = 120
FAST_BPM = 150
BPM_OPTIONS = [SLOW_BPM, NORMAL_BPM, FAST_BPM]
DEFAULT_BPM_IDX = 1

GAME_DURATION_SECONDS = 120
TONE_FREQ = 2000
ERROR_FREQ = 600
PWM_DUTY = 50

VR_LEFT_THRESH = 1400
VR_RIGHT_THRESH = 2600
AIN_BASE_PATH = "/sys/bus/iio/devices/iio:device0"

# --------------------------
# Automatic pin configuration
# --------------------------
def configure_pins():
    for pin in LED_PINS:
        os.system(f"config-pin {pin} gpio")
    os.system(f"config-pin {JOY_SW_PIN} gpio")
    os.system(f"config-pin {BUZZER_A_PIN} pwm")
    os.system(f"config-pin {BUZZER_B_PIN} pwm")
    os.system("config-pin P2_09 i2c")
    os.system("config-pin P2_11 i2c")
    print("Pins configured.")

# --------------------------
# HT16K33 7-segment driver
# --------------------------
class SevenSegHT16K33:
    DIGITS = [0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F]
    def __init__(self, address, busnum=1, brightness=15):
        self.addr = address
        self.bus = smbus.SMBus(busnum)
        self.bus.write_byte(self.addr, 0x21)  # system on
        self.bus.write_byte(self.addr, 0x81)  # display on, no blinking
        self.bus.write_byte(self.addr, 0xE0 | (brightness & 0x0F))
        self.buf = [0x00]*16
        self.show_blank()
    def _flush(self): self.bus.write_i2c_block_data(self.addr, 0x00, self.buf)
    def show_blank(self):
        self.buf = [0x00]*16
        self._flush()
    def show_number(self, number, leading_zero=False):
        n = int(number) % 10000
        digits = [0,0,0,0]
        for i in range(4):
            digits[3-i] = n % 10
            n //= 10
        self.buf[0] = self.DIGITS[digits[0]] if leading_zero or digits[0]!=0 else 0x00
        self.buf[2] = self.DIGITS[digits[1]] if leading_zero or (digits[0]!=0 or digits[1]!=0) else 0x00
        self.buf[4] = self.DIGITS[digits[2]] if leading_zero or any(d!=0 for d in digits[:3]) else 0x00
        self.buf[6] = self.DIGITS[digits[3]]
        self._flush()

# --------------------------
# GPIO setup
# --------------------------
def setup_gpio():
    for p in LED_PINS:
        GPIO.setup(p, GPIO.OUT)
        GPIO.output(p, GPIO.LOW)
    GPIO.setup(JOY_SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BUZZER_A_PIN, GPIO.OUT)
    GPIO.setup(BUZZER_B_PIN, GPIO.OUT)
    GPIO.output(BUZZER_A_PIN, GPIO.LOW)
    GPIO.output(BUZZER_B_PIN, GPIO.LOW)

def led_index(col,row):
    return col*4 + row

def set_led(col,row,on):
    GPIO.output(LED_PINS[led_index(col,row)], GPIO.HIGH if on else GPIO.LOW)

def clear_all_leds():
    for p in LED_PINS:
        GPIO.output(p, GPIO.LOW)

# --------------------------
# PWM helpers
# --------------------------
def play_tone(pin, freq, duration):
    def _t():
        try:
            PWM.start(pin, PWM_DUTY, freq)
            time.sleep(duration)
        finally:
            try: PWM.stop(pin)
            except: pass
    t = threading.Thread(target=_t)
    t.daemon = True
    t.start()

def tempo_click(): play_tone(BUZZER_A_PIN, TONE_FREQ, 0.05)
def error_buzz(): play_tone(BUZZER_B_PIN, ERROR_FREQ, 0.14)

# --------------------------
# ADC read
# --------------------------
def read_adc_raw(channel=0):
    path = f"{AIN_BASE_PATH}/in_voltage{channel}_raw"
    try:
        with open(path,"r") as f:
            return int(f.read().strip())
    except Exception as e:
        raise RuntimeError(f"Failed to read ADC channel {channel}: {e}")

def joystick_from_analog():
    raw = read_adc_raw(VR_ADC_CHANNEL)
    left = raw < VR_LEFT_THRESH
    right = raw > VR_RIGHT_THRESH
    press = not GPIO.input(JOY_SW_PIN)
    return {"left": left, "right": right, "press": press}

# --------------------------
# DDR Game class
# --------------------------
class DDRGame:
    def __init__(self):
        configure_pins()
        setup_gpio()
        self.display = SevenSegHT16K33(HT16K33_ADDR, busnum=HT16K33_BUS)
        self.bpm_idx = DEFAULT_BPM_IDX
        self.bpm = BPM_OPTIONS[self.bpm_idx]
        self.running = False
        self.active_notes = deque()
        self.stop_event = threading.Event()
        self.show_bpm()

    def show_bpm(self):
        # Show BPM on display before game starts
        self.display.show_number(self.bpm, leading_zero=True)

    def cycle_bpm(self, direction):
        if direction == "left":
            self.bpm_idx = (self.bpm_idx - 1) % len(BPM_OPTIONS)
        elif direction == "right":
            self.bpm_idx = (self.bpm_idx + 1) % len(BPM_OPTIONS)
        self.bpm = BPM_OPTIONS[self.bpm_idx]
        self.show_bpm()

    def start_game(self):
        self.running = True
        start_time = time.time()
        end_time = start_time + GAME_DURATION_SECONDS
        beat_interval = 60.0 / self.bpm
        next_beat = start_time

        while time.time() < end_time and not self.stop_event.is_set():
            now = time.time()
            if now >= next_beat:
                tempo_click()
                self.advance_notes()
                next_beat += beat_interval

            # Handle joystick BPM changes before game starts
            js = joystick_from_analog()
            if not self.running and js["left"]:
                self.cycle_bpm("left")
            elif not self.running and js["right"]:
                self.cycle_bpm("right")

            # Update display with countdown
            if self.running:
                remaining = int(end_time - now)
                minutes = remaining // 60
                seconds = remaining % 60
                self.display.show_number(minutes*100 + seconds, leading_zero=True)

            time.sleep(0.01)

        self.end_sequence()

    def advance_notes(self):
        # Simple placeholder: light a random top-row LED per beat
        clear_all_leds()
        col = random.randint(0,1)
        set_led(col,0,True)

    def end_sequence(self):
        clear_all_leds()
        # Flash 00:00 three times
        for _ in range(3):
            self.display.show_number(0)
            time.sleep(0.5)
            self.display.show_blank()
            time.sleep(0.5)
        self.show_bpm()
        self.running = False

    def run_ui(self):
        print("Use joystick left/right to select BPM. Press to start.")
        while True:
            js = joystick_from_analog()
            if js["press"]:
                self.start_game()
            elif js["left"]:
                self.cycle_bpm("left")
            elif js["right"]:
                self.cycle_bpm("right")
            time.sleep(0.1)

# --------------------------
# Main
# --------------------------
def main():
    game = DDRGame()
    try:
        game.run_ui()
    finally:
        try:
            clear_all_leds()
            game.display.show_blank()
            GPIO.cleanup()
            try: PWM.stop(BUZZER_A_PIN); PWM.stop(BUZZER_B_PIN)
            except: pass
            try: PWM.cleanup()
            except: pass
        except: pass

if __name__ == "__main__":
    main()
