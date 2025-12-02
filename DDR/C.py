#!/usr/bin/env python3
"""
PocketBeagle DDR-style game — 4x2 LED grid, joystick (VRy -> AIN0), push button on P2_10

- Automatic pin configuration included for GPIO, PWM, and I2C
- 4x2 LED grid: Left P2_02,P2_04,P2_06,P2_08; Right P2_18,P2_20,P2_22,P2_24
- VRy (analog) -> voltage divider -> P1_19
- SW (press) -> P2_10
- Buzzers: P2_15 (tempo), P2_16 (error)
- HT16K33 displays: Display A addr 0x70 on I2C1, Display B addr 0x71 on I2C2
"""

import time, threading, random
from collections import deque
import smbus
import os
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM

# --------------------------
# Pin config
# --------------------------
LED_PINS = ["P2_02","P2_04","P2_06","P2_08","P2_18","P2_20","P2_22","P2_24"]
JOY_SW_PIN = "P2_10"
VR_ADC_CHANNEL = 0  # AIN0 (P1_19 via voltage divider)
BUZZER_A_PIN = "P2_01"
BUZZER_B_PIN = "P2_03"

HT16K33_ADDR_A = 0x70
HT16K33_ADDR_B = 0x71
HT16K33_BUS_A = 1
HT16K33_BUS_B = 2

# --------------------------
# Game constants
# --------------------------
SLOW_BPM = 90
NORMAL_BPM = 120
FAST_BPM = 150
BPM_OPTIONS = [SLOW_BPM, NORMAL_BPM, FAST_BPM]
DEFAULT_BPM_IDX = 1

GAME_DURATION_SECONDS = 120
POINTS_PER_HIT = 5
LONG_PRESS_SECONDS = 5.0

TONE_FREQ = 2000
ERROR_FREQ = 600
PWM_DUTY = 50

# ADC thresholds for 4.7k/5.1k voltage divider
VR_LEFT_THRESH = 1400
VR_RIGHT_THRESH = 2600

AIN_BASE_PATH = "/sys/bus/iio/devices/iio:device0"

# --------------------------
# Automatic pin configuration
# --------------------------
def configure_pins():
    # LEDs
    for pin in LED_PINS:
        os.system(f"config-pin {pin} gpio")
    # Joystick SW
    os.system(f"config-pin {JOY_SW_PIN} gpio")
    # Buzzers
    os.system(f"config-pin {BUZZER_A_PIN} pwm")
    os.system(f"config-pin {BUZZER_B_PIN} pwm")
    # I2C (HT16K33)
    os.system("config-pin P2_09 i2c")  # Display A SCL
    os.system("config-pin P2_11 i2c")  # Display A SDA
    os.system("config-pin P1_26 i2c")  # Display B SCL
    os.system("config-pin P1_28 i2c")  # Display B SDA
    print("All pins configured automatically.")

# --------------------------
# HT16K33 driver
# --------------------------
class SevenSegHT16K33:
    DIGITS = [0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F]
    def __init__(self, address, busnum=1, brightness=15):
        self.addr = address
        self.bus = smbus.SMBus(busnum)
        self.bus.write_byte(self.addr, 0x21)
        self.bus.write_byte(self.addr, 0x81)
        self.bus.write_byte(self.addr, 0xE0 | (brightness & 0x0F))
        self.buf = [0x00]*16
        self.show_blank()
    def _flush(self): self.bus.write_i2c_block_data(self.addr, 0x00, self.buf)
    def show_blank(self):
        self.buf = [0x00]*16
        self._flush()
    def show_number(self, number, leading_zero=False):
        n = int(number) % 10000
        digits=[0,0,0,0]
        for i in range(4):
            digits[3-i] = n % 10; n//=10
        self.buf[0] = self.DIGITS[digits[0]] if leading_zero or digits[0]!=0 else 0x00
        self.buf[2] = self.DIGITS[digits[1]] if leading_zero or (digits[0]!=0 or digits[1]!=0) else 0x00
        self.buf[4] = self.DIGITS[digits[2]] if leading_zero or any(d!=0 for d in digits[:3]) else 0x00
        self.buf[6] = self.DIGITS[digits[3]]
        self._flush()

# --------------------------
# GPIO helpers
# --------------------------
def setup_gpio():
    for p in LED_PINS:
        try:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)
        except Exception as e:
            print(f"Failed to setup {p}: {e}")
    GPIO.setup(JOY_SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    #GPIO.setup(BUZZER_A_PIN, GPIO.OUT)
    #GPIO.setup(BUZZER_B_PIN, GPIO.OUT)
    #GPIO.output(BUZZER_A_PIN, GPIO.LOW)
    #GPIO.output(BUZZER_B_PIN, GPIO.LOW)

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
        with open(path, "r") as f:
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
# DDR game engine
# --------------------------
class DDRGame:
    def __init__(self):
        # 1️⃣ Automatic pin configuration
        configure_pins()   # <- must run first

        # 2️⃣ GPIO setup
        setup_gpio()

        # 3️⃣ Displays
        self.display_a = SevenSegHT16K33(HT16K33_ADDR_A, busnum=HT16K33_BUS_A)
        self.display_b = SevenSegHT16K33(HT16K33_ADDR_B, busnum=HT16K33_BUS_B)

        # 4️⃣ Game variables
        self.bpm_idx = DEFAULT_BPM_IDX
        self.bpm = BPM_OPTIONS[self.bpm_idx]
        self.running = False
        self.score = 0
        self.active = deque()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.show_bpm()

    def show_bpm(self):
        self.display_a.show_number(self.bpm, leading_zero=True)
        self.display_b.show_blank()
        
    def cycle_bpm(self, d):
        self.bpm_idx = (self.bpm_idx + d) % len(BPM_OPTIONS)
        self.bpm = BPM_OPTIONS[self.bpm_idx]
        self.show_bpm()

    def spawn_note(self):
        col = random.choice([0,1])
        with self.lock:
            self.active.append((col,0))
            set_led(col,0,True)

    def advance_notes(self):
        with self.lock:
            nq = deque()
            while self.active:
                col,row = self.active.popleft()
                set_led(col,row,False)
                nr = row + 1
                if nr <= 3:
                    set_led(col,nr,True)
                    nq.append((col,nr))
                else:
                    error_buzz()  # missed
            self.active = nq

    def check_input(self, pressed_col):
        with self.lock:
            for i,(col,row) in enumerate(self.active):
                if col==pressed_col and row==3:
                    self.score += POINTS_PER_HIT
                    set_led(col,row,False)
                    del self.active[i]
                    tempo_click()
                    return True
        error_buzz()
        return False

    def _format_mmss(self, s):
        m = int(s)//60; sec = int(s)%60; return m*100 + sec

    def _game_thread(self):
        seconds_per_beat = 60.0 / float(self.bpm)
        next_beat = time.time()
        end_time = time.time() + GAME_DURATION_SECONDS
        while not self.stop_event.is_set():
            now = time.time()
            if now >= end_time:
                break
            if now >= next_beat:
                tempo_click()
                self.advance_notes()
                self.spawn_note()
                remaining = max(0, int(end_time - now))
                self.display_a.show_number(self._format_mmss(remaining))
                next_beat += seconds_per_beat
            else:
                time.sleep(0.003)
        self.running = False

    def start_game(self):
        if self.running: return
        self.running = True
        self.score = 0
        self.active.clear()
        clear_all_leds()
        self.stop_event.clear()
        t = threading.Thread(target=self._game_thread); t.daemon=True; t.start()

    def end_sequence(self):
        self.stop_event.set()
        clear_all_leds()
        # flash score on display B for 8s
        until = time.time()+8.0
        while time.time() < until:
            self.display_b.show_number(self.score)
            time.sleep(0.6)
            self.display_b.show_blank()
            time.sleep(0.2)
        # display A flash 00:00 for 3s then off 5s
        t0 = time.time()
        while time.time() - t0 < 3.0:
            self.display_a.show_number(0)
            time.sleep(0.5)
            self.display_a.show_blank()
            time.sleep(0.25)
        self.display_a.show_blank()
        time.sleep(5.0)
        self.show_bpm()
        self.display_b.show_blank()
        clear_all_leds()
        self.running = False

    def run_ui(self):
        last = {"left":0,"right":0,"press":0}
        press_start = None
        long_done = False
        self.show_bpm()
        try:
            while True:
                js = joystick_from_analog()
                if not self.running:
                    if js["left"] and not last["left"]:
                        self.cycle_bpm(-1); time.sleep(0.18)
                    elif js["right"] and not last["right"]:
                        self.cycle_bpm(+1); time.sleep(0.18)
                    elif js["press"] and not last["press"]:
                        self.start_game(); time.sleep(0.25)
                    else:
                        time.sleep(0.02)
                else:
                    if js["press"]:
                        if press_start is None: press_start = time.time()
                        elif (not long_done) and (time.time() - press_start >= LONG_PRESS_SECONDS):
                            long_done = True
                            self.end_sequence()
                            press_start = None
                            long_done = False
                    else:
                        press_start = None
                    if js["left"] and not last["left"]:
                        self.check_input(0)
                    if js["right"] and not last["right"]:
                        self.check_input(1)
                    last = {"left": js["left"], "right": js["right"], "press": js["press"]}
                    time.sleep(0.01)
        except KeyboardInterrupt:
            print("Interrupted; stopping.")
            self.stop_event.set()


    # (Game functions: cycle_bpm, spawn_note, advance_notes, check_input, etc.)
    # Use the same functions from previous full code
    # For brevity, they are omitted here, but keep them unchanged.

    # You would copy all methods from the prior full script: cycle_bpm, spawn_note, advance_notes, check_input, _format_mmss, _game_thread, start_game, end_sequence, run_ui

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
            game.display_a.show_blank()
            game.display_b.show_blank()
            GPIO.cleanup()
            try: PWM.stop(BUZZER_A_PIN); PWM.stop(BUZZER_B_PIN)
            except: pass
            try: PWM.cleanup()
            except: pass
        except: pass

if __name__ == "__main__":
    main()
