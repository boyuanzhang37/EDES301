#!/usr/bin/env python3
import time, random, threading
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import board, busio
from adafruit_ht16k33 import segments

# -------------------------
# Config
# -------------------------
BPM_OPTIONS = [60, 90, 120]
BPM_INDEX_DEFAULT = 1

LEDS = [  # 8 individually wired LEDs
    "P2_01", "P2_02", "P2_03", "P2_04",  # Left column (top to bottom)
    "P2_05", "P2_06", "P2_07", "P2_08",  # Right column (top to bottom)
]

JOYSTICK = {
    "left":  "P1_29",
    "right": "P1_28",
    "press": "P1_31",
}

BUZZER_A = "P2_19"
BUZZER_B = "P2_17"

I2C_ADDR_A = 0x70
I2C_ADDR_B = 0x71

POINTS_PER_HIT = 5
ON_BEAT_TOLERANCE = 0.08
GAME_SECONDS = 120
DEBOUNCE_MS = 40

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
# Shared state
# -------------------------
class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.score = 0
        self.remaining = GAME_SECONDS
        self.bpm = BPM_OPTIONS[BPM_INDEX_DEFAULT]
        self.running = False
        self.end_requested = False
        self.latest_dir = None
        self.press_start = None

shared = Shared()

# -------------------------
# Display thread
# -------------------------
def display_thread(dispA, dispB, stop_event):
    while not stop_event.is_set():
        with shared.lock:
            if shared.running:
                mm = shared.remaining // 60
                ss = shared.remaining % 60
                dispA.print(f"{mm:02}{ss:02}")
                dispB.print(f"{shared.score:4d}"[-4:])
            else:
                dispA.print(str(shared.bpm))
                dispB.fill(0)
        time.sleep(0.1)

# -------------------------
# Input thread
# -------------------------
_last_states = {k: 0 for k in JOYSTICK}

def pressed_edge(key):
    pin = JOYSTICK[key]
    val = 1 if GPIO.input(pin) else 0
    if val == 1 and _last_states[key] == 0:
        time.sleep(DEBOUNCE_MS / 1000.0)
        if GPIO.input(pin):
            _last_states[key] = 1
            return True
    _last_states[key] = val
    return False

def input_thread(stop_event):
    while not stop_event.is_set():
        states = {k: GPIO.input(pin) for k, pin in JOYSTICK.items()}
        with shared.lock:
            if states["left"]:
                shared.latest_dir = "left"
            elif states["right"]:
                shared.latest_dir = "right"
            else:
                shared.latest_dir = None

            if not shared.running:
                if pressed_edge("left"):
                    idx = BPM_OPTIONS.index(shared.bpm)
                    shared.bpm = BPM_OPTIONS[(idx - 1) % len(BPM_OPTIONS)]
                elif pressed_edge("right"):
                    idx = BPM_OPTIONS.index(shared.bpm)
                    shared.bpm = BPM_OPTIONS[(idx + 1) % len(BPM_OPTIONS)]

            if states["press"]:
                if shared.press_start is None:
                    shared.press_start = time.time()
                elif time.time() - shared.press_start >= 5.0:
                    shared.end_requested = True
            else:
                shared.press_start = None
        time.sleep(0.003)

# -------------------------
# Game loop
# -------------------------
def run_game():
    with shared.lock:
        bpm = shared.bpm
        shared.score = 0
        shared.remaining = GAME_SECONDS
        shared.running = True
        shared.end_requested = False

    beat_int = 60.0 / bpm
    end_time = time.time() + GAME_SECONDS
    next_beat = time.time() + 0.3
    col = random.randrange(0, 2)
    row = -1

    while True:
        now = time.time()
        with shared.lock:
            shared.remaining = max(0, int(end_time - now))
            if shared.end_requested or shared.remaining <= 0:
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
                while time.time() <= deadline:
                    with shared.lock:
                        dir_in = shared.latest_dir
                    if dir_in:
                        if (dir_in == "left" and col == 0) or (dir_in == "right" and col == 1):
                            with shared.lock:
                                shared.score += POINTS_PER_HIT
                        else:
                            error_buzz()
                        break
                    time.sleep(0.005)
                time.sleep(max(0, target + beat_int - time.time()))
                all_leds_off()
                row = -1
            next_beat += beat_int
        time.sleep(0.002)

    with shared.lock:
        shared.running = False

# -------------------------
# Main
# -------------------------
def main():
    dispA, dispB = setup_gpio_and_displays()
    stop_event = threading.Event()
    th_disp = threading.Thread(target=display_thread, args=(dispA, dispB, stop_event), daemon=True)
    th_inp = threading.Thread(target=input_thread, args=(stop_event,), daemon=True)
    th_disp.start()
    th_inp.start()

    try:
        while True:
            while True:
                if pressed_edge("press"):
                    run_game()
                    dispA.print("0000")
                    time.sleep(3)
                    dispA.fill(0)
                    time.sleep(5)
                    dispB.print(f"{shared.score:04d}")
                    time.sleep(8)
                    dispB.fill(0)
                    break
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        th_disp.join(1)
        th_inp.join(1)
        cleanup()

if __name__ == "__main__":
    main()