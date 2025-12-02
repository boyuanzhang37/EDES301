#!/usr/bin/env python3
"""
pocketbeagle_ddr_final.py

DDR-style game using user-provided drivers:
 - LED class (uses Adafruit_BBIO.GPIO)
 - Buzzer class (uses Adafruit_BBIO.PWM)
 - HT16K33 class (uses i2cset via OS)
Wiring confirmed:
 - LEDs: P2_02,P2_04,P2_06,P2_08, P2_18,P2_20,P2_22,P2_24 (left col top->bottom then right)
 - Joystick VRy -> P1_19 (AIN0 via voltage divider)
 - Joystick SW -> P2_10
 - Buzzers (PWM) -> P2_01 (tempo), P2_03 (error)
 - HT16K33 -> bus 1 addr 0x70
"""

import os
import time
import threading
import random
from collections import deque

# --------------------------
# Inserted user drivers (LED, Buzzer, HT16K33)
# (kept exactly as provided, with small adjustments only where integration requires)
# --------------------------

# --- Buzzer driver (user provided) ---
import Adafruit_BBIO.PWM as PWM

class Buzzer():
    pin       = None

    def __init__(self, pin):
        self.pin = pin

    def play(self, frequency, length=1.0, stop=False):
        """ Plays the frequency for the length of time.
            frequency - Value in Hz or None for no tone
            length    - Time in seconds (default 1.0 seconds)
            stop      - Stop the buzzer (will cause breaks between tones)
        """
        if frequency is not None:
            PWM.start(self.pin, 50, frequency)

        time.sleep(length)

        if (stop):
            self.stop()

    def stop(self, length=0.0):
        PWM.stop(self.pin)
        time.sleep(length)

    def cleanup(self):
        PWM.stop(self.pin)
        PWM.cleanup()

# --- LED driver (user provided) ---
import Adafruit_BBIO.GPIO as GPIO

HIGH = GPIO.HIGH
LOW  = GPIO.LOW

class LED():
    pin = None
    on_value = None
    off_value = None

    def __init__(self, pin=None, low_off=True):
        if (pin == None):
            raise ValueError("Pin not provided for LED()")
        else:
            self.pin = pin

        if low_off:
            self.on_value  = HIGH
            self.off_value = LOW
        else:
            self.on_value  = LOW
            self.off_value = HIGH

        #self._setup()

    def _setup(self):
        GPIO.setup(self.pin, GPIO.OUT)
        self.off()

    def is_on(self):
        return GPIO.input(self.pin) == self.on_value

    def on(self):
        GPIO.output(self.pin, self.on_value)

    def off(self):
        GPIO.output(self.pin, self.off_value)

    def cleanup(self):
        self.off()

# --- HT16K33 driver (user provided) ---
import os as _os

HEX_DIGITS = [0x3f, 0x06, 0x5b, 0x4f,
              0x66, 0x6d, 0x7d, 0x07,
              0x7f, 0x6f, 0x77, 0x7c,
              0x39, 0x5e, 0x79, 0x71]

LETTERS = { "a":0x77,"A":0x77,"b":0x7c,"B":0x7c,"c":0x58,"C":0x39,
            "d":0x5e,"D":0x5e,"e":0x79,"E":0x79,"f":0x71,"F":0x71,
            "g":0x6F,"G":0x6F,"h":0x74,"H":0x76,"i":0x04,"I":0x30,
            "j":0x0e,"J":0x0e,"l":0x38,"L":0x38,"n":0x54,"N":0x54,
            "o":0x5c,"O":0x3f,"p":0x73,"P":0x73,"q":0x67,"Q":0x67,
            "r":0x50,"R":0x50,"s":0x6D,"S":0x6D,"t":0x78,"T":0x78,
            "u":0x1c,"U":0x3e,"y":0x6e,"Y":0x6e," ":"0x00","-":0x40,
            "0":0x3f,"1":0x06,"2":0x5b,"3":0x4f,"4":0x66,"5":0x6d,"6":0x7d,"7":0x07,"8":0x7f,"9":0x6f,"?":0x53 }

CLEAR_DIGIT = 0x7F
POINT_VALUE = 0x80
DIGIT_ADDR = [0x00, 0x02, 0x06, 0x08]
COLON_ADDR = 0x04
HT16K33_BLINK_CMD = 0x80
HT16K33_BLINK_DISPLAYON = 0x01
HT16K33_SYSTEM_SETUP = 0x20
HT16K33_OSCILLATOR = 0x01
HT16K33_BRIGHTNESS_CMD = 0xE0

class HT16K33():
    def __init__(self, bus, address=0x70, blink=0x00, brightness=0x0F):
        self.bus = bus
        self.address = address
        self.command = "/usr/sbin/i2cset -y {0} {1}".format(bus, address)
        self.setup(blink, brightness)
        self.blank()

    def setup(self, blink, brightness):
        _os.system("{0} {1}".format(self.command, (HT16K33_SYSTEM_SETUP | HT16K33_OSCILLATOR)))
        _os.system("{0} {1}".format(self.command, (HT16K33_BLINK_CMD | blink | HT16K33_BLINK_DISPLAYON)))
        _os.system("{0} {1}".format(self.command, (HT16K33_BRIGHTNESS_CMD | brightness)))

    def encode(self, data, double_point=False):
        ret_val = 0
        try:
            if (data != CLEAR_DIGIT):
                if double_point:
                    ret_val = HEX_DIGITS[data] + POINT_VALUE
                else:
                    ret_val = HEX_DIGITS[data]
        except:
            raise ValueError("Digit value must be between 0 and 15.")
        return ret_val

    def set_digit(self, digit_number, data, double_point=False):
        _os.system("{0} {1} {2}".format(self.command, DIGIT_ADDR[digit_number], self.encode(data, double_point)))

    def set_digit_raw(self, digit_number, data, double_point=False):
        _os.system("{0} {1} {2}".format(self.command, DIGIT_ADDR[digit_number], data))

    def set_colon(self, enable):
        if enable:
            _os.system("{0} {1} {2}".format(self.command, COLON_ADDR, 0x02))
        else:
            _os.system("{0} {1} {2}".format(self.command, COLON_ADDR, 0x00))

    def blank(self):
        self.set_colon(False)
        self.set_digit_raw(3, 0x00)
        self.set_digit_raw(2, 0x00)
        self.set_digit_raw(1, 0x00)
        self.set_digit_raw(0, 0x00)

    def clear(self):
        self.set_colon(False)
        self.update(0)

    def update(self, value):
        if ((value < 0) or (value > 9999)):
            raise ValueError("Value is not between 0 and 9999")
        self.set_digit(3, (value % 10))
        self.set_digit(2, (value // 10) % 10)
        self.set_digit(1, (value // 100) % 10)
        self.set_digit(0, (value // 1000) % 10)

    def text(self, value):
        if ((len(value) < 1) or (len(value) > 4)):
            raise ValueError("Must have between 1 and 4 characters")
        self.blank()
        for i, char in enumerate(value):
            try:
                char_value = LETTERS[char]
                self.set_digit_raw(i, char_value)
            except:
                raise ValueError("Character {0} not supported".format(char))

# --------------------------
# End drivers insertion
# --------------------------

# --------------------------
# Hardware/pin configuration (user-confirmed)
# --------------------------
LED_PINS = ["P2_02","P2_04","P2_06","P2_08","P2_18","P2_20","P2_22","P2_24"]
JOY_VR_ADC_CHANNEL = 0   # AIN0 -> P1_19
JOY_SW_PIN = "P2_10"
BUZZER_A_PIN = "P2_01"
BUZZER_B_PIN = "P2_03"
HT16K33_BUS = 1
HT16K33_ADDR = 0x70

# --------------------------
# Game constants
# --------------------------
SLOW_BPM = 90
NORMAL_BPM = 120
FAST_BPM = 150
BPM_OPTIONS = [SLOW_BPM, NORMAL_BPM, FAST_BPM]
DEFAULT_BPM_IDX = 1

GAME_DURATION_SECONDS = 120
VR_LEFT_THRESH = 1400
VR_RIGHT_THRESH = 2600
AIN_BASE_PATH = "/sys/bus/iio/devices/iio:device0"

# tempo/error tones (Hz) and buzzer behavior
TEMPO_FREQ = 2000
TEMPO_LEN = 0.05
ERROR_FREQ = 600
ERROR_LEN = 0.14

# --------------------------
# Pinmux + instances setup
# --------------------------
def configure_pins_and_init():
    """Run config-pin for the used pins, wait, then create device objects."""
    # pinmux
    for p in LED_PINS:
        os.system(f"config-pin {p} gpio")
    os.system(f"config-pin {JOY_SW_PIN} gpio")
    os.system(f"config-pin {BUZZER_A_PIN} pwm")
    os.system(f"config-pin {BUZZER_B_PIN} pwm")
    os.system("config-pin P2_09 i2c")
    os.system("config-pin P2_11 i2c")

    # allow pinmux to apply
    time.sleep(1.0)

    # instantiate LED objects (will call GPIO.setup inside)
    leds = [LED(pin) for pin in LED_PINS]
    
    #joystick
    GPIO.setup(JOY_SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    # instantiate buzzers
    buzzer_tempo = Buzzer(BUZZER_A_PIN)
    buzzer_error = Buzzer(BUZZER_B_PIN)

    # instantiate display
    display = HT16K33(HT16K33_BUS, HT16K33_ADDR)

    return leds, buzzer_tempo, buzzer_error, display

# --------------------------
# ADC read helper
# --------------------------
def read_adc_raw(channel=0):
    path = f"{AIN_BASE_PATH}/in_voltage{channel}_raw"
    try:
        with open(path, "r") as f:
            return int(f.read().strip())
    except Exception as e:
        raise RuntimeError(f"Failed to read ADC channel {channel}: {e}")

def joystick_from_analog():
    raw = read_adc_raw(JOY_VR_ADC_CHANNEL)
    left = raw < VR_LEFT_THRESH
    right = raw > VR_RIGHT_THRESH
    press = not GPIO.input(JOY_SW_PIN)   # pressed pulls to GND
    return {"left": left, "right": right, "press": press, "raw": raw}

# --------------------------
# LED grid helpers (use LED instances)
# --------------------------
def led_pattern_to_leds(pattern, leds):
    """pattern: list of 8 ints (0/1). leds: list of 8 LED objects."""
    for val, led in zip(pattern, leds):
        if val:
            led.on_value()
        else:
            led.off_value()

def clear_all_leds(leds):
    for led in leds:
        led.off_value()

# --------------------------
# DDR Game
# --------------------------
class DDRGame:
    def __init__(self, leds, buzzer_tempo, buzzer_error, display):
        self.leds = leds
        self.buzzer_tempo = buzzer_tempo
        self.buzzer_error = buzzer_error
        self.display = display

        self.bpm_idx = DEFAULT_BPM_IDX
        self.bpm = BPM_OPTIONS[self.bpm_idx]
        self.running = False

        # active notes: deque of (col, row)
        self.active_notes = deque()
        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.long_press_start = None

        # show bpm initially
        self.show_bpm()

    # display helpers
    def show_bpm(self):
        # show raw bpm number e.g. 120
        self.display.update(self.bpm)

    def show_mmss(self, seconds):
        # display expects a 4-digit number; format as MMSS (m*100 + s)
        m = int(seconds) // 60
        s = int(seconds) % 60
        self.display.update(m*100 + s)

    # game control
    def cycle_bpm(self, d):
        self.bpm_idx = (self.bpm_idx + d) % len(BPM_OPTIONS)
        self.bpm = BPM_OPTIONS[self.bpm_idx]
        self.show_bpm()

    def spawn_note_top(self):
        col = random.choice([0,1])
        with self.lock:
            self.active_notes.append((col, 0))

    def advance_notes_one_row(self):
        """Move notes down one row; notes that fall off bottom are removed and cause error buzz."""
        new = deque()
        missed = False
        with self.lock:
            while self.active_notes:
                col, row = self.active_notes.popleft()
                row += 1
                if row <= 3:
                    new.append((col, row))
                else:
                    missed = True
            self.active_notes = new
        return missed

    def build_led_pattern(self):
        """Return 8-length list (0/1) describing LEDs to light based on active_notes."""
        pattern = [0]*8
        with self.lock:
            for col, row in self.active_notes:
                idx = col*4 + row
                if 0 <= idx < 8:
                    pattern[idx] = 1
        return pattern

    def tempo_click(self):
        # use buzzer_tempo.play with stop=True to ensure it stops
        threading.Thread(target=self.buzzer_tempo.play, args=(TEMPO_FREQ, TEMPO_LEN, True), daemon=True).start()

    def error_buzz(self):
        threading.Thread(target=self.buzzer_error.play, args=(ERROR_FREQ, ERROR_LEN, True), daemon=True).start()

    # input checking: expects pressed_col = 0 or 1
    def check_input_hit(self, pressed_col):
        """Return True if hit is correct: a note exists at bottom row (row==3) and matching column."""
        with self.lock:
            for i, (col, row) in enumerate(self.active_notes):
                if col == pressed_col and row == 3:
                    # remove this note
                    del self.active_notes[i]
                    return True
        return False

    # game run loop (in separate thread)
    def _game_thread(self):
        seconds_per_beat = 60.0 / float(self.bpm)
        next_beat = time.time()
        end_time = time.time() + GAME_DURATION_SECONDS

        # clear any previous active notes
        with self.lock:
            self.active_notes.clear()

        while not self.stop_flag.is_set():
            now = time.time()
            if now >= end_time:
                break
            if now >= next_beat:
                # on-beat actions
                self.tempo_click()
                missed = self.advance_notes_one_row()
                if missed:
                    # a note fell off bottom without hit; play error buzz
                    self.error_buzz()

                # spawn a new top note
                self.spawn_note_top()

                # update LEDs to reflect current notes
                pattern = self.build_led_pattern()
                led_pattern_to_leds(pattern, self.leds)

                # update display with countdown
                remaining = max(0, int(end_time - now))
                self.show_mmss(remaining)

                next_beat += seconds_per_beat
            else:
                # brief sleep to let input thread run
                time.sleep(0.002)

        # end of game -> run end/reset sequence
        self.running = False
        self._end_sequence()

    def start_game(self):
        if self.running:
            return
        self.running = True
        self.stop_flag.clear()
        # start the beat thread
        t = threading.Thread(target=self._game_thread, daemon=True)
        t.start()

    def stop_game(self):
        self.stop_flag.set()

    def _end_sequence(self):
        # clear LEDs
        clear_all_leds(self.leds)
        # flash 00:00 for 3 cycles, then restore BPM display
        for _ in range(3):
            self.display.update(0)    # show 00:00 as 0000
            time.sleep(0.5)
            self.display.blank()
            time.sleep(0.5)
        self.show_bpm()

    # main UI loop to be run in main thread
    def run_ui(self):
        print("Use joystick left/right to select BPM. Press to start. Hold 5s to abort game.")
        last = {"left": False, "right": False, "press": False}

        try:
            while True:
                js = joystick_from_analog()

                # when not running: left/right cycles BPM, press starts game
                if not self.running:
                    if js["left"] and not last["left"]:
                        self.cycle_bpm(-1)   # left cycles backward
                        time.sleep(0.18)    # simple debounce
                    elif js["right"] and not last["right"]:
                        self.cycle_bpm(+1)
                        time.sleep(0.18)
                    elif js["press"] and not last["press"]:
                        # start game
                        self.start_game()
                        time.sleep(0.25)
                else:
                    # game is running: handle long-press for abort
                    if js["press"]:
                        if self.long_press_start is None:
                            self.long_press_start = time.time()
                        elif (time.time() - self.long_press_start) >= 5.0:
                            # abort and reset
                            self.stop_game()
                            self.long_press_start = None
                            time.sleep(0.5)
                    else:
                        self.long_press_start = None

                    # gameplay input: detect rising edges on left/right
                    if js["left"] and not last["left"]:
                        hit = self.check_input_hit(0)
                        if not hit:
                            self.error_buzz()
                        else:
                            # correct hit: small tempo click to acknowledge (optional)
                            self.tempo_click()
                    if js["right"] and not last["right"]:
                        hit = self.check_input_hit(1)
                        if not hit:
                            self.error_buzz()
                        else:
                            self.tempo_click()

                last = {"left": js["left"], "right": js["right"], "press": js["press"]}
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Interrupted by user — exiting.")
            self.stop_game()

# --------------------------
# Main program
# --------------------------
def main():
    # configure pins and instantiate hardware
    leds, buzzer_tempo, buzzer_error, display = configure_pins_and_init()

    # instantiate game using the driver objects
    game = DDRGame(leds, buzzer_tempo, buzzer_error, display)

    try:
        game.run_ui()
    finally:
        # cleanup on exit
        try:
            game.stop_game()
        except: pass
        # turn off leds
        clear_all_leds(leds)
        # blank display
        try:
            display.blank()
        except: pass
        # cleanup buzzers PWM
        try:
            buzzer_tempo.cleanup()
        except: pass
        try:
            buzzer_error.cleanup()
        except: pass
        # cleanup LEDs GPIO wrapper (calls off)
        for led in leds:
            try:
                led.cleanup()
            except: pass
        # final GPIO cleanup if desired
        try:
            GPIO.cleanup()
        except: pass

if __name__ == "__main__":
    main()
