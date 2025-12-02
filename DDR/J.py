#!/usr/bin/env python3
"""
--------------------------------------------------------------------------
Buzzer
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
DDR-style falling-note game for PocketBeagle (final)


This file contains:
 - safe pinmuxing/config
 - joystick (Y axis) driver + press handling
 - RhythmGame class implementing falling-lane logic
 - main() to instantiate and run
 
Very much does not work overall, but the joystick driver works well
"""

import os
import time
import threading
import random
from collections import deque

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC

# -----------------------
# Hardware mapping — confirm these match your wiring
# -----------------------
#Quick pin config
LED_PINS = ["P2_02","P2_04","P2_06","P2_08","P2_18","P2_20","P2_22","P2_24"]
JOY_VR_ADC_CHANNEL = 0   # AIN0 -> P1_19
JOY_ADC_PIN = "P1_19"
JOY_SW_PIN = "P2_10"
BUZZER_TEMPO_PIN = "P2_01"
BUZZER_ERROR_PIN = "P2_03"
HT16K33_BUS = 1
HT16K33_ADDR = 0x70

def configure_pins_and_init():
    """Run config-pin for the used pins, wait, then create device objects."""
    # pinmux
    for p in LED_PINS:
        os.system(f"config-pin {p} gpio")
    os.system(f"config-pin {JOY_SW_PIN} gpio")
    os.system(f"config-pin {BUZZER_TEMPO_PIN} pwm")
    os.system(f"config-pin {BUZZER_ERROR_PIN} pwm")
    os.system("config-pin P2_09 i2c")
    os.system("config-pin P2_11 i2c")

    # allow pinmux to apply
    time.sleep(1.0)

    # instantiate LED objects (will call GPIO.setup inside)
    leds = [LED(pin) for pin in LED_PINS]
    
    #joystick
    GPIO.setup(JOY_SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    # instantiate buzzers
    buzzer_tempo = Buzzer("P2_01")
    buzzer_error = Buzzer("P2_03")

    # instantiate display
    display = HT16K33(HT16K33_BUS, HT16K33_ADDR)

    return leds, buzzer_tempo, buzzer_error, display
    

# Game constants
SLOW_BPM = 90
NORMAL_BPM = 150   
FAST_BPM = 200
BPM_MIN = 60
BPM_MAX = 300
DEFAULT_BPM = 150

GAME_DURATION_SECONDS = 120   # 2 minutes
POINTS_PER_HIT = 5

# ADC thresholds for left/center/right
LEFT_THRESH = 0.35
RIGHT_THRESH = 0.65

# Buzzer tones
TEMPO_FREQ = 800
TEMPO_LEN  = 0.06
ERROR_FREQ = 600
ERROR_LEN  = 0.12

# --- Buzzer driver  ---
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

# --- LED driver  ---
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

    #def _setup(self):
        #GPIO.setup(self.pin, GPIO.OUT)
        #self.off()

    #def is_on(self):
        #return GPIO.input(self.pin) == self.on_value

    def on(self):
        GPIO.output(self.pin, self.on_value)

    def off(self):
        GPIO.output(self.pin, self.off_value)

    def cleanup(self):
        self.off()

# --- HT16K33 driver  ---
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

# ================================
# JOYSTICK CLASS (Y-axis + switch)
# ================================
class Joystick:
    """
    Single-axis (Y) joystick plus push-button.
    read_axis() -> -1 (left), 0 (center), +1 (right)
    pressed() -> boolean
    press_duration() -> blocks until release and returns duration in seconds
    """

    def __init__(self, adc_pin=JOY_ADC_PIN, sw_pin=JOY_SW_PIN,
                 left_thresh=LEFT_THRESH, right_thresh=RIGHT_THRESH):
        self.adc_pin = adc_pin
        self.sw_pin = sw_pin
        self.left_thresh = left_thresh
        self.right_thresh = right_thresh

        ADC.setup()
        # Ensure GPIO pull-up input set for switch
        GPIO.setup(self.sw_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read_raw(self):
        """Return raw ADC reading 0.0..1.0 (may return None on error)."""
        return ADC.read(self.adc_pin)

    def read_axis(self):
        """Return -1 (left), 0 (center), +1 (right)."""
        val = self.read_raw()
        if val is None:
            return 0
        if val < self.left_thresh:
            return -1
        elif val > self.right_thresh:
            return 1
        else:
            return 0

    def pressed(self):
        """Return True if push-button is pressed (active-low)."""
        return GPIO.input(self.sw_pin) == 0

    def press_duration(self):
        """If button currently pressed, block until release and return duration (s)."""
        if not self.pressed():
            return 0.0
        start = time.time()
        while self.pressed():
            time.sleep(0.01)
        return time.time() - start

    def cleanup(self):
        pass


# ================================
# HELPER: pinmux + instantiate devices
# ================================
def configure_pins_before_instantiation():
    """Run config-pin for pins we use and do a safe GPIO setup for LED pins."""
    # pinmux: LED GPIO pins
    for p in LED_PINS:
        os.system(f"config-pin {p} gpio")
    # joystick switch
    os.system(f"config-pin {JOY_SW_PIN} gpio")
    # buzzers as pwm
    os.system(f"config-pin {BUZZER_TEMPO_PIN} pwm")
    os.system(f"config-pin {BUZZER_ERROR_PIN} pwm")
    # HT16K33 I2C
    os.system("config-pin P2_09 i2c")
    os.system("config-pin P2_11 i2c")

    # small pause for kernel to apply pinmux
    time.sleep(0.6)

    # perform a safe GPIO.setup for LED pins to avoid race condition:
    for p in LED_PINS:
        try:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)
        except Exception:
            # ignore here — LED class will also attempt setup
            pass

# ================================
# RHYTHM GAME CLASS
# ================================
class RhythmGame:
    def __init__(self, led_objs, buzzer_tempo, buzzer_error, display, joystick,
                 default_bpm=DEFAULT_BPM):

        # keep the passed-in objects exactly as they are
        self.leds = led_objs
        self.buzzer_tempo = buzzer_tempo
        self.buzzer_error = buzzer_error
        self.display = display
        self.joystick = joystick

        # game state
        self.bpm = default_bpm
        self.score = 0
        self.running = False
        self.stop_event = threading.Event()
        self.note_lock = threading.Lock()

        # active notes: deque of (col, row)
        self.active_notes = deque()

    # -------------------------
    # Display helpers
    # -------------------------
    def show_bpm(self):
        # show BPM as integer (e.g., 150)
        try:
            self.display.update(self.bpm)
        except Exception:
            # fallback to text if update fails
            try:
                self.display.text(str(self.bpm))
            except Exception:
                pass

    def show_score(self):
        # show score as integer (00..)
        try:
            self.display.update(self.score)
        except Exception:
            try:
                # If update fails (value > 9999) just show text
                self.display.text(f"{self.score}")
            except Exception:
                pass

    # -------------------------
    # Note generation and movement
    # -------------------------
    def spawn_note_top(self):
        """Pick a random column (0=left,1=right) and append at row 0."""
        col = random.choice([0,1])
        with self.note_lock:
            self.active_notes.append((col, 0))

    def advance_notes(self):
        """Advance each note down by 1 row. Return list of notes that reached bottom this beat."""
        reached_bottom = []
        new_deque = deque()
        with self.note_lock:
            while self.active_notes:
                col, row = self.active_notes.popleft()
                row += 1
                if row <= 3:
                    new_deque.append((col, row))
                else:
                    # row > 3 -> note fell off (missed) — treat as missed
                    # but per spec we only check input at moment it reaches row==3
                    pass
            # assign new notes
            self.active_notes = new_deque
        return reached_bottom  # not used directly; logic will check row==3 after advance

    def build_led_pattern_and_set(self):
        """Set LED objects according to active_notes. Only one LED per row will be on because spawning chooses column per note."""
        # Clear all first
        for led in self.leds:
            led.off()

        with self.note_lock:
            for col, row in self.active_notes:
                # map (col, row) to led index:
                # left column indices: 0..3 top->bottom = row
                # right column indices: 4..7 top->bottom = 4 + row
                idx = col * 4 + row
                if 0 <= idx < len(self.leds):
                    try:
                        self.leds[idx].on()
                    except Exception:
                        pass

    # -------------------------
    # Hit checking (only when note is at bottom)
    # -------------------------
    def check_hits_at_bottom(self):
        """Check for notes that are at row 3 and compare joystick direction.
           For each bottom note: if correct input present at that beat award points, else error buzz.
           Returns how many correct hits.
        """
        correct_hits = 0
        with self.note_lock:
            # iterate copy because we may remove hits
            for i in range(len(self.active_notes)):
                col, row = self.active_notes[i]
                if row == 3:
                    # expected column: 0 -> left, 1 -> right
                    axis = self.joystick.read_axis()  # -1,0,+1
                    expected = -1 if col == 0 else 1
                    # Accept input only if axis equals expected (i.e., user is holding direction at the beat)
                    if axis == expected:
                        correct_hits += 1
                        # remove that note
                        del self.active_notes[i]
                        break  # modify-safe: exit after one removal, small simplification
                    else:
                        # missed or wrong: play error (but keep note — it will be removed next beat by falling off)
                        threading.Thread(target=self.buzzer_error.play, args=(ERROR_FREQ, ERROR_LEN, True), daemon=True).start()
                        # don't remove note here (it will advance and be cleared)
                        break
        return correct_hits

    # -------------------------
    # Game core (beat-driven)
    # -------------------------
    def _game_thread(self):
        beat_interval = 60.0 / float(self.bpm)
        end_time = time.time() + GAME_DURATION_SECONDS
        next_beat = time.time()

        # Initially spawn a first note so game has flow
        self.spawn_note_top()

        while not self.stop_event.is_set():
            now = time.time()
            if now >= end_time:
                break

            if now >= next_beat:
                # 1) On-beat: tempo click
                threading.Thread(target=self.buzzer_tempo.play, args=(TEMPO_FREQ, TEMPO_LEN, True), daemon=True).start()

                # 2) Advance notes downward
                # move existing notes down one row
                with self.note_lock:
                    new = deque()
                    for col, row in self.active_notes:
                        new_row = row + 1
                        if new_row <= 3:
                            new.append((col, new_row))
                        else:
                            # note fell off bottom without hit -> error
                            threading.Thread(target=self.buzzer_error.play, args=(ERROR_FREQ, ERROR_LEN, True), daemon=True).start()
                    self.active_notes = new

                # 3) Spawn a new top note
                self.spawn_note_top()

                # 4) Build LED pattern and set outputs
                self.build_led_pattern_and_set()

                # 5) If any note now at bottom row (row==3), check player input immediately
                #    (We check once per beat.)
                correct = 0
                with self.note_lock:
                    for idx, (col, row) in enumerate(self.active_notes):
                        if row == 3:
                            axis = self.joystick.read_axis()
                            expected = -1 if col == 0 else 1
                            if axis == expected:
                                correct += 1
                                # award points and remove this note
                                del self.active_notes[idx]
                                # play a small confirmation tone (tempo beep)
                                threading.Thread(target=self.buzzer_tempo.play, args=(TEMPO_FREQ, 0.03, True), daemon=True).start()
                                self.score += POINTS_PER_HIT
                                self.show_score()
                                break
                            else:
                                # error buzz already handled above on fall-off; also play error immediately
                                threading.Thread(target=self.buzzer_error.play, args=(ERROR_FREQ, ERROR_LEN, True), daemon=True).start()
                                break

                # 6) update display (score shown during game)
                self.show_score()

                # schedule next beat
                next_beat += beat_interval

            else:
                time.sleep(0.003)

        # end of game loop
        self.running = False

        # End-of-game sequence:
        # Show final score for 8 seconds (flashing each 0.5s)
        final = self.score
        for _ in range(16):
            try:
                self.display.update(final)
            except Exception:
                pass
            time.sleep(0.25)
            self.display.blank()
            time.sleep(0.25)

        # Flash 00:00 three times (0.5s on/off)
        for _ in range(3):
            try:
                self.display.update(0)
            except Exception:
                pass
            time.sleep(0.5)
            self.display.blank()
            time.sleep(0.5)

        # Blank for 5 seconds
        self.display.blank()
        time.sleep(5)

        # restore BPM display
        self.show_bpm()

    # -------------------------
    # Start / stop game
    # -------------------------
    def start_game(self):
        if self.running:
            return
        self.running = True
        self.stop_event.clear()
        self.score = 0
        self.show_score()
        self.thread = threading.Thread(target=self._game_thread, daemon=True)
        self.thread.start()

    def stop_game(self):
        self.stop_event.set()
        # thread will finish and run end sequence

    # -------------------------
    # BPM selection UI (before start)
    # -------------------------
    def bpm_selection_ui(self):
        # show BPM on display
        self.show_bpm()
        last_move = time.time()

        print("Use joystick left/right to select BPM. Short press to start. Hold 5s to abort/return.")

        while True:
            axis = self.joystick.read_axis()
            now = time.time()

            # axis -1 -> left (decrease), +1 -> right (increase)
            if axis != 0 and (now - last_move) > 0.12:
                if axis < 0:
                    self.bpm = max(BPM_MIN, self.bpm - 1)
                else:
                    self.bpm = min(BPM_MAX, self.bpm + 1)
                self.show_bpm()
                last_move = now

            # Button press handling
            if self.joystick.pressed():
                dur = self.joystick.press_duration()
                if dur >= 5.0:
                    # long hold at selection – treat as "go back / abort" (here we just re-show BPM)
                    print("Long hold detected in menu — returning to BPM display.")
                    self.show_bpm()
                    # continue loop
                    time.sleep(0.2)
                else:
                    # short press: start game
                    return

            time.sleep(0.02)

    # -------------------------
    # Run the full game loop
    # -------------------------
    def run(self):
        try:
            while True:
                # pre-game: show BPM and allow changes
                self.bpm_selection_ui()

                # start game
                self.start_game()

                # monitor for long press to abort during play (5s)
                while self.running:
                    if self.joystick.pressed():
                        # measure duration without blocking main monitor
                        start = time.time()
                        while self.joystick.pressed() and (time.time() - start) < 5.0:
                            time.sleep(0.05)
                        if self.joystick.pressed() and (time.time() - start) >= 5.0:
                            # long hold detected -> stop game
                            print("Long hold during game -> aborting to menu.")
                            self.stop_game()
                            break
                    time.sleep(0.05)

                # after game ends, loop returns to BPM selection state (end sequence runs in thread)
                # give a short pause to allow end-sequence display to start
                time.sleep(0.5)

        except KeyboardInterrupt:
            print("User requested exit. Cleaning up...")
            self.stop_game()
            # small delay to let threads cleanup
            time.sleep(0.2)

    # -------------------------
    # Cleanup
    # -------------------------
    def cleanup(self):
        # turn off leds
        for led in self.leds:
            try:
                led.off()
            except Exception:
                pass
        # blank display
        try:
            self.display.clear()
        except Exception:
            pass
        # cleanup buzzers
        try:
            self.buzzer_tempo.cleanup()
        except Exception:
            pass
        try:
            self.buzzer_error.cleanup()
        except Exception:
            pass
        # joystick cleanup (noop)
        try:
            self.joystick.cleanup()
        except Exception:
            pass
        # final GPIO cleanup (optional)
        try:
            GPIO.cleanup()
        except Exception:
            pass


# ================================
# MAIN
# ================================
def main():
    configure_pins_before_instantiation()
    # Instantiate hardware
    leds = [
        LED("P2_02"), LED("P2_04"), LED("P2_06"), LED("P2_08"),
        LED("P2_18"), LED("P2_20"), LED("P2_22"), LED("P2_24")
    ]

    buzzer_tempo = Buzzer("P2_01")   # Working PWM pin
    buzzer_error = Buzzer("P2_03")   # Working PWM pin

    display = HT16K33(bus=1)

    # Y-axis ADC + press pin
    joystick = Joystick("P1_19", "P2_10")

    
    game = RhythmGame(
        leds,
        buzzer_tempo,
        buzzer_error,
        display,
        joystick
    )

    try:
        game.run()
    except KeyboardInterrupt:
        pass
    finally:
        game.cleanup()


if __name__ == "__main__":
    main()
