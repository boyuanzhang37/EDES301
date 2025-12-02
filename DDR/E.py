#!/usr/bin/env python3
"""
Simplified DDR-style game for PocketBeagle
- Single HT16K33 display (shows BPM before start, countdown timer during game)
- 4x2 LED grid for DDR arrows
- Joystick: VRy -> P1_19, SW -> P2_10
- Buzzers: P2_15 (tempo), P2_16 (error)
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
    # Pinmux for LEDs
    for pin in LED_PINS:
        os.system(f"config-pin {pin} gpio")
    # Joystick button
    os.system(f"config-pin {JOY_SW_PIN} gpio")
    # Buzzers
    os.system(f"config-pin {BUZZER_A_PIN} pwm")
    os.system(f"config-pin {BUZZER_B_PIN} pwm")
    # I2C bus for HT16K33
    os.system("config-pin P2_09 i2c")
    os.system("config-pin P2_11 i2c")

    print("Pins configured. Waiting for pinmux to settle...")
    time.sleep(1.0)  # Give the kernel time to apply pinmux
    
 
    #Initialize GPIO pins
    for pin in LED_PINS:
        #GPIO.setup(pin, GPIO.OUT)
        #GPIO.output(pin, GPIO.LOW)
    
        GPIO.setup(JOY_SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    #GPIO.setup(BUZZER_A_PIN, GPIO.OUT)
    #GPIO.setup(BUZZER_B_PIN, GPIO.OUT)
    #GPIO.output(BUZZER_A_PIN, GPIO.LOW)
    #GPIO.output(BUZZER_B_PIN, GPIO.LOW)
    #print("GPIO initialized successfully.")
# --------------------------
# HT16K33 7-segment driver
# --------------------------
HEX_DIGITS                  = [0x3f, 0x06, 0x5b, 0x4f,     # 0, 1, 2, 3
                               0x66, 0x6d, 0x7d, 0x07,     # 4, 5, 6, 7
                               0x7f, 0x6f, 0x77, 0x7c,     # 8, 9, A, b
                               0x39, 0x5e, 0x79, 0x71]     # C, d, E, F

LETTERS                     = { "a" : 0x77, "A" : 0x77,    # "A"
                                "b" : 0x7c, "B" : 0x7c,    # "b"
                                "c" : 0x58, "C" : 0x39,    # "c", "C"
                                "d" : 0x5e, "D" : 0x5e,    # "d"
                                "e" : 0x79, "E" : 0x79,    # "E"
                                "f" : 0x71, "F" : 0x71,    # "F"
                                "g" : 0x6F, "G" : 0x6F,    # "g"
                                "h" : 0x74, "H" : 0x76,    # "h", "H"
                                "i" : 0x04, "I" : 0x30,    # "i", "I"
                                "j" : 0x0e, "J" : 0x0e,    # "J"
# Cannot be implemented         "k" : None, "K" : None,    
                                "l" : 0x38, "L" : 0x38,    # "L"
# Cannot be implemented         "m" : None, "M" : None,    
                                "n" : 0x54, "N" : 0x54,    # "n"
                                "o" : 0x5c, "O" : 0x3f,    # "o", "O"
                                "p" : 0x73, "P" : 0x73,    # "P"
                                "q" : 0x67, "Q" : 0x67,    # "q"
                                "r" : 0x50, "R" : 0x50,    # "r"
                                "s" : 0x6D, "S" : 0x6D,    # "S"
                                "t" : 0x78, "T" : 0x78,    # "t"
                                "u" : 0x1c, "U" : 0x3e,    # "u", "U"
# Cannot be implemented         "v" : None, "V" : None,    
# Cannot be implemented         "w" : None, "W" : None,    
# Cannot be implemented         "x" : None, "X" : None,    
                                "y" : 0x6e, "Y" : 0x6e,    # "y"
# Cannot be implemented         "z" : None, "Z" : None,    
                                " " : 0x00,                # " "
                                "-" : 0x40,                # "-"
                                "0" : 0x3f,                # "0"
                                "1" : 0x06,                # "1"
                                "2" : 0x5b,                # "2"
                                "3" : 0x4f,                # "3"
                                "4" : 0x66,                # "4"
                                "5" : 0x6d,                # "5"
                                "6" : 0x7d,                # "6"
                                "7" : 0x07,                # "7"
                                "8" : 0x7f,                # "8"
                                "9" : 0x6f,                # "9"
                                "?" : 0x53                 # "?"
                              }                               

CLEAR_DIGIT                 = 0x7F
POINT_VALUE                 = 0x80

DIGIT_ADDR                  = [0x00, 0x02, 0x06, 0x08]
COLON_ADDR                  = 0x04

HT16K33_BLINK_CMD           = 0x80
HT16K33_BLINK_DISPLAYON     = 0x01
HT16K33_BLINK_OFF           = 0x00
HT16K33_BLINK_2HZ           = 0x02
HT16K33_BLINK_1HZ           = 0x04
HT16K33_BLINK_HALFHZ        = 0x06

HT16K33_SYSTEM_SETUP        = 0x20
HT16K33_OSCILLATOR          = 0x01

HT16K33_BRIGHTNESS_CMD      = 0xE0
HT16K33_BRIGHTNESS_HIGHEST  = 0x0F
HT16K33_BRIGHTNESS_DARKEST  = 0x00

# Maximum decimal value that can be displayed on 4 digit Hex Display
HT16K33_MAX_VALUE           = 9999


# ------------------------------------------------------------------------
# Functions / Classes
# ------------------------------------------------------------------------
'''
class HT16K33():
    """ Class to manage a HT16K33 I2C display """
    bus     = None
    address = None
    command = None
    
    def __init__(self, bus, address=0x70, blink=HT16K33_BLINK_OFF, brightness=HT16K33_BRIGHTNESS_HIGHEST):
        """ Initialize class variables; Set up display; Set display to blank """
        
        # Initialize class variables
        self.bus     = bus
        self.address = address
        self.command = "/usr/sbin/i2cset -y {0} {1}".format(bus, address)

        # Set up display        
        self.setup(blink, brightness)
        
        # Set display to blank
        self.blank()
    
    # End def
    
    
    def setup(self, blink, brightness):
        """Initialize the display itself"""
        # i2cset -y 1 0x70 0x21
        os.system("{0} {1}".format(self.command, (HT16K33_SYSTEM_SETUP | HT16K33_OSCILLATOR)))
        # i2cset -y 1 0x70 0x81
        os.system("{0} {1}".format(self.command, (HT16K33_BLINK_CMD | blink | HT16K33_BLINK_DISPLAYON)))
        # i2cset -y 1 0x70 0xEF
        os.system("{0} {1}".format(self.command, (HT16K33_BRIGHTNESS_CMD | brightness)))

    # End def    


    def encode(self, data, double_point=False):
        """Encode data to TM1637 format.
        
        This function will convert the data from decimal to the TM1637 data fromt
        
        :param value: Value must be between 0 and 15
        
        Will throw a ValueError if number is not between 0 and 15.
        """
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

    # End def


    def set_digit(self, digit_number, data, double_point=False):
        """Update the given digit of the display."""
        os.system("{0} {1} {2}".format(self.command, DIGIT_ADDR[digit_number], self.encode(data, double_point)))    

    # End def


    def set_digit_raw(self, digit_number, data, double_point=False):
        """Update the given digit of the display using raw data value"""
        os.system("{0} {1} {2}".format(self.command, DIGIT_ADDR[digit_number], data))    

    # End def


    def set_colon(self, enable):
        """Set the colon on the display."""
        if enable:
            os.system("{0} {1} {2}".format(self.command, COLON_ADDR, 0x02))
        else:
            os.system("{0} {1} {2}".format(self.command, COLON_ADDR, 0x00))

    # End def        


    def blank(self):
        """Clear the display to read nothing"""
        self.set_colon(False)

        self.set_digit_raw(3, 0x00)
        self.set_digit_raw(2, 0x00)
        self.set_digit_raw(1, 0x00)
        self.set_digit_raw(0, 0x00)

    # End def


    def clear(self):
        """Clear the display to read '0000'"""
        self.set_colon(False)
        self.update(0)

    # End def


    def update(self, value):
        """Update the value on the display.  
        
        This function will clear the display and then set the appropriate digits
        
        :param value: Value must be between 0 and 9999.
        
        Will throw a ValueError if number is not between 0 and 9999.
        """
        if ((value < 0) or (value > 9999)):
            raise ValueError("Value is not between 0 and 9999")
        
        self.set_digit(3, (value % 10))
        self.set_digit(2, (value // 10) % 10)
        self.set_digit(1, (value // 100) % 10)
        self.set_digit(0, (value // 1000) % 10)

    # End def
    
    def text(self, value):
        """ Update the value on the display with text
        
        :param value:  Value must have between 1 and 4 characters
        
        Will throw a ValueError if there are not the appropriate number of 
        characters or if characters are used that are not supported.
        """
        if ((len(value) < 1) or (len(value) > 4)):
            raise ValueError("Must have between 1 and 4 characters")        
        
        # Clear the display
        self.blank()

        # Set the display to the correct characters        
        for i, char in enumerate(value):
            try:
                char_value = LETTERS[char]
                self.set_digit_raw(i, char_value)
            except:
                raise ValueError("Character {0} not supported".format(char))

# End class
SevenSegHT16K33 = HT16K33
'''

class SevenSegHT16K33:
    DIGITS = [0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F]
    def __init__(self, address, busnum=1, brightness=15):
        self.addr = address
        self.bus = smbus.SMBus(busnum)
        self.bus.write_byte(self.addr, 0x21)  # system on
        self.bus.write_byte(self.addr, 0x81)  # display on
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
