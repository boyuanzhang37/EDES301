import os
import time
import threading
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC

#Quick pin config
LED_PINS = ["P2_02","P2_04","P2_06","P2_08","P2_18","P2_20","P2_22","P2_24"]
JOY_VR_ADC_CHANNEL = 0   # AIN0 -> P1_19
JOY_SW_PIN = "P2_10"
BUZZER_A_PIN = "P2_01"
BUZZER_B_PIN = "P2_03"
HT16K33_BUS = 1
HT16K33_ADDR = 0x70

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

# ============================================================
#  JOYSTICK DRIVER (Y-axis only, using ADC)
# ============================================================
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import time

class Joystick:
    """
    Reads:
      - Analog Y axis (0–1.0)
      - Digital press button (active low)
    """
    def __init__(self, adc_pin, sw_pin,
                 left_thresh=0.35, right_thresh=0.65):
        self.adc_pin = adc_pin
        self.sw_pin = sw_pin
        self.left_thresh = left_thresh
        self.right_thresh = right_thresh

        ADC.setup()
        GPIO.setup(sw_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read_axis(self):
        """Return -1 (left), 0 (center), +1 (right)."""
        val = ADC.read(self.adc_pin)

        if val is None:
            return 0

        if val < self.left_thresh:
            return -1
        elif val > self.right_thresh:
            return 1
        else:
            return 0

    def pressed(self):
        """Return True when joystick button is pressed."""
        return GPIO.input(self.sw_pin) == 0

    def press_duration(self):
        """
        Block until button released.
        Returns duration of press in seconds.
        """
        if not self.pressed():
            return 0

        start = time.time()
        while self.pressed():
            time.sleep(0.01)
        return time.time() - start

# --------------------------
# End drivers insertion
# --------------------------


# ============================================================
#  GAME ENGINE
# ============================================================
class RhythmGame:
    """
    Main game logic.
    - Display BPM slider
    - Wait for joystick press to start game
    - Show LED patterns
    - Play tones
    """

    def __init__(self, leds, buzzer_tempo, buzzer_error, display, joystick):
        self.leds = leds
        self.buzzer_tempo = buzzer_tempo
        self.buzzer_error = buzzer_error
        self.display = display
        self.joystick = joystick

        self.running = False
        self.abort_flag = False

        # BPM selection limits
        self.bpm = 150
        self.bpm_min = 60
        self.bpm_max = 300

    # --------------------------------------------------------
    # Utility: turn off all LEDs
    # --------------------------------------------------------
    def clear_leds(self):
        for led in self.leds:
            led.off()

    # --------------------------------------------------------
    # Convert integer pattern to LED states
    # --------------------------------------------------------
    def led_pattern(self, pattern):
        bit_index = 0
        for led in self.leds:
            if pattern & (1 << bit_index):
                led.on()
            else:
                led.off()
            bit_index += 1

     # --------------------------------------------------------
    # UI Loop – select BPM using joystick left/right + press
    # --------------------------------------------------------
    def bpm_selection_ui(self):
        print("\nUse joystick left/right to adjust BPM. Press joystick to start.")
        self.display.update(self.bpm)
        last_move = time.time()

        while True:
            axis = self.joystick.read_axis()

            # Move BPM left/right
            now = time.time()
            if axis != 0 and (now - last_move) > 0.15:
                if axis < 0:     # LEFT
                    self.bpm = max(self.bpm_min, self.bpm - 1)
                else:            # RIGHT
                    self.bpm = min(self.bpm_max, self.bpm + 1)

                self.display.update(self.bpm)
                last_move = now

            # Check press
            if self.joystick.pressed():
                dur = self.joystick.press_duration()

                if dur >= 5.0:
                    # Long hold → abort back to menu
                    print("Long press detected — abort")
                    self.abort_flag = True
                    return

                else:
                    # Short press → start game
                    print("Starting game...")
                    return

            time.sleep(0.05)


    # --------------------------------------------------------
    # Game loop – LED sequence and tones
    # --------------------------------------------------------
    def run_gameplay(self):
        beat_time = 60.0 / self.bpm

        print("\nGame started at BPM =", self.bpm)

        patterns = [
            0b00000001,
            0b00000010,
            0b00000100,
            0b00001000,
            0b00010000,
            0b00100000,
            0b01000000,
            0b10000000,
        ]

        index = 0

        while not self.abort_flag:

            pattern = patterns[index]
            self.led_pattern(pattern)

            # Tempo beep
            self.buzzer_tempo.play(880, beat_time * 0.3, stop=True)

            time.sleep(beat_time * 0.7)

            index = (index + 1) % len(patterns)

        self.clear_leds()
        self.buzzer_error.play(440, 0.5, stop=True)

    # --------------------------------------------------------
    # Main entry point
    # --------------------------------------------------------
    def run(self):
        self.abort_flag = False
        self.display.update(self.bpm)

        self.bpm_selection_ui()

        if self.abort_flag:
            print("\nGame aborted.")
            return

        self.run_gameplay()

    # --------------------------------------------------------
    # Cleanup on exit
    # --------------------------------------------------------
    def cleanup(self):
        self.clear_leds()
        self.joystick.cleanup()
        self.display.blank()
        self.buzzer_tempo.cleanup()
        self.buzzer_error.cleanup()


def main():
    leds, buzzer_tempo, buzzer_error, display = configure_pins_and_init()
    leds = [
        LED("P2_02"), LED("P2_04"), LED("P2_06"), LED("P2_08"),
        LED("P2_18"), LED("P2_20"), LED("P2_22"), LED("P2_24")
    ]

    buzzer_tempo = Buzzer("P2_01")
    buzzer_error = Buzzer("P2_03")

    display = HT16K33(bus=1)

    joystick = Joystick(
        adc_pin="P1_19",
        sw_pin="P2_10"
    )

    game = RhythmGame(leds, buzzer_tempo, buzzer_error, display, joystick)

    try:
        game.run()
    except KeyboardInterrupt:
        pass
    finally:
        game.cleanup()


if __name__ == "__main__":
    main()
