import time
import Adafruit_BBIO.PWM as PWM
PWM.start("P2_01", 50, 440)  # 1 kHz
time.sleep(2)
PWM.stop("P2_01")
PWM.cleanup()
