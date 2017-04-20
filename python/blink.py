#!usr/bin/python3

import time
import penguinPi as ppi

ppi.init()
led = ppi.LED(ppi.AD_LED_R)

while True:
    led.set_state(1);
    led.set_count(5000);
    time.sleep(2);
