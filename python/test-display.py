#!/usr/bin/python3

'''
PenguinPi.py test script
'''

import penguinPi as ppi
import time

display = ppi.Display(ppi.AD_DISPLAY_A)

#initialise serial, and retrieve initial values from the Atmega
ppi.init()

display.get_all()

display.set_mode('x')
for i in range(0,255):
    display.set_value(i)
    time.sleep(0.1)

display.set_mode('u')
for i in range(0,99):
    display.set_value(i)
    time.sleep(0.1)

display.set_mode('d')
for i in range(10,-11,-1):
    display.set_value(i)
    time.sleep(0.1)

ppi.close()
