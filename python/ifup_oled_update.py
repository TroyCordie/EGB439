#!/usr/bin/python3

'''
PenguinPi.py test script
'''

import os
import penguinPi as ppi


#initialise serial, and retrieve initial values from the Atmega
ppi.init()

oled = ppi.OLED( ppi.AD_OLED )

#This works but fails if network not present ie split has no elements
#ip_addr_eth0_str  = os.popen('ip addr show eth0').read().split("inet ")[1].split("/")[0]
#ip_addr_wlan0_str = os.popen('ip addr show wlan0').read().split("inet ")[1].split("/")[0]

ip_addr_eth0_str   = os.popen('ip addr show eth0').read()
ip_addr_wlan0_str  = os.popen('ip addr show wlan0').read()


if ip_addr_eth0_str.find("inet ") != -1:
    ip_addr_eth0_str   = ip_addr_eth0_str.split("inet ")[1].split("/")[0]
else:
    ip_addr_eth0_str   = "0.0.0.0"

if ip_addr_wlan0_str.find("inet ") != -1:
    ip_addr_wlan0_str  = ip_addr_wlan0_str.split("inet ")[1].split("/")[0]
else:
    ip_addr_wlan0_str   = "0.0.0.0"

print ( "ETH0 Address is ", ip_addr_eth0_str )
print ( "WLAN0 Address is ", ip_addr_wlan0_str )

oled.set_ip_eth ( ip_addr_eth0_str )
oled.set_ip_wlan( ip_addr_wlan0_str )


