#!/bin/sh

#TBD : Generate a log file in /var/log. If it exists then do not program the AVR. Can then run this script every boot


if [ -f "/var/log/PPi_startup_check.log" ]
then
	echo "Not first boot"
	echo "AVR programmed"



else
	echo "First boot ..."


	echo "Programming AVR ... "
	touch /var/log/PPi_startup_check.log

	#AVR software location, compile and load
	cd /home/pi/PenguinPi/EGB439/atmelStudio/V2.0/

	#AVR fuse programming on first boot
	sudo avrdude -c pi_isp -p m644p -U lfuse:w:0xFF:m
	sudo avrdude -c pi_isp -p m644p -U hfuse:w:0xD9:m
	sudo avrdude -c pi_isp -p m644p -U efuse:w:0xFF:m

	#Compile and load
	make clean
	make
	make load

fi



