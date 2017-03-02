main.hex: main.c
	avr-gcc -O2  -o main.elf -mmcu=avr5 -D__AVR_ATmega328PB__ -Ilib main.c lib/twimaster.o  lib/uart.o
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

load:
	avrdude -c avrisp2 -p ATmega328P -F -U flash:w:main.hex
