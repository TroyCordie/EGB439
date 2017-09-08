/*
 * main.c
 *
 * NOTES
 * ========
 *
 * Motor, set_power() invokes MOTOR_SET_SPEED_DPS which sets motor->dir and motor->setSpeedDPS
 * set_degrees() invokes MOTOR_SET_DEGREES which sets motor->setDegrees which is the PID setpoint
 *
 * LED usage:
 *  red 	- valid command packet
 *  green 	- encoder ISR
 *  blue 	- bad command or framing error
 */

/*
 * cpu is ATmega644PA
 */
#define DEBUG 1
 
#define BAUD  115200
#define F_CPU 8000000UL

#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <util/crc16.h>
#include "i2cmaster.h"
#include "uart.h"
#include "PenguinPi.h"
#include "PCA6416A.h"



Motor 		motorA;
Motor 		motorB;
//Servo 		servoA;			//DO WE NEED THESE
//Servo 		servoB;			//DO WE NEED THESE
LED 		ledR;
LED 		ledG;
LED 		ledB;
Display 	displayA;
Button 		buttonA;
Button 		buttonB;
Button 		buttonC;
AnalogIn 	vdiv;
AnalogIn 	csense;

//Global variables
char 	fstring[32];
uint8_t datagramG[DGRAM_MAX_LENGTH+1];
int8_t 	mAQuadTable[4][4] = {{ 0, +1, -1,  2},
							{-1,  0,  2, +1},
							{+1,  2,  0, -1},
							{ 2, -1, +1,  0}};

int8_t mBQuadTable[4][4] = {{ 0, -1, +1,  2},
							{+1,  0,  2, -1},
							{-1,  2,  0, +1},
							{ 2, +1, -1,  0}};
//digit strings for display: includes hexadecimal only
uint8_t digit0[] = {DIGIT0_0, DIGIT0_1, DIGIT0_2, DIGIT0_3, DIGIT0_4, DIGIT0_5, DIGIT0_6, DIGIT0_7, DIGIT0_8, DIGIT0_9, DIGIT0_A, DIGIT0_B, DIGIT0_C, DIGIT0_D, DIGIT0_E, DIGIT0_F};
uint8_t digit1[] = {DIGIT1_0, DIGIT1_1, DIGIT1_2, DIGIT1_3, DIGIT1_4, DIGIT1_5, DIGIT1_6, DIGIT1_7, DIGIT1_8, DIGIT1_9, DIGIT1_A, DIGIT1_B, DIGIT1_C, DIGIT1_D, DIGIT1_E, DIGIT1_F};
//Battery struct
struct Battery {
	float cutoff;//volts
	uint16_t count;
	uint16_t limit;//number of cycles until it triggers a shutdown
} battery;

































//V2.0	ISR(PCINT1_vect){
//V2.0		//Motor A encoders
//V2.0		//detect which pin triggered the interrupt
//V2.0		uint8_t enc1val = (PINC & (1<<MOTOR_A_ENC_1))>>MOTOR_A_ENC_1; //store the current state
//V2.0		uint8_t enc2val = (PINC & (1<<MOTOR_A_ENC_2))>>MOTOR_A_ENC_2;
//V2.0		
//V2.0		if(motorA.encoderMode == 0){
//V2.0			//single encoder mode, on pin 1
//V2.0			//uint8_t encdiff = motorA.enc1PinState ^ enc1val;
//V2.0			if(motorA.enc1PinState ^ enc1val){
//V2.0				if(motorA.dir == 1){
//V2.0					motorA.position++;
//V2.0					motorA.lastDir = 1;
//V2.0				}else if(motorA.dir == -1){
//V2.0					motorA.position--;
//V2.0					motorA.lastDir = -1;
//V2.0				}else{
//V2.0					//wheel slip!!
//V2.0					//probably going to still be rotating in the direction it just was, so use that past value
//V2.0					if(motorA.lastDir == 1) motorA.position++;
//V2.0					else if(motorA.lastDir == -1) motorA.position--;
//V2.0				}
//V2.0				motorA.enc1PinState = enc1val;
//V2.0			}//otherwise there was a tick but it wasn't the first channel...
//V2.0	
//V2.0		}else if(motorA.encoderMode == 1){
//V2.0			//standard quadrature
//V2.0			uint8_t lastEncSum = (motorA.enc1PinState<<1)|(motorA.enc2PinState);
//V2.0			uint8_t encSum = (enc1val<<1)|(enc2val);
//V2.0			int8_t effect = mAQuadTable[lastEncSum][encSum];
//V2.0	
//V2.0			motorA.position += effect;
//V2.0	
//V2.0			motorA.enc1PinState = enc1val;
//V2.0			motorA.enc2PinState = enc2val;
//V2.0	
//V2.0		}else if(motorA.encoderMode == 2){
//V2.0			//x4 counting (xor'ed both channels)
//V2.0			uint8_t x4 = enc1val ^ enc2val;
//V2.0			if(motorA.enc1PinState ^ x4){
//V2.0				if(motorA.dir == 1){
//V2.0					motorA.position++;
//V2.0					motorA.lastDir = 1;
//V2.0				}else if(motorA.dir == -1){
//V2.0					motorA.position--;
//V2.0					motorA.lastDir = -1;
//V2.0				}else{
//V2.0					//wheel slip!!
//V2.0					//probably going to still be rotating in the direction it just was, so use that past value
//V2.0					if(motorA.lastDir == 1) motorA.position++;
//V2.0					else if(motorA.lastDir == -1) motorA.position--;
//V2.0				}
//V2.0				motorA.enc1PinState = x4;
//V2.0			}
//V2.0		}else{
//V2.0			//my mode isn't specified
//V2.0			motorA.encoderMode = 1;//set to default
//V2.0		}
//V2.0	
//V2.0		ledG.state = 1;
//V2.0		ledG.count = 100;
//V2.0	}
//V2.0	
//V2.0	ISR(PCINT3_vect){
//V2.0		//Motor B encoders
//V2.0		//detect which pin triggered the interrupt
//V2.0		uint8_t enc1val = (PINE & (1<<MOTOR_B_ENC_1))>>MOTOR_B_ENC_1; //store the current state
//V2.0		uint8_t enc2val = (PINE & (1<<MOTOR_B_ENC_2))>>MOTOR_B_ENC_2;
//V2.0		
//V2.0		if(motorB.encoderMode == 0){
//V2.0			//single encoder mode, on pin 1
//V2.0			//uint8_t encdiff = motorB.enc1PinState ^ enc1val;
//V2.0			if(motorB.enc1PinState ^ enc1val){
//V2.0				if(motorB.dir == 1){
//V2.0					motorB.position++;
//V2.0					motorB.lastDir = 1;
//V2.0				}else if(motorB.dir == -1){
//V2.0					motorB.position--;
//V2.0					motorB.lastDir = -1;
//V2.0				}else{
//V2.0					//wheel slip!!
//V2.0					//probably going to still be rotating in the direction it just was, so use that past value
//V2.0					if(motorB.lastDir == 1) motorB.position++;
//V2.0					else if(motorB.lastDir == -1) motorB.position--;
//V2.0				}
//V2.0				motorB.enc1PinState = enc1val;
//V2.0			}//otherwise there was a tick but it wasn't the first channel...
//V2.0	
//V2.0		}else if(motorB.encoderMode == 1){
//V2.0			//standard quadrature
//V2.0			uint8_t lastEncSum = (motorB.enc1PinState<<1)|(motorB.enc2PinState);
//V2.0			uint8_t encSum = (enc1val<<1)|(enc2val);
//V2.0			int8_t effect = mBQuadTable[lastEncSum][encSum];
//V2.0	
//V2.0			motorB.position += effect;
//V2.0	
//V2.0			motorB.enc1PinState = enc1val;
//V2.0			motorB.enc2PinState = enc2val;
//V2.0	
//V2.0		}else if(motorB.encoderMode == 2){
//V2.0			//x4 counting (xor'ed both channels)
//V2.0			uint8_t x4 = enc1val ^ enc2val;
//V2.0			if(motorB.enc1PinState ^ x4){
//V2.0				if(motorB.dir == 1){
//V2.0					motorB.position++;
//V2.0					motorB.lastDir = 1;
//V2.0				}else if(motorB.dir == -1){
//V2.0					motorB.position--;
//V2.0					motorB.lastDir = -1;
//V2.0				}else{
//V2.0					//wheel slip!!
//V2.0					//probably going to still be rotating in the direction it just was, so use that past value
//V2.0					if(motorB.lastDir == 1) motorB.position++;
//V2.0					else if(motorB.lastDir == -1) motorB.position--;
//V2.0				}
//V2.0				motorB.enc1PinState = x4;
//V2.0			}
//V2.0		}else{
//V2.0			//my mode isn't specified
//V2.0			motorB.encoderMode = 1;//set to default
//V2.0		}
//V2.0	
//V2.0		ledG.state = 1;
//V2.0		ledG.count = 100;
//V2.0	}
//V2.0	
//V2.0	ISR(PCINT0_vect){
//V2.0		uint8_t btnAval = (PINB & (1<<BTN_A))>>BTN_A;
//V2.0	
//V2.0		buttonLogic(&buttonA, btnAval);
//V2.0	}
//V2.0	
//V2.0	ISR(PCINT2_vect){
//V2.0		//determine which button triggered the interrupt
//V2.0		uint8_t btnBval = (PIND & (1<<BTN_B))>>BTN_B;
//V2.0		uint8_t btnCval = (PIND & (1<<BTN_C))>>BTN_C;
//V2.0	
//V2.0		if(buttonB.pinState ^ btnBval){//has it actually changed?
//V2.0			buttonLogic(&buttonB, btnBval);
//V2.0		}
//V2.0		if(buttonC.pinState ^ btnCval){
//V2.0			buttonLogic(&buttonC, btnCval);
//V2.0		}
//V2.0	}

ISR(TIMER2_OVF_vect){							//period of 21.3333us. use a counter for longer delays
	static uint8_t motorControlCount = 0;
	
	if(motorControlCount < CONTROL_COUNT) {		//should achieve a period of 64 us
		motorControlCount++;
	}
	else {
		//set PID flags
		motorA.pidTimerFlag = 1;
		motorB.pidTimerFlag = 1;
		motorControlCount   = 0;
	}
	
	if(ledR.count > 0) 	ledR.count--;
	else 				ledR.state = 0;
	
	if(ledG.count > 0) 	ledG.count--;
	else 				ledG.state = 0;
	
	if(ledB.count > 0) 	ledB.count--;
	else 				ledB.state = 0;
	
//DELETE	if(buttonA.debounceCount > 0) buttonA.debounceCount--;
//DELETE	if(buttonB.debounceCount > 0) buttonB.debounceCount--;
//DELETE	if(buttonC.debounceCount > 0) buttonC.debounceCount--;
}

ISR(ADC_vect) {								//period of 69.3333us for a standard ADC read, 2x longer for first
	if(vdiv.count > 1){
		vdiv.count--;						//really this is just a counter to get a few readings before actually using the ADC value
		ADCSRA |= (1<<ADSC);
	}
	else if(vdiv.count == 1){
		vdiv.ready = 1;
		vdiv.count = 0;
	}
	if(csense.count > 1){
		csense.count--;
		ADCSRA |= (1<<ADSC);
	}
	else if(csense.count == 1){
		csense.ready = 1;
		csense.count = 0;
	}
}





void init_structs(void){
	float kP = 70.0;
	float kI = 0.0075;
	float kD = 2.0;

	motorA.gainP = kP*PID_SCALE;
	motorA.gainI = kI*PID_SCALE;
	motorA.gainD = kD*PID_SCALE;
	motorA.maxError = INT16_MAX / (motorA.gainP + 1);
	motorA.maxErrorSum = (INT32_MAX / 2) / (motorA.gainI + 1);
	motorA.encoderMode = 1;

	motorB.gainP = kP*PID_SCALE;
	motorB.gainI = kI*PID_SCALE;
	motorB.gainD = kD*PID_SCALE;
	motorB.maxError = INT16_MAX / (motorB.gainP + 1);
	motorB.maxErrorSum = (INT32_MAX / 2) / (motorB.gainI + 1);
	motorB.encoderMode = 1;

//DELETE	servoA.setPos = 90;
//DELETE	servoA.minRange = 0;
//DELETE	servoA.maxRange= 180;
//DELETE	servoA.minPWM = SERVO_PWM_RANGE_MIN;
//DELETE	servoA.maxPWM = SERVO_PWM_RANGE_MAX;

//DELETE	servoB.setPos= 90;
//DELETE	servoB.minRange = 0;
//DELETE	servoB.maxRange= 180;
//DELETE	servoB.minPWM = SERVO_PWM_RANGE_MIN;
//DELETE	servoB.maxPWM = SERVO_PWM_RANGE_MAX;

	displayA.address = PCA6416A_0;
	displayA.value= 0;
	displayA.digit0 = 0;
	displayA.digit1 = 0;
	displayA.draw = 0;

	buttonA.pinMode = 0;
	buttonB.pinMode = 0;
	buttonC.pinMode = 0;
	buttonA.programMode = 0;
	buttonB.programMode = 0;
	buttonC.programMode = 0;

	vdiv.count = 0;
	vdiv.scale = 16.018497;//mV per div
	csense.count = 0;
	csense.scale = 3.2226562;//mA per div

	battery.cutoff = 6.5;
	battery.count= 0;
	battery.limit = 5000;//about 1.5 seconds
}

void init(void){
//V2.0		//Power reduction register
//V2.0		PRR0 &= ~((1<<PRTWI0)|(1<<PRTIM2)|(1<<PRTIM0)|(1<<PRUSART1)|(1<<PRTIM1)|(1<<PRADC));

	//Motor Pins
		DDRB |= (1<<MOTOR_A_PWM)|(1<<MOTOR_B_PWM);
		DDRB |= (1<<MOTOR_A_PHA)|(1<<MOTOR_B_PHA);

	//Motor PWM
		//V1 was OC1A and OC1B
		//V2  is OC0A and OC0B
		TCCR0A |= (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10); 	// Non-inverting, 16 bit fast PWM
		TCCR0B |= (1<<WGM13)|(1<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10); 						// DIV1 prescaler
		ICR1 	= 0xFFFF; 																	//set TOP

	//Encoders
		//V1 was PC2, PC3, PE0, PE1 ... PCINT 10,11,24,25
		//V2  is PA0, PA1, PA2, PA3 ... PCINT 0:3
		DDRA 	&= ~((1<<MOTOR_A_ENC_1)|(1<<MOTOR_A_ENC_2)|(1<<MOTOR_B_ENC_1)|(1<<MOTOR_B_ENC_2));		//Direction to INPUT
		PORTA 	|=   (1<<MOTOR_A_ENC_1)|(1<<MOTOR_A_ENC_2)|(1<<MOTOR_B_ENC_1)|(1<<MOTOR_B_ENC_2);		//Enable internal PULLUPs
		PCMSK0   =   (1<<PCINT0)|(1<<PCINT1)|(1<<PCINT2)|(1<<PCINT3);									//Enable Pin Change Mask
		PCICR 	|=   (1<<PCIE0);																		//Enable interrupt on pin change
		
//DELETE	//Servo
//DELETE	DDRD |= (1<<SERVO_A)|(1<<SERVO_B);
//DELETE	TCCR3A |= (1<<COM3A1)|(0<<COM3A0)|(1<<WGM31)|(0<<WGM30); // non-inverting, 15 bit resolution fast PWM
//DELETE	TCCR3B |= (1<<WGM33)|(1<<WGM32)|(0<<CS32)|(1<<CS31)|(0<<CS30); // DIV8 prescaler, TOP 20,000 to run at 50 Hz for 20ms pulse
//DELETE	ICR3 = 0x7530;

//DELETE	TCCR4A |= (1<<COM4A1)|(0<<COM4A0)|(1<<WGM41)|(0<<WGM40); // non-inverting, 15 bit resolution fast PWM
//DELETE	TCCR4B |= (1<<WGM43)|(1<<WGM42)|(0<<CS42)|(1<<CS41)|(0<<CS40);	// DIV8 prescaler	
//DELETE	ICR4 = 0x7530;
	
	//LED's
		DDRC |= 0x0C;	//LEDs on C3:2
		DDRD |= 0xE0;	//RGB on D7:5
	
	
	//LEDR	was OC0A	now OC2A
	//LEDG	was OC0B	now OC1A
	//LEDB	was OC2B	now OC2B
	
	//GREEN ON A
		TCCR1A |= (1<<COM0A1)|(1<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(1<<WGM00); 			// inverting, 8 bit fast PWM
		TCCR1B |= (0<<WGM02)|(0<<CS02)|(0<<CS01)|(1<<CS00);
		
	//RED on A and BLUE on B
		TCCR2A |= (1<<COM2A1)|(1<<COM2A0)|(1<<COM2B1)|(1<<COM2B0)|(1<<WGM21)|(1<<WGM20); 			// inverting, 8 bit fast PWM
		TCCR2B |= (0<<WGM22)|(0<<CS22)|(0<<CS21)|(1<<CS20);
		
//V2.0		//Buttons (external pullups)
//V2.0		DDRB &= ~(1<<BTN_A);
//V2.0		PCMSK0 |= (1<<PCINT0);
//V2.0	
//V2.0		DDRD &= ~(1<<BTN_B);
//V2.0		PCMSK2 |= (1<<PCINT23);
//V2.0		
//V2.0		DDRD &= ~(1<<BTN_C);
//V2.0		PCMSK2 |= (1<<PCINT20);
//V2.0		PCICR |= (1<<PCIE0)|(1<<PCIE2);
//V2.0	
//V2.0		//Pi Interrupt
//V2.0		DDRB |= (1<<PB5);
//V2.0		PORTB |= (1<<PB5);

	//8 bit controller timer
		TIMSK2 |= (1<<TOIE2);

	//ADC
		DIDR0	= (1<<ADC6D)|(1<<ADC7D);
		ADMUX 	= (0<<REFS1)|(1<<REFS0)|(0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0); 	//AVCC reference voltage with external cap on AREF, multiplex to channel 6
		ADCSRA 	= (1<<ADEN) |(1<<ADIE) |(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);					//Enable ADC, enable interrupt, prescaler of 64 (187.5kHz sample rate)
//This access at mo puts device into a loop cycle												

	uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(BAUD, F_CPU));
	
	i2c_init();

	sei();
}









uint8_t checkBuffer(void){
	uint16_t com = uart_getc();
	
	if(com & UART_NO_DATA){
		// there's no data available
		return 0;
	} else {
		//check for errors
        if(com & UART_FRAME_ERROR){
            /* Framing Error detected, i.e no stop bit detected */
            uart_puts_P("ERROR: Bad UART Frame\n");
			//flash Blue LED
			ledB.state = 1;
			ledB.count = 1000;
			return 0;
        }
        if(com & UART_OVERRUN_ERROR){
            /* 
                * Overrun, a character already present in the UART UDR register was 
                * not read by the interrupt handler before the next character arrived,
                * one or more received characters have been dropped
                */
            uart_puts_P("ERROR: UART Buffer Overrun\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
			return 0;
        }
        if(com & UART_BUFFER_OVERFLOW){
            /* 
                * We are not reading the receive buffer fast enough,
                * one or more received character have been dropped 
                */
            uart_puts_P("ERROR: UART Buffer Overflow\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
			return 0;
        }
		return com & 0xFF;//return lowbyte
	}
}

void formdatagram(uint8_t *datagram, uint8_t address, uint8_t opCode, union dgramMem payl, uint8_t type){
	// datagrams : length, address, opCode, payload[1-4], crc
	datagram[1] = address;
	datagram[2] = opCode; 
	uint8_t paylen;
	switch(type){
		case 'C':
		case 'c':;
			datagram[3] = payl.ch;
			paylen = 1;
			
		break;
		case 'I':
		case 'i':;
			datagram[3] = (payl.in >> 8);
			datagram[4] = payl.in & 0xFF;
			paylen = 2;
		break;
		case 'f':;
			uint8_t *fl = float2char(payl.fl);
			for(uint8_t i = 0; i < 4; i++){
				datagram[3+i] = fl[3-i];//avr stores as little endian
			}
			paylen = 4;
		break;
		default:
			paylen = 0;
		break;
	}
	datagram[0] = 4 + paylen;
	datagram[3+paylen] = crc8(datagram, datagram[0]-1);
}

void parseDatagram(uint8_t *datagram){
	//flash RED LED
	ledR.state = 1;
	ledR.count = 1000;
	
	_delay_us(UART_INTERBYTE_WAIT);
	uint8_t dlen = checkBuffer();
	datagram[0] = dlen; //length of the datagram
	for(uint8_t i = 1; i < dlen; i++){
		_delay_us(UART_INTERBYTE_WAIT);
		
		datagram[i] = checkBuffer();
		if(i >= DGRAM_MAX_LENGTH){
			uart_puts_P("ERROR: Datagram Buffer Overflow\n");
			ledB.state = 1;
			ledB.count = 1000;
			return;
		}
	}
	uint8_t crcDgram = datagram[dlen-1];
	datagram[dlen-1] = 0;
	uint8_t crcCalc = crc8(datagram, dlen-1);
	datagram[0] -= 1;	

	if(crcCalc != crcDgram){
		uart_puts_P("ERROR: CRC Failed\n");
		ledB.state = 1;
		ledB.count = 1000;
		return;
	}
	
	switch(datagram[1]){
		case AD_MOTOR_A:
			parseMotorOp(datagram, &motorA);
		break;
		case AD_MOTOR_B:
			parseMotorOp(datagram, &motorB);
		break;
		
//DELETE		case AD_SERVO_A:
//DELETE			parseServoOp(datagram, &servoA);
//DELETE		break;
//DELETE		case AD_SERVO_B:
//DELETE			parseServoOp(datagram, &servoB);
//DELETE		break;
		
		case AD_LED_R:
			parseLEDOp(datagram, &ledR);
		break;
		case AD_LED_G:
			parseLEDOp(datagram, &ledG);
		break;
		case AD_LED_B:
			parseLEDOp(datagram, &ledB);
		break;
		
		case AD_DISPLAY_A:
			parseDisplayOp(datagram, &displayA);
		break;

		case AD_BTN_A:
			parseButtonOp(datagram, &buttonA);
		break;
		case AD_BTN_B:
			parseButtonOp(datagram, &buttonB);
		break;
		case AD_BTN_C:
			parseButtonOp(datagram, &buttonC);
		break;

		case AD_ADC_V:
			parseADCOp(datagram, &vdiv);
		break;
		case AD_ADC_C:
			parseADCOp(datagram, &csense);
		break;

		case AD_ALL:
			parseAllOp(datagram);
		break;
		
		default:
			uart_puts_P("ERROR: Unknown Address\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

/* TODO:
  change the constant in datagram[0] test to a symbolic value
   ISINT
   ISFLOAT
   ISCHAR  etc
 */
void parseMotorOp(uint8_t *datagram, Motor *motor){
	switch(datagram[2]){
		//SETTERS
		case MOTOR_SET_SPEED_DPS:
			if(datagram[0] == 5){
				int16_t speed = (datagram[3]<<8) | datagram[4];
				motor->setSpeedDPS = abs(speed);
				if(speed > 0) motor->dir = 1;
				else if(speed < 0) motor->dir = -1;
				else motor->dir = 0;
				ledB.state = 1;
				ledB.count = 1000;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_DEGREES:
			if(datagram[0] == 5){
                cli();
                    motor->position = 0;
                sei();
                motor->degrees = 0;

				int16_t degrees = (datagram[3]<<8) | datagram[4];
				motor->setDegrees = degrees;
				if(degrees > 0) motor->dir = 1;
				else if(degrees < 0) motor->dir = -1;
				else motor->dir = 0;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_ENC:
            // actually does a reset
            cli();
                motor->position = 0;
            sei();
            motor->degrees = 0;
        break;
		case MOTOR_SET_DIRECTION:
			if(datagram[0] == 4){
				int8_t direction = datagram[3];
				if(direction > 0) motor->dir = 1;
				else if(direction < 0) motor->dir = -1;
				else motor->dir = 0;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_GAIN_P:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainP = readFloat(flMem) * PID_SCALE;
				motor->maxError = INT16_MAX / (motor->gainP + 1);				
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_GAIN_I:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainI = readFloat(flMem) * PID_SCALE;
				motor->maxErrorSum = (INT32_MAX / 2) / (motor->gainI + 1);
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_GAIN_D:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				motor->gainD = readFloat(flMem) * PID_SCALE;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_ENC_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				if(mode > 2) motor->encoderMode = 1; //the default
				else motor->encoderMode = mode;
				//also resets some of the motor struct
				motor->degrees = 0;
				motor->dir = 0;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case MOTOR_SET_CONTROL_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				motor->controlMode = mode;
				//also resets some of the motor struct
				motor->degrees = 0;
				motor->dir = 0;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		
		//GETTERS
		case MOTOR_GET_SPEED_DPS:
			dgrammem.in = motor->speedDPS;
			formdatagram(datagramG, datagram[1], MOTOR_SET_SPEED_DPS, dgrammem, 'i');
			uartputcs(datagramG);
		break;	
		case MOTOR_GET_DEGREES:
			dgrammem.in = motor->degrees;
			formdatagram(datagramG, datagram[1], MOTOR_SET_DEGREES, dgrammem, 'i');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_DIRECTION:
			dgrammem.ch = motor->dir;
			formdatagram(datagramG, datagram[1], MOTOR_SET_DIRECTION, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_GAIN_P:
			dgrammem.fl = motor->gainP/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_P, dgrammem, 'f');
			uartputcs(datagramG);
		break;	
		case MOTOR_GET_GAIN_I:
			dgrammem.fl = motor->gainI/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_I, dgrammem, 'f');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_GAIN_D:
			dgrammem.fl = motor->gainD/PID_SCALE;
			formdatagram(datagramG, datagram[1], MOTOR_SET_GAIN_D, dgrammem, 'f');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_ENC_MODE:
			dgrammem.ch = motor->encoderMode;
			formdatagram(datagramG, datagram[1], MOTOR_SET_ENC_MODE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_CONTROL_MODE:
			dgrammem.ch = motor->controlMode;
			formdatagram(datagramG, datagram[1], MOTOR_SET_CONTROL_MODE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case MOTOR_GET_ENC:
            cli();
                dgrammem.in = motor->position;
            sei();
			formdatagram(datagramG, datagram[1], MOTOR_SET_ENC, dgrammem, 'i');
			uartputcs(datagramG);
		break;
		
		default:
			uart_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;		
	}
}

void parseLEDOp(uint8_t *datagram, LED *led){
	switch(datagram[2]){
		//SETTERS
		case LED_SET_STATE:
			if(datagram[0] == 4){
				int8_t state = datagram[3];
				if(state >= 1) led->state = 1;
				else led->state = 0;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case LED_SET_BRIGHTNESS:
			if(datagram[0] == 4){
				int8_t brightness = datagram[3];
				if(brightness > 100) led->brightness = 100;
				else if(brightness > 0) led->brightness = brightness;
				else led->brightness = 0;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case LED_SET_COUNT:
			if(datagram[0] == 5){
				led->count = (datagram[3]<<8)|datagram[4];
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;

		//GETTERS
		case LED_GET_STATE:
			dgrammem.ch = led->state;
			formdatagram(datagramG, datagram[1], LED_SET_STATE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case LED_GET_BRIGHTNESS:
			dgrammem.ch = led->brightness;
			formdatagram(datagramG, datagram[1], LED_SET_BRIGHTNESS, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case LED_GET_COUNT:
			dgrammem.uin = led->count;
			formdatagram(datagramG, datagram[1], LED_SET_COUNT, dgrammem, 'i');
			uartputcs(datagramG);
		break;

		default:
			uart_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

void parseDisplayOp(uint8_t *datagram, Display *display){
	switch(datagram[2]){
		//SETTERS
		case DISPLAY_SET_VALUE:
			if(datagram[0] == 4){
				display->value = datagram[3];
				display->draw = 1;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case DISPLAY_SET_DIGIT_0:
			if(datagram[0] == 4){
				uint8_t digit = datagram[3];
				if(digit > 15) display->digit0 = 15;
				else if(digit < -15) display->digit0 = -15;
				else display->digit0 = digit;
				display->draw = 1;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case DISPLAY_SET_DIGIT_1:
			if(datagram[0] == 4){
				uint8_t digit = datagram[3];
				if(digit > 15) display->digit1 = 15;
				else if(digit < -15) display->digit1 = -15;
				else display->digit1 = digit;
				display->draw = 1;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
        case DISPLAY_SET_MODE:
			if(datagram[0] == 4){
				display->mode = datagram[3];
				display->draw = 1;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		
		//GETTERS
		case DISPLAY_GET_VALUE:
			dgrammem.ch = display->value;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_VALUE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case DISPLAY_GET_DIGIT_0:
			dgrammem.ch = display->digit0;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_DIGIT_0, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case DISPLAY_GET_DIGIT_1:
			dgrammem.ch = display->digit1;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_DIGIT_1, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case DISPLAY_GET_MODE:
			dgrammem.ch = display->mode;
			formdatagram(datagramG, datagram[1], DISPLAY_SET_MODE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		
		default:
			uart_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

void parseButtonOp(uint8_t *datagram, Button *btn){
	switch(datagram[2]){
		//SETTERS
		case BUTTON_SET_PROGRAM_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				if(mode == 1) btn->programMode = 1;
				else if(mode == 0) btn->programMode = 0;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		case BUTTON_SET_PIN_MODE:
			if(datagram[0] == 4){
				uint8_t mode = datagram[3];
				if(mode == 1) btn->pinMode = 1;
				else if(mode == 0) btn->pinMode = 0;
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		
		//GETTERS
		case BUTTON_GET_PROGRAM_MODE:
			dgrammem.ch = btn->programMode;
			formdatagram(datagramG, datagram[1], BUTTON_SET_PROGRAM_MODE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		case BUTTON_GET_PIN_MODE:
			dgrammem.ch = btn->pinMode;
			formdatagram(datagramG, datagram[1], BUTTON_SET_PIN_MODE, dgrammem, 'c');
			uartputcs(datagramG);
		break;
		
		default:
			uart_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

void parseADCOp(uint8_t *datagram, AnalogIn *adc){
	switch(datagram[2]){
		//SETTERS
		case ADC_SET_SCALE:
			if(datagram[0] == 7){
				uint8_t flMem[4];
				for(uint8_t i=0; i<4; i++) flMem[i] = datagram[3+i];
				adc->scale = readFloat(flMem);
			}else{
				uart_puts_P("ERROR: Incorrect Type\n");
			}
		break;
		
		//GETTERS
		case ADC_GET_SCALE:
			dgrammem.fl = adc->scale;
			formdatagram(datagramG, datagram[1], ADC_SET_SCALE, dgrammem, 'f');
			uartputcs(datagramG);
		break;
		case ADC_GET_RAW:
			dgrammem.in = adc->raw;
			formdatagram(datagramG, datagram[1], ADC_SET_RAW, dgrammem, 'i');
			uartputcs(datagramG);
		break;
		case ADC_GET_READING:
			dgrammem.fl = adc->value;
			formdatagram(datagramG, datagram[1], ADC_SET_READING, dgrammem, 'f');
			uartputcs(datagramG);
		break;
		
		default:
			uart_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}

void parseAllOp(uint8_t *datagram){
	switch(datagram[2]){
		case ALL_STOP:
			motorA.position = 0;
			motorA.setDegrees = 0;
			motorA.setSpeedDPS = 0;
			motorB.position = 0;
			motorB.setDegrees = 0;
			motorB.setSpeedDPS = 0;
//DELETE			servoA.state = 0;
//DELETE			servoB.state = 0;
			displayA.draw = 0;
			ledR.state = 0;
			ledG.state = 0;
			ledB.state = 0;
		break;
		case CLEAR_DATA:
			motorA = (Motor){0};
			motorB = (Motor){0};
//DELETE			servoA = (Servo){0};
//DELETE			servoB = (Servo){0};
			ledR = (LED){0};
			ledG = (LED){0};
			ledB = (LED){0};
			displayA = (Display){0};
			buttonA = (Button){0};
			buttonB = (Button){0};
			buttonC = (Button){0};
			vdiv = (AnalogIn){0};
			csense = (AnalogIn){0};
		break;
		
		default:
			uart_puts_P("ERROR: Unknown OpCode\n");
			//flash BLUE LED
			ledB.state = 1;
			ledB.count = 1000;
		break;
	}
}





































//#################################################################################################
//
// Put main last so easier to find and navigate the file
//
//#################################################################################################

int16_t main(void){

	init_structs();
	init();	
//DELETE	init_display();

	ledB.state = 1;
	ledB.count = 10000;
	ledR.state = 1;
	ledR.count = 10000;	


	uart_puts_P("PenguinPi v2.0\n");
	
	detect_reset();
	
//INIT Done	
	

//TESTS
	//LEDs
	for(uint8_t j = 0; j < 3; j++) {
		PORTC= 0x00;
		_delay_ms(500);		
		PORTC= 0x0C;
		_delay_ms(500);		
	}


	//RGB
	redLEDPercent(50);	
	_delay_ms(500);
	redLEDPercent(100);	
	_delay_ms(500);
	redLEDPercent(0);	
	greenLEDPercent(50);
	_delay_ms(500);
	greenLEDPercent(100);
	_delay_ms(500);	
	greenLEDPercent(0);
	blueLEDPercent(50);
	_delay_ms(500);
	blueLEDPercent(100);
	_delay_ms(500);	
	blueLEDPercent(0);
//END TESTS
	
	uint8_t com;

	vdiv.count = ADC_COUNT;
	ADCSRA |= (1<<ADSC);		//start the first ADC conversion
	
    while (1) 
    {
		com = checkBuffer();					
		if(com == STARTBYTE){
			parseDatagram(datagramG);
		}
		//cleanup buffers
		com = 0;
		for(uint8_t j = 0; j < DGRAM_MAX_LENGTH; j++) datagramG[j] = 0;
		dgrammem.fl = 0;

		if ( DEBUG==1 ) {
			sprintf(fstring, "encA: %6d\n", motorA.position);
			uart_puts(fstring);
			sprintf(fstring, "encB: %6d\n", motorB.position);
			uart_puts(fstring);
		}

		motorA.degrees = motorA.position * DEGPERCOUNT;
		motorB.degrees = motorB.position * DEGPERCOUNT;


        /*
         * this is the non-PID code that needs to be run only if motor->controlMode == 0
         */
		OCR0B = mapRanges(abs(motorA.setSpeedDPS), 0, 100, 0, 0xFFFF);

		if(motorA.dir == 1){
			PORTB |= (1<<MOTOR_A_PHA);
		} else if(motorA.dir == -1) {
			PORTB &= ~(1<<MOTOR_A_PHA);
		} else {
			OCR0B = 0;
		}

		OCR0A = mapRanges(abs(motorB.setSpeedDPS), 0, 100, 0, 0xFFFF);

		if(motorB.dir == 1){
			PORTB &= ~(1<<MOTOR_B_PHA);
			}else if(motorB.dir == -1){
			PORTB |= (1<<MOTOR_B_PHA);
			}else{
			OCR0A = 0;
		}

        /*
         * this is the PID code that needs to be reinstated if motor->controlMode == 1
         */
		//Motor update		
		/*if(motorA.pidTimerFlag == 1){
			motorA.degrees = motorA.position * DEGPERCOUNT;
			int16_t result = motorPIDControl(motorA.setDegrees, &motorA);
			
			OCR1B = (mapRanges(abs(result), 0, 0xFFFF, MOTOR_PWM_RANGE_MIN, 0xFFFF));
			if(result > 0){
				motorA.dir = 1;
				//PORTC &= ~(1<<MOTOR_A_PHA);//reversed
				PORTC |= (1<<MOTOR_A_PHA);
			}else if(result < 0){
				motorA.dir = -1;
				//PORTC |= (1<<MOTOR_A_PHA);//reversed
				PORTC &= ~(1<<MOTOR_A_PHA);
			}else{
				motorA.dir = 0;
				OCR1B = 0;
			}
			motorA.pidTimerFlag = 0;
		}
		if(motorB.pidTimerFlag == 1){
			motorB.degrees = motorB.position * DEGPERCOUNT;
			int16_t result = motorPIDControl(motorB.setDegrees, &motorB);
			
			OCR1A = (mapRanges(abs(result), 0, 0xFFFF, MOTOR_PWM_RANGE_MIN, 0xFFFF));
			if(result > 0){
				motorB.dir = 1;
				//PORTC |= (1<<MOTOR_B_PHA);//reversed
				PORTC &= ~(1<<MOTOR_B_PHA);
			}else if(result < 0){
				motorB.dir = -1;
				//PORTC &= ~(1<<MOTOR_B_PHA);//reversed
				PORTC |= (1<<MOTOR_B_PHA);
			}else{
				motorB.dir = 0;
				OCR1A = 0;
			}
			motorB.pidTimerFlag = 0;
		}*/
		
//DELETE		//Servo update
//DELETE		if(servoA.state){
//DELETE			if(servoA.setPos < servoA.minRange) servoA.setPos = servoA.minRange;//clip the position to within bounds
//DELETE			if(servoA.setPos > servoA.maxRange) servoA.setPos = servoA.maxRange;
//DELETE				OCR3A = MAP(servoA.setPos, servoA.minRange, servoA.maxRange, servoA.minPWM, servoA.maxPWM);//bad stuff happens when data types are to small
//DELETE			//debug
//DELETE			//sprintf(fstring, "ServoA: %ld deg\n", servoA.setPos);
//DELETE			//uart_puts(fstring);
//DELETE			//sprintf(fstring, "Mapped: %ld tic\n", MAP(servoA.setPos, servoA.minRange, servoA.maxRange, servoA.minPWM, servoA.maxPWM));
//DELETE			//uart_puts(fstring);
//DELETE			//displayA.value = servoA.setPos/10;
//DELETE		}else{
//DELETE			PORTD &= ~(1<<SERVO_A);
//DELETE		}
//DELETE		if(servoB.state){
//DELETE			if(servoB.setPos < servoB.minRange) servoB.setPos = servoB.minRange;
//DELETE			if(servoB.setPos > servoB.maxRange) servoB.setPos = servoB.maxRange;
//DELETE				OCR4A = MAP(servoB.setPos, servoB.minRange, servoB.maxRange, servoB.minPWM, servoB.maxPWM);
//DELETE		}else{
//DELETE			PORTD &= ~(1<<SERVO_B);
//DELETE		}
				

//DELETE		//Display update
//DELETE		if(displayA.draw){
//DELETE			update_dd7s(&displayA);
//DELETE			displayA.draw = 0;
//DELETE		}
		
						
		//LED update
		if(ledR.state > 0){
			if(ledR.brightness > 0) redLEDPercent(ledR.brightness);
			else LEDOn(LED_R);
		} else LEDOff(LED_R);

		if(ledG.state > 0){
			if(ledG.brightness > 0) greenLEDPercent(ledG.brightness);
			else LEDOn(LED_G);
		} else LEDOff(LED_G);

		if(ledB.state > 0){ 
			if(ledB.brightness > 0) blueLEDPercent(ledB.brightness);
			else LEDOn(LED_B);
		} else LEDOff(LED_B);
		

		//Button update
		if(buttonA.state == 1){
			ledR.state = 1;
			ledR.count = 2000;
// 			displayA.value--;
// 			displayA.draw = 1;
			
			uart_puts_P("This is a shutdown request");

			motorA.position = 0;
			motorA.setDegrees = 180;
			motorA.dir = -1;
			motorB.position = 0;
			motorB.setDegrees = 180;
			motorB.dir = 1;
			if(buttonA.programMode == 0) buttonA.state = 0;
		}else{
			
		}
		if(buttonB.state == 1){
			ledG.state = 1;
			ledG.count = 2000;
// 			displayA.value = 0;
// 			displayA.draw = 1;

			motorA.position = 0;
			motorA.setSpeedDPS = motorA.setSpeedDPS +10;
			motorA.dir = -1;
			motorB.position = 0;
			motorB.setDegrees = -180;
			motorB.dir = -1;
			if(buttonB.programMode == 0) buttonB.state = 0;
		}else{
			
		}
		if(buttonC.state == 1){
			ledB.state = 1;
			ledB.count = 2000;
// 			displayA.value++;
// 			displayA.draw = 1;

			motorA.position = 0;
			motorA.setSpeedDPS = motorA.setSpeedDPS -10;
			motorA.dir = 1;
			motorB.position = 0;
			motorB.setDegrees = -180;
			motorB.dir = -1;
			if(buttonC.programMode == 0) buttonC.state = 0;
		}else{
			
		}

		//Analog update
		if(vdiv.ready && !(ADCSRA&(1<<ADSC))){
			vdiv.raw   = ADC;
			vdiv.value = vdiv.raw * vdiv.scale/1000;
			vdiv.ready = 0;

			//change multiplexer to csense
			ADMUX 		|= (1<<MUX0);//change mux
			csense.count = ADC_COUNT;
			ADCSRA 		|= (1<<ADSC);//restart

			if ( DEBUG==1 ) {
				sprintf(fstring, "Voltage: %5.3f V\n", vdiv.value);
				uart_puts(fstring);
			}
		}
		if(csense.ready && !(ADCSRA&(1<<ADSC))){
			csense.raw   = ADC;
			csense.value = csense.raw * csense.scale;
			csense.ready = 0;

			//change multiplexer to csense
			ADMUX 	  &= ~(1<<MUX0);//change mux
			vdiv.count = ADC_COUNT;
			ADCSRA 	  |= (1<<ADSC);//restart

			if ( DEBUG==1 ) {		
				sprintf(fstring, "Current: %5.3f mA\n", csense.value);
				uart_puts(fstring);
			}
		}

		if(vdiv.value < battery.cutoff){
			if(battery.count < battery.limit) battery.count++;
			else{
				//lockout most functions
				PRR0 |= (1<<PRTIM2)|(1<<PRTIM0)|(1<<PRTIM1);//|(1<<PRADC);
				//display low battery warning
				uint8_t reg[2] = {0, 0};
				reg[0] = DIGIT0_B;
				reg[1] = DIGIT1_F;
				i2cWritenBytes(reg, displayA.address, OUTPUT_0, 2);
								
				while(1){
					//send shutdown command
					uart_puts_P("Low battery shutdown request");
					PORTB &= ~(1<<PB5);//pull the pin low
					//loop until the battery really is flat...
				}			
			}
		}else battery.count = 0;
		
		
		if ( DEBUG==1 ) {
			//TOGGLE LED1 : C3
			PORTC = PORTC ^ 0x04;
			_delay_ms(100);
		}
		
    }
}

