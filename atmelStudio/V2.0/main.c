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
LED 		ledR;
LED 		ledG;
LED 		ledB;
Display 	displayA;
//Button 		buttonA;	-- No connected buttons to AVR, all on I2C
//Button 		buttonB;	-- No connected buttons to AVR, all on I2C
//Button 		buttonC;	-- No connected buttons to AVR, all on I2C
AnalogIn 	vdiv;
AnalogIn 	csense;



//Global variables
char 	fstring[32];
uint8_t datagramG[DGRAM_MAX_LENGTH+1];



//#################################################################################################
//
// ISRs
//
//#################################################################################################

ISR(PCINT0_vect) {
	//All encoders are now on one PCINT vector
	//Externally on PCINT{3:0}

	//detect which pin triggered the interrupt
		uint8_t PINC_val   = PINC;
		uint8_t enc_a1_val = ( PINC_val & (1<<MOTOR_A_ENC_1) )>>MOTOR_A_ENC_1; 	//store the current state
		uint8_t enc_a2_val = ( PINC_val & (1<<MOTOR_A_ENC_2) )>>MOTOR_A_ENC_2;
		uint8_t enc_b1_val = ( PINC_val & (1<<MOTOR_B_ENC_1) )>>MOTOR_B_ENC_1; 	//store the current state
		uint8_t enc_b2_val = ( PINC_val & (1<<MOTOR_B_ENC_2) )>>MOTOR_B_ENC_2;
		
	//Update Motor states
		fn_update_motor_states( &motorA, enc_a1_val, enc_a2_val ); 
		fn_update_motor_states( &motorB, enc_b1_val, enc_b2_val ); 
	
	//Update LEDs
		ledG.state = 1;
		ledG.count = 100;
}

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


//#################################################################################################
//
// INITs
//
//#################################################################################################

void init_structs(void){
	float kP 			= 70.0;
	float kI 			= 0.0075;
	float kD 			= 2.0;

	motorA.which_motor	= 0;
	motorA.gainP 		= kP*PID_SCALE;
	motorA.gainI 		= kI*PID_SCALE;
	motorA.gainD 		= kD*PID_SCALE;
	motorA.maxError 	= INT16_MAX / (motorA.gainP + 1);
	motorA.maxErrorSum 	= (INT32_MAX / 2) / (motorA.gainI + 1);
	motorA.encoderMode 	= 1;

	motorB.which_motor	= 1;
	motorB.gainP 		= kP*PID_SCALE;
	motorB.gainI 		= kI*PID_SCALE;
	motorB.gainD 		= kD*PID_SCALE;
	motorB.maxError 	= INT16_MAX / (motorB.gainP + 1);
	motorB.maxErrorSum 	= (INT32_MAX / 2) / (motorB.gainI + 1);
	motorB.encoderMode 	= 1;

	vdiv.count 			= 0;
	vdiv.scale 			= 16.018497;//mV per div
	csense.count 		= 0;
	csense.scale 		= 3.2226562;//mA per div

	battery.cutoff 		= 6.5;
	battery.count		= 0;
	battery.limit 		= 5000;//about 1.5 seconds
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
		TCCR0A |= (1<<COM0A1)|(0<<COM0A0)|			// Clear OC0A on Compare Match	Set OC0A on Bottom
				  (1<<COM0B1)|(0<<COM0B0)|			// Clear OC0B on Compare Match	Set OC0B on Bottom
				  (1<<WGM01) |(1<<WGM00); 			// Non-inverting, 8 bit fast PWM
				  
		TCCR0B |= (0<<WGM02) |
				  (0<<CS02)|(0<<CS01)|(1<<CS00);	// DIV1 prescaler


	//Encoders
		//V1 was PC2, PC3, PE0, PE1 ... PCINT 10,11,24,25
		//V2  is PA0, PA1, PA2, PA3 ... PCINT 0:3
		DDRA 	&= ~((1<<MOTOR_A_ENC_1)|(1<<MOTOR_A_ENC_2)|(1<<MOTOR_B_ENC_1)|(1<<MOTOR_B_ENC_2));		//Direction to INPUT
		PORTA 	|=   (1<<MOTOR_A_ENC_1)|(1<<MOTOR_A_ENC_2)|(1<<MOTOR_B_ENC_1)|(1<<MOTOR_B_ENC_2);		//Enable internal PULLUPs
		PCMSK0   =   (1<<PCINT0)|(1<<PCINT1)|(1<<PCINT2)|(1<<PCINT3);									//Enable Pin Change Mask
		PCICR 	|=   (1<<PCIE0);																		//Enable interrupt on pin change
			
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

	//8 bit controller timer
		TIMSK2 |= (1<<TOIE2);

	//ADC
		DIDR0	= (1<<ADC6D)|(1<<ADC7D);
		ADMUX 	= (0<<REFS1)|(1<<REFS0)|(0<<MUX4)|(0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0); 	//AVCC reference voltage with external cap on AREF, multiplex to channel 6
		ADCSRA 	= (1<<ADEN) |(1<<ADIE) |(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);					//Enable ADC, enable interrupt, prescaler of 64 (187.5kHz sample rate)

	//UART
		uart_init(UART_BAUD_SELECT_DOUBLE_SPEED(BAUD, F_CPU));
	
	//I2C
		i2c_init();

	//Enable Interrupts
		sei();
}


//#################################################################################################
//
// Message Parsing
//
//#################################################################################################

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

//DELETE		case AD_BTN_A:
//DELETE			parseButtonOp(datagram, &buttonA);
//DELETE		break;
//DELETE		case AD_BTN_B:
//DELETE			parseButtonOp(datagram, &buttonB);
//DELETE		break;
//DELETE		case AD_BTN_C:
//DELETE			parseButtonOp(datagram, &buttonC);
//DELETE		break;

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

void parseLEDOp		( uint8_t *datagram, LED *led ){
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

void parseDisplayOp	( uint8_t *datagram, Display *display ){
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

void parseADCOp		( uint8_t *datagram, AnalogIn *adc ){
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

void parseAllOp		( uint8_t *datagram ){
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
//DELETE			buttonA = (Button){0};
//DELETE			buttonB = (Button){0};
//DELETE			buttonC = (Button){0};
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

	ledB.state = 1;
	ledB.count = 10000;
	ledR.state = 1;
	ledR.count = 10000;	

	uart_puts_P("PenguinPi v2.0\n");
	
	detect_reset();	
//INIT Done	
	

//TESTS
	//LEDs
//	for(uint8_t j = 0; j < 3; j++) {
//		PORTC= 0x00;
//		_delay_ms(500);		
//		PORTC= 0x0C;
//		_delay_ms(500);		
//	}


	//RGB
//	redLEDPercent(50);	
//	_delay_ms(500);
//	redLEDPercent(100);	
//	_delay_ms(500);
//	redLEDPercent(0);	
//	greenLEDPercent(50);
//	_delay_ms(500);
//	greenLEDPercent(100);
//	_delay_ms(500);	
//	greenLEDPercent(0);
//	blueLEDPercent(50);
//	_delay_ms(500);
//	blueLEDPercent(100);
//	_delay_ms(500);	
//	blueLEDPercent(0);
	
	//MOTORS	
//	motorA.dir			=  1;
//	motorA.setSpeedDPS	= 50;
	
//	motorB.dir			= -1;
//	motorB.setSpeedDPS	= 90;	

//END TESTS


	
	uint8_t com;

	vdiv.count = ADC_COUNT;
	ADCSRA |= (1<<ADSC);		//start the first ADC conversion
	
    while (1) 
    {
		com = checkBuffer();					
		if(com == STARTBYTE){
			parseDatagram( datagramG );
		}
		//cleanup buffers
		com = 0;
		for(uint8_t j = 0; j < DGRAM_MAX_LENGTH; j++) datagramG[j] = 0;
		dgrammem.fl = 0;



		motorA.degrees = motorA.position * DEGPERCOUNT;
		motorB.degrees = motorB.position * DEGPERCOUNT;


        /*
         * this is the non-PID code that needs to be run only if motor->controlMode == 0
         */
		OCR0B = mapRanges( abs(motorA.setSpeedDPS), 0, 100, 0, 255 );

		if(motorA.dir == 1){
			PORTB |= (1<<MOTOR_A_PHA);
		} else if(motorA.dir == -1) {
			PORTB &= ~(1<<MOTOR_A_PHA);
		} else {
			OCR0B = 0;
		}

		OCR0A = mapRanges( abs(motorB.setSpeedDPS), 0, 100, 0, 255 );

		if(motorB.dir == 1) {			
			PORTB &= ~(1<<MOTOR_B_PHA);
		}
		else if(motorB.dir == -1) {
			PORTB |= (1<<MOTOR_B_PHA);
		}
		else {
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
			
						
		//LED update
			if(ledR.state > 0) {
				if(ledR.brightness > 0) redLEDPercent(ledR.brightness);
				else 					LEDOn(LED_R);
			} else 						LEDOff(LED_R);
	
			if(ledG.state > 0) {
				if(ledG.brightness > 0) greenLEDPercent(ledG.brightness);
				else 					LEDOn(LED_G);
			} else 						LEDOff(LED_G);
	
			if(ledB.state > 0) { 
				if(ledB.brightness > 0) blueLEDPercent(ledB.brightness);
				else 					LEDOn(LED_B);
			} else 						LEDOff(LED_B);
		


		//Analog update
		if(vdiv.ready && !(ADCSRA&(1<<ADSC))){
			vdiv.raw   = ADC;
			vdiv.value = vdiv.raw * vdiv.scale/1000;
			vdiv.ready = 0;

			//change multiplexer to csense
			ADMUX 		|= (1<<MUX0);//change mux
			csense.count = ADC_COUNT;
			ADCSRA 		|= (1<<ADSC);//restart
		}
		
		if(csense.ready && !(ADCSRA&(1<<ADSC))){
			csense.raw   = ADC;
			csense.value = csense.raw * csense.scale;
			csense.ready = 0;

			//change multiplexer to csense
			ADMUX 	  &= ~(1<<MUX0);//change mux
			vdiv.count = ADC_COUNT;
			ADCSRA 	  |= (1<<ADSC);//restart
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
			
			//to UART
				//Voltage
					sprintf(fstring, "Voltage: %5.3f V\n", vdiv.value);
					uart_puts(fstring);				
				
				//Current
					sprintf(fstring, "Current: %5.3f mA\n", csense.value);
					uart_puts(fstring);	
			
				//Motors
					uart_puts_P("M:A\n");
						fn_dbg_motor ( &motorA );					
					uart_puts_P("M:B\n");
						fn_dbg_motor ( &motorB );				
						
				//Timers
					sprintf(fstring, "OC MA: %5d\n", OCR0B );	//Motor A on OCR0B
					uart_puts(fstring);			
					sprintf(fstring, "OC MB: %5d\n", OCR0A ); 	//Motor B on OCR0A
					uart_puts(fstring);					
			
		}
		
    }
}

