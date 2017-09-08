
#include <stdio.h>
#include <avr/io.h>

#include "i2cmaster.h"
#include "uart.h"

#include "PenguinPi.h"



void detect_reset(void){
	//read MCUSR and determine what reset the AVR
	uint8_t reg = MCUSR;
	MCUSR = 0x00;
	switch (reg){
		case 1:
			uart_puts_P("Device Reset from Power On Reset\n");
		break;
		case 2:
			uart_puts_P("Device Reset from External Reset\n");
		break;
		case 4:
			uart_puts_P("Device Reset from Brown Out Detector\n");
		break;
		case 8:
			uart_puts_P("Device Reset from Watchdog Timeout\n");
		break;
		case 16:
			uart_puts_P("Device Reset from JTAG flag\n");
		break;		
		default:
			uart_puts_P("Device turned on\n");
		break;
	}
}

float readFloat(uint8_t *flMem){
	/*
	* Method courtesy AVR Freaks user 'clawson'
	* http://www.avrfreaks.net/forum/converting-4-bytes-float-help
	*/
	for(uint8_t i=0; i<4; i++){
		#ifdef LITTLE_ENDIAN
		// fill in 0, 1, 2, 3
		floatChar.c[i] = flMem[i];
		#else
		// fill in 3, 2, 1, 0
		floatChar.c[3-i] = flMem[i];
		#endif
	}
	return floatChar.f;
}

uint8_t *float2char(float f){
	floatChar.f = f;
	return floatChar.c;
}

uint8_t crc8(uint8_t *word, uint8_t length){
	uint8_t crc = 0;
	for(uint8_t i=0; i < length; i++){
		crc ^= word[i];
		for(uint8_t j=0; j < 8; j++){
			if(crc & 1) crc = (crc >> 1) ^ CRC_8_POLY;
			else crc = (crc >> 1);
		}
	}
	return crc;
}

uint16_t mapRanges(uint16_t a, uint16_t amin, uint16_t amax, uint16_t omin, uint16_t omax){
	return ((a-amin)*((omax-omin)/(amax-amin))) + omin; //maps from scale amin->amax to scale omin->omax
}

void buttonLogic(Button *button, uint8_t btnVal){
	if(button->debounceCount == 0){
		button->pinState = btnVal;
		if(btnVal == 0){//button is held down, falling edge
			button->state ^= 1;
		}else if(btnVal == 1 && button->pinMode == 1){//rising edge
			button->state ^= 1;
		}
		button->debounceCount = 500;    // stop further transitions being processed for 500 ticks
	}
}

void uartputcs(uint8_t *datagram){
	uart_putc(STARTBYTE);
	for(uint8_t i = 0; i < datagram[0]; i++){
		uart_putc(datagram[i]);
	}
	//uart_putc(STOPBYTE);
}

//#################################################################################################
//
// I2C
//
//#################################################################################################
void i2cReadnBytes(uint8_t *data, uint8_t address, uint8_t reg, uint8_t n){
	i2c_start_wait(address+I2C_WRITE);
	i2c_write(reg);				//where to read
	i2c_rep_start(address+I2C_READ);
	
	for(uint8_t i = 0; i < n-1; i++){
		data[i] = i2c_readAck();
	}
	data[n-1] = i2c_readNak();
	i2c_stop();
}

int8_t i2cWriteByte(uint8_t data, uint8_t address, uint8_t reg){
	i2c_start_wait(address+I2C_WRITE);
	i2c_write(reg); //where to write...
	i2c_write(data);
	i2c_stop();
	return 1;//completed all writes without failure
}

int8_t i2cWritenBytes(uint8_t *data, uint8_t address, uint8_t reg, uint8_t n){
	i2c_start_wait(address+I2C_WRITE);
	i2c_write(reg); //where to write...
	for(uint8_t i = 0; i < n; i++){
		i2c_write(data[i]);
	}
	i2c_stop();
	return 1;//completed all writes without failure
}



//#################################################################################################
//
// LEDs
//
//#################################################################################################
void LEDOff(uint8_t led){
	if(led == 0xFF){
		DDRD &= ~((1<<LED_R)|(1<<LED_G)|(1<<LED_B));
		//PORTD |= (1<<LED_R)|(1<<LED_G)|(1<<LED_B);
	}else{
		DDRD &= ~(1<<led);
		//PORTD |= (1<<led);//common anode so high: off
	}
}

void LEDOn(uint8_t led){
	if(led == 0xFF){
		DDRD |= (1<<LED_R)|(1<<LED_G)|(1<<LED_B);
		//PORTD &= ~((1<<LED_R)|(1<<LED_G)|(1<<LED_B));
	}else{
		DDRD |= (1<<led);
		//PORTD &= ~(1<<led);//common anode so low: on
	}
}

void redLEDPercent(uint8_t percent){
	percent %= 100;
	if(percent == 0){
		LEDOff(LED_R);
	}else{
		DDRD |= (1<<LED_R);
		OCR2A = MAP(percent, 0, 100, 0, RED_MAX);
	}
}

void greenLEDPercent(uint8_t percent){
	percent %= 100;
	if(percent == 0){
		LEDOff(LED_G);
	}else{
		DDRD |= (1<<LED_G);
		OCR1A = MAP(percent, 0, 100, 0, GREEN_MAX);
	}
}

void blueLEDPercent(uint8_t percent){
	percent %= 100;
	if(percent == 0){
		LEDOff(LED_B);
	}else{
		DDRD |= (1<<LED_B);
		OCR2B = MAP(percent, 0, 100, 0, BLUE_MAX);		
	}
}



//#################################################################################################
//
// MOTORs
//
//#################################################################################################

int16_t motorPIDControl(int16_t setPoint, Motor *motor){
	//based upon Application Note AVR221
	int16_t error = setPoint - motor->degrees;

	int16_t pTerm, dTerm;
	int32_t iTerm;
	
	//P term, also limit overflow
	if(error > motor->maxError)			pTerm = INT16_MAX;
	else if(error < -motor->maxError)	pTerm = -INT16_MAX;
	else								pTerm = motor->gainP * error;
	
	//I term, also limit overflow
	int32_t newSum = motor->errorSum + error;
	if(newSum > motor->maxErrorSum){
		motor->errorSum = motor->maxErrorSum;
		iTerm = INT32_MAX/2;
	}else if(newSum < -motor->maxErrorSum){
		motor->errorSum = -motor->maxErrorSum;
		iTerm = -(INT32_MAX/2);
	}else{
		motor->errorSum = newSum;
		iTerm = motor->gainI * motor->errorSum;
	}

	//D term //put this on a slower interval
	dTerm = motor->gainD * (motor->lastVal - motor->degrees);
	
	motor->lastVal = motor->degrees;
	
	int32_t result = (pTerm + iTerm + dTerm)/PID_SCALE;
	
	if(result > INT16_MAX) result = INT16_MAX;
	else if(result < -INT16_MAX) result = -INT16_MAX;
	
	return (int16_t)result;		
}
































//#################################################################################################
//
// V1.0 CODE - Useful for Reference 
//
//#################################################################################################

//DELETE	void init_display(void){
//DELETE		//initialises the 7 segment display controller
//DELETE		uint8_t reg[2] = {0, 0};
//DELETE		i2cWritenBytes(reg, displayA.address, CONFIG_0, 2); //configures both ports as outputs
//DELETE		i2cWritenBytes(reg, displayA.address, OUTPUT_0, 2); //sets both outputs to 0
//DELETE	}

//DELETE	void update_dd7s(Display *display){//TODO: fix this up
//DELETE		uint8_t reg[2]= {0, 0}; // MS, LS digits
//DELETE	    uint8_t value = display->value;
//DELETE	    int8_t  svalue;
//DELETE	
//DELETE	    switch (display->mode) {
//DELETE	    case 0: // hex mode
//DELETE	        reg[0] = digit0[(value>>4) & 0x0f];
//DELETE	        reg[1] = digit1[value & 0x0f];
//DELETE	        break;
//DELETE	    case 1: // unsigned decimal, light up the decimal point
//DELETE	        if (value > 99)
//DELETE	            value = 99;
//DELETE	        reg[0] = digit0[(uint8_t)value/10];
//DELETE	        reg[1] = digit1[(uint8_t)value%10]|SEGMENTDP_1;
//DELETE	        break;
//DELETE	    case 2: // signed decimal -9 to +9
//DELETE	        svalue = (int8_t) value;
//DELETE	        if (svalue < -9)
//DELETE	            svalue = -9;
//DELETE	        else if (svalue > 9)
//DELETE	            svalue = 9;
//DELETE	        if (svalue < 0)
//DELETE				reg[0] = SEGMENTG_0;    // minus sign
//DELETE	        reg[1] = digit1[abs(svalue)]|SEGMENTDP_1;
//DELETE	        break;
//DELETE	    }
//DELETE	    /*
//DELETE		//determine the best way to display the value
//DELETE		if(display->value != 0xFF){
//DELETE			displayBase10(reg, display->value);
//DELETE		}else{
//DELETE	
//DELETE		}
//DELETE		//if(display->digit0 != 0xFF) reg[0] = digit0[display->digit0];//digit0 and digit1 will override value
//DELETE		//if(display->digit1 != 0xFF) reg[1] = digit1[display->digit1];
//DELETE	    */
//DELETE	
//DELETE		i2cWritenBytes(reg, display->address, OUTPUT_0, 2);
//DELETE	}

//DELETE	void displayBase10(uint8_t *reg, int16_t value){
//DELETE		if(value < 0){
//DELETE			//negative numbers
//DELETE			if(value > -1){
//DELETE				//decimal
//DELETE	
//DELETE			}else if(value < -15){
//DELETE				//too low to display
//DELETE				reg[0] = SEGMENTG_0;
//DELETE				reg[1] = DIGIT1_O;
//DELETE			}else{
//DELETE				reg[0] = SEGMENTG_0;
//DELETE				reg[1] = digit1[abs(value)]|SEGMENTDP_1;
//DELETE			}
//DELETE		}else{
//DELETE			//positive numbers
//DELETE			if(value < 1){
//DELETE				//decimal
//DELETE				reg[0] = digit0[(uint8_t)value*10];
//DELETE			}else if(value < 10){
//DELETE				//can still display 1 decimal place
//DELETE			}else if(value > 10){
//DELETE				//only integers here
//DELETE				reg[0] = digit0[(uint8_t)value/10];
//DELETE				reg[1] = digit1[(uint8_t)value%10]|SEGMENTDP_1;
//DELETE			}
//DELETE		}
//DELETE	}

//DELETE	void parseServoOp(uint8_t *datagram, Servo *servo){
//DELETE		switch(datagram[2]){
//DELETE			//SETTERS
//DELETE			case SERVO_SET_POSITION:
//DELETE				if(datagram[0] == 5){
//DELETE					int16_t pos = (datagram[3]<<8)|datagram[4];
//DELETE					if(pos > servo->maxRange) pos = servo->maxRange;
//DELETE					else if(pos < servo->minRange) pos = servo->minRange;
//DELETE					servo->setPos = pos;
//DELETE					servo->state = 1;
//DELETE				}else{
//DELETE					uart_puts_P("ERROR: Incorrect Type\n");
//DELETE				}
//DELETE			break;
//DELETE			case SERVO_SET_STATE:
//DELETE				if(datagram[0] == 4){
//DELETE					int8_t state = datagram[3];
//DELETE					if(state >= 1) servo->state = 1;
//DELETE					else servo->state = 0;
//DELETE				}else{
//DELETE					uart_puts_P("ERROR: Incorrect Type\n");
//DELETE				}
//DELETE			break;
//DELETE			case SERVO_SET_MIN_RANGE:
//DELETE				if(datagram[0] == 5){
//DELETE					int16_t min = (datagram[3]<<8)|datagram[4];
//DELETE					if(min < 0) min = 0;
//DELETE					if(min > 360) min = 360;
//DELETE					if(min > servo->maxRange) min = servo->maxRange;
//DELETE					servo->minRange = min;
//DELETE				}else{
//DELETE					uart_puts_P("ERROR: Incorrect Type\n");
//DELETE				}
//DELETE			break;
//DELETE			case SERVO_SET_MAX_RANGE:
//DELETE				if(datagram[0] == 5){
//DELETE					int16_t max = (datagram[3]<<8)|datagram[4];
//DELETE					if(max < 0) max = 0;
//DELETE					if(max > 360) max = 360;
//DELETE					if(max < servo->minRange) max = servo->minRange;
//DELETE					servo->maxRange = max;
//DELETE				}else{
//DELETE					uart_puts_P("ERROR: Incorrect Type\n");
//DELETE				}
//DELETE			break;
//DELETE			case SERVO_SET_MIN_PWM:
//DELETE				if(datagram[0] == 5){
//DELETE					uint16_t min = (datagram[3]<<8)|datagram[4];
//DELETE					if(min < 0) min = 0;
//DELETE					if(min > 30000) min = 30000;
//DELETE					if(min > servo->maxPWM) min = servo->maxPWM;
//DELETE					servo->minPWM = min;
//DELETE				}else{
//DELETE					uart_puts_P("ERROR: Incorrect Type\n");
//DELETE				}
//DELETE			break;
//DELETE			case SERVO_SET_MAX_PWM:
//DELETE				if(datagram[0] == 5){
//DELETE					uint16_t max = (datagram[3]<<8)|datagram[4];
//DELETE					if(max < 0) max = 0;
//DELETE					if(max > 30000) max = 30000;
//DELETE					if(max < servo->minPWM) max = servo->minPWM;
//DELETE					servo->maxPWM = max;
//DELETE				}else{
//DELETE					uart_puts_P("ERROR: Incorrect Type\n");
//DELETE				}
//DELETE			break;
//DELETE	
//DELETE			//GETTERS
//DELETE			case SERVO_GET_POSITION:
//DELETE				dgrammem.in = servo->setPos;
//DELETE				formdatagram(datagramG, datagram[1], SERVO_SET_POSITION, dgrammem, 'i');
//DELETE				uartputcs(datagramG);
//DELETE			break;
//DELETE			case SERVO_GET_STATE:
//DELETE				dgrammem.ch = servo->state;
//DELETE				formdatagram(datagramG, datagram[1], SERVO_SET_STATE, dgrammem, 'c');
//DELETE				uartputcs(datagramG);
//DELETE			break;
//DELETE			case SERVO_GET_MIN_RANGE:
//DELETE				dgrammem.in = servo->minRange;
//DELETE				formdatagram(datagramG, datagram[1], SERVO_SET_MIN_RANGE, dgrammem, 'i');
//DELETE				uartputcs(datagramG);
//DELETE			break;
//DELETE			case SERVO_GET_MAX_RANGE:
//DELETE				dgrammem.in = servo->maxRange;
//DELETE				formdatagram(datagramG, datagram[1], SERVO_SET_MAX_RANGE, dgrammem, 'i');
//DELETE				uartputcs(datagramG);
//DELETE			break;
//DELETE			case SERVO_GET_MIN_PWM:
//DELETE				dgrammem.in = servo->minPWM;
//DELETE				formdatagram(datagramG, datagram[1], SERVO_SET_MIN_PWM, dgrammem, 'i');
//DELETE				uartputcs(datagramG);
//DELETE			break;
//DELETE			case SERVO_GET_MAX_PWM:
//DELETE				dgrammem.in = servo->maxPWM;
//DELETE				formdatagram(datagramG, datagram[1], SERVO_SET_MAX_PWM, dgrammem, 'i');
//DELETE				uartputcs(datagramG);
//DELETE			break;
//DELETE	
//DELETE			default:
//DELETE				uart_puts_P("ERROR: Unknown OpCode\n");
//DELETE				//flash BLUE LED
//DELETE				ledB.state = 1;
//DELETE				ledB.count = 1000;
//DELETE			break;
//DELETE		}
//DELETE		
//DELETE	}




