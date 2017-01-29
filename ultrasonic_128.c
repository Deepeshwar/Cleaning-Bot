/*
 * ultrasonic_128.c
 *
 * Created: 1/28/2017 6:57:05 PM
 *  Author: Baba
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART_128.h"
#include <avr/sfr_defs.h>


#define trigger_DDR				DDRF		
#define triggerPORT				PORTF
#define trigger_pin1			PF0
#define trigger_pin2			PF1
#define echoPORT				PORTE
#define echo_interrupt1			4
#define echo_interrupt2			5
#define preScalar1				0x05
#define preScalar2				0x05

volatile int timer_counter1 = 0;
volatile int timer_counter2 = 0;

void trigger(int n) {
	
	if(n == 1) {
		//trigger 1st US sensor
		if(bit_is_set(echoPORT, echo_interrupt1))	return;			//donot trigger if echo pin already in high state
			triggerPORT &=~(1<<trigger_pin1);						//setting trig pin to low
			_delay_us(2);											//delay to let signal settle
			triggerPORT |= (1<<trigger_pin1); 						//setting trig pin to high to activate sensor
			_delay_us(10);											//pause in high state greater than 10us
			triggerPORT &= ~(1<<trigger_pin1);						//bring trig pin back to low
			TCNT1 = 0;
		return;
	}
	
	else if(n == 2) {
	
		if(bit_is_set(echoPORT, echo_interrupt1))	return;			//donot trigger if echo pin already in high state
	
		//trigger 2nd US sensor
		triggerPORT &=~(1<<trigger_pin2);							//setting trig pin to low
		_delay_us(2);												//delay to let signal settle
		triggerPORT |= (1<<trigger_pin2); 							//setting trig pin to high to activate sensor
		_delay_us(10);												//pause in high state greater than 10us
		triggerPORT &= ~(1<<trigger_pin2);							//bring trig pin back to low
		TCNT3 = 0;
		return;
	}	
}

float calc_dist(int n) {
	if(n == 1) 		 	    return ((timer_counter1 / 58.0) * 128);			//calculation for dist1
	else if(n == 2)			return ((timer_counter2 / 58.0) * 128);				//calculations for dist2
}

void init() {
	
	trigger_DDR = (1 << trigger_pin1) | (1 << trigger_pin2);
	
	//timer init
	TCCR1B = preScalar1;										//prescalar of 1024
	TCCR3B = preScalar2;										//prescalar of 1024
	EICRB|=(1<<ISC41)|(1<<ISC51);								//falling edge
	EIMSK|=(1 << echo_interrupt1) | (1 << echo_interrupt2);		//external interrupt 4 and 5 enabled
	
	USART_Init(51,1);
	sei();
}


int main(void)
{
	
	init();
	
    while(1) {
/*		trigger(1);
		trigger(2);
		//USART_TransmitNumber(calc_dist(1) , 1);
*/		
		USART_Transmitchar('a', 1);
	//	USART_Transmitchar(0x0D,1);
				
	}
}


ISR(INT4_vect) {
	
	timer_counter1 = TCNT1;
}

ISR(INT5_vect) {
	timer_counter2 = TCNT3;
}




