#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "wire1_shared.h"
#include "wire1_lite_slave.h"

void wire1_listener(){

	char sigLength;

	// If line is "1" do nothing
	if(WIRE1_PIN & _BV(WIRE1))
		return;

	cli();
	WIRE1_TIMER_CNT = 0;
	sei();

	sigLength = 0;
	while(~WIRE1_PIN & _BV(WIRE1) && WIRE1_TIMER_CNT<160){
		sigLength = WIRE1_TIMER_CNT;
	}

	// If this is not "reset" (480uS) - exit
	if(sigLength<147 || sigLength>153)
		return;

	// Send "present"
	WIRE1_PORT_TO_SEND;
	_delay_us(60);
	WIRE1_PORT_TO_RECEIVE;

	// Collect command bits
	char res = 0;
	char i = 0;
	char startTime = 0;
	sigLength = 0;
	cli();
	WIRE1_TIMER_CNT = 0;
	sei();
	while(i<8 && sigLength<27){
		sigLength = WIRE1_TIMER_CNT;
		if((~WIRE1_PIN & _BV(WIRE1)) && startTime==0){
			startTime = sigLength;
			WIRE1_TIMER_CNT = 0;
		}
		else{
			if((WIRE1_PIN & _BV(WIRE1)) && startTime>0){
				if(sigLength<3)
					res |= 1;
				if(i<7)
					res <<= 1;
				i++;
				startTime = 0;
			}
		}
	}

	if(sigLength>=27)
		return;

/*for(i=0;i<8;i++){
	_delay_us(50);
	if((res>>i) & 1){
		PORTC ^= _BV(PC0);
		_delay_us(20);
		PORTC ^= _BV(PC0);
	}
	else{
		PORTC ^= _BV(PC0);
		_delay_us(40);
		PORTC ^= _BV(PC0);
	}
}*/


	// If this is "set default" command (check with reversed value)
	if(res==0b10100110){
		sendByteSlave(~wire1_address);
	}

	// If this is "check alarm state" command (check with reversed value)
	if(res==0b00000000){
		sendByteSlave(wire1_address);
	}
}

void sendByteSlave(char resp){

	char i=0;
	char startTime = 0;

	cli();
	WIRE1_TIMER_CNT = 0;
	sei();

	while(i<8 && WIRE1_TIMER_CNT<70){
		if(~WIRE1_PIN & _BV(WIRE1) && !startTime){
			// Reset timer
			startTime = 1;
			WIRE1_TIMER_CNT = 0;
			// Start response signal asap
			if(~(resp>>i) & 1){
				WIRE1_PORT_TO_SEND;
				_delay_us(50);
				WIRE1_PORT_TO_RECEIVE;
			}
			i++;
		}
		else{
			if((WIRE1_PIN & _BV(WIRE1)) && startTime)
				startTime = 0;
		}
	}

}
