#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "wire1_shared.h"
#include "wire1_lite_slave.h"
#include "wire1_timing.h"

char wire1_listener(){

	char sigLength;

	// If line is "1" do nothing
	if(WIRE1_PIN & _BV(WIRE1))
		return 0xFF;

	cli();
	WIRE1_TIMER_CNT = 0;
	sei();

	sigLength = 0;
	while(~WIRE1_PIN & _BV(WIRE1) && WIRE1_TIMER_CNT < WAIT_FOR_RESET_END){
		sigLength = WIRE1_TIMER_CNT;
	}

	// If this is not "reset" (480uS) - exit
	if(sigLength<RESET_SIG_MIN || sigLength>RESET_SIG_MAX)
		return 0xFF;

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
	while(i < 8 && sigLength < MAX_RC_BIT_LENGTH){
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

	if(sigLength >= MAX_RC_BIT_LENGTH)
		return 0xFF;

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

	return res;

}

void sendByteSlave(char resp){

	char i=0;
	char startTime = 0;

	cli();
	WIRE1_TIMER_CNT = 0;
	sei();

	while(i < 8 && WIRE1_TIMER_CNT < MAX_TX_BIT_LENGTH){
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
