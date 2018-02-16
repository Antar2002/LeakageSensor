#include <avr/interrupt.h>
//#include <util/delay.h>
//#include <util/delay_basic.h>

#include "wire1_shared.h"

int wire1_waitForImpulsStart(char timeout){

	char sigLength = 0;

	cli();
	WIRE1_TIMER_CNT = 0;
	sei();

	while((WIRE1_PIN & _BV(WIRE1)) && sigLength<20){
		sigLength = WIRE1_TIMER_CNT;
	}

	return sigLength;
}

/*void wire1_sendSignal(char sigLength){
	WIRE1_PORT_TO_SEND;
	//_delay_us(sigLength);
	delay_us2(sigLength);
	WIRE1_PORT_TO_RECEIVE;
}

void delay_us2(int __us){
	uint8_t __ticks;
	int __tmp ; 

	__tmp = ((F_CPU) / 3e6) * __us;
	if (__tmp < 1.0)
		__ticks = 1;
	else
		__ticks = (uint8_t)__tmp;
	_delay_loop_1(__ticks);
}*/
