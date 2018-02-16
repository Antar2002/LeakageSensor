#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "wire1_shared.h"
#include "wire1_lite_master.h"


char wire1_setDefault(){

	if(!wire1_sendReset())
		return;

	// Send command
	wire1_sendByteMaster(0b01100101);

	// Collect responose
	char resp = wire1_getResponse();

	return resp;
}

char wire1_checkAlarmStatus(){

	if(!wire1_sendReset())
		return;

	// Send command
	wire1_sendByteMaster(0x00);

	// Collect responose
	char resp = wire1_getResponse();

	return resp;
}


char wire1_sendReset(){

	char sigLength;

	// Send "reset"
	WIRE1_PORT_TO_SEND;
	_delay_us(480);
	WIRE1_PORT_TO_RECEIVE;

	// Wait for start of "present" impuls
	sigLength = wire1_waitForImpulsStart(20);
	if(sigLength>=20)
		return 0;

	// Wait for end of "present" impuls
	cli();
	WIRE1_TIMER_CNT = 0;
	sei();
	sigLength = 0;
	while((~WIRE1_PIN & _BV(WIRE1)) && sigLength<35){
		sigLength = WIRE1_TIMER_CNT;
	}
	if(sigLength>20 || sigLength<17)
		return 0;

	return 1;
}

void wire1_sendByteMaster(char cmd){
	for(char i=0;i<8;i++){
		if(cmd & 1){
			WIRE1_PORT_TO_SEND;
			_delay_us(6);
			WIRE1_PORT_TO_RECEIVE;
			_delay_us(60);
		}
		else{
			WIRE1_PORT_TO_SEND;
			_delay_us(60);
			WIRE1_PORT_TO_RECEIVE;
			_delay_us(10);
		}
		cmd>>=1;
	}
}

char wire1_getResponse(){

	char resp = 0;
	for(char i=0;i<8;i++){
		// Send synchronization impuls
		WIRE1_PORT_TO_SEND;
		_delay_us(6);
		WIRE1_PORT_TO_RECEIVE;
		_delay_us(9);
		if(WIRE1_PIN & _BV(WIRE1))
			resp |= 1;
		if(i<7)
			resp<<=1;
		_delay_us(55);
	}

	return resp;
}
