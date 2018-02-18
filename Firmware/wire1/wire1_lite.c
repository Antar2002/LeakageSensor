#include "wire1_shared.h"

void wire1_init(){
	WIRE1_PORT_TO_RECEIVE;
	WIRE1_PORT &= ~_BV(WIRE1);

	// Mega16
	//TCCR2 = _BV(CS21) | _BV(CS20);

	//Mega88
	TCCR2B = _BV(CS21) | _BV(CS20);
}
