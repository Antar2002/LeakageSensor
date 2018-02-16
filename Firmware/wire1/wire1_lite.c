#include "wire1_shared.h"

void wire1_init(){
	WIRE1_PORT_TO_RECEIVE;
	WIRE1_PORT &= ~_BV(WIRE1);

	wire1_address = 0b00001000;
}
