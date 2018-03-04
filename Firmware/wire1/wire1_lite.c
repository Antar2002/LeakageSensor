#include "wire1_shared.h"

void wire1_init(){
	WIRE1_PORT_TO_RECEIVE;
	WIRE1_PORT &= ~_BV(WIRE1);

	// Mega16
#if defined(__AVR_ATxmega16A4__) \
|| defined(__AVR_ATxmega16A4U__) \
|| defined(__AVR_ATxmega16D4__)
	TCCR2 = _BV(CS21) | _BV(CS20);
#endif

	//Mega88
#if defined(__AVR_ATmega88__) \
|| defined(__AVR_ATmega88A__) \
|| defined(__AVR_ATmega88P__) \
|| defined(__AVR_ATmega88PA__)
	TCCR2B = _BV(CS21) | _BV(CS20);	// CLK/32		434.7826 kHz (overflow 1698.37 per sec)
#endif

	//TINY24
	// Timer shared with frequency detection - must be reinitialized in main function
}
