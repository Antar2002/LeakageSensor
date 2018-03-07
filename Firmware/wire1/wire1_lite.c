#include "wire1_shared.h"

void wire1_init(){
	WIRE1_PORT_TO_RECEIVE;
	WIRE1_PORT &= ~_BV(WIRE1);

	// Mega16
#if defined(__AVR_ATxmega16A4__) \
|| defined(__AVR_ATxmega16A4U__) \
|| defined(__AVR_ATxmega16D4__)
#if F_CPU == 10000000  || F_CPU == 8000000
	TCCR2 = _BV(CS21) | _BV(CS20);
#endif
#endif

	//Mega88
#if defined(__AVR_ATmega88__) \
|| defined(__AVR_ATmega88A__) \
|| defined(__AVR_ATmega88P__) \
|| defined(__AVR_ATmega88PA__)
#if F_CPU == 10000000 || F_CPU == 8000000
	TCCR2B = _BV(CS21) | _BV(CS20);	// CLK/32		434.7826 kHz = 2.3uS (overflow 1698.37 per sec)
#endif
#if F_CPU == 1000000
	TCCR2B = _BV(CS21);		// CLK/8		125 kHz (overflow 488,28125 per sec)
#endif
#ifndef F_CPU
#error "F_CPU not defined. It's required for 1-wire lib"
#endif
#endif

	//TINY24
	// Timer shared with frequency detection - must be reinitialized in main function
}
