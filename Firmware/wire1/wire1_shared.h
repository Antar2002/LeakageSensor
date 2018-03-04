#include <avr/io.h>

// Mega88
#if defined(__AVR_ATmega88__) \
|| defined(__AVR_ATmega88A__) \
|| defined(__AVR_ATmega88P__) \
|| defined(__AVR_ATmega88PA__)
#define WIRE1_DDR		DDRD
#define WIRE1_PORT		PORTD
#define WIRE1_PIN		PIND
#define WIRE1			PD7

#define WIRE1_TIMER_CNT	TCNT2
#endif

// TINY24
#if defined(__AVR_ATtiny24A__) \
|| defined(__AVR_ATtiny48__)
#define WIRE1_DDR		DDRA
#define WIRE1_PORT		PORTA
#define WIRE1_PIN		PINA
#define WIRE1			PA0

#define WIRE1_TIMER_CNT	TCNT0
#endif

#define WIRE1_PORT_TO_RECEIVE	WIRE1_DDR &= ~_BV(WIRE1)
#define WIRE1_PORT_TO_SEND		WIRE1_DDR |= _BV(WIRE1)

char wire1_address;

int wire1_waitForImpulsStart(char timeout);
