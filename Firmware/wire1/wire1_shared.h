#include <avr/io.h>

#define WIRE1_DDR		DDRB
#define WIRE1_PORT		PORTB
#define WIRE1_PIN		PINB
#define WIRE1			PB0

#define WIRE1_TIMER_CNT	TCNT2

#define WIRE1_PORT_TO_RECEIVE	WIRE1_DDR &= ~_BV(WIRE1)
#define WIRE1_PORT_TO_SEND		WIRE1_DDR |= _BV(WIRE1)

char wire1_address;

int wire1_waitForImpulsStart(char timeout);
