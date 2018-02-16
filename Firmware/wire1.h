#include <avr/io.h>

#define WIRE1_DDR		DDRB
#define WIRE1_PORT		PORTB
#define WIRE1_PIN		PINB
#define WIRE1			PB0

#define WIRE1_PORT_TO_RECEIVE	WIRE1_DDR &= ~_BV(WIRE1_PIN); wire1status &= ~(1<<3)
#define WIRE1_PORT_TO_SEND		WIRE1_DDR |= _BV(WIRE1_PIN); wire1status |= (1<<3)
#define WIRE1_IS_SEND_MODE		wire1status | (1<<3)

char wire1address[3];

// (1<<WIRE1) - предыдущий уровень
// остальное надо согласовывать с битом выше
// (1<<3) - прием или передача (0 - прием, 1 - передача)
// (1<<7) - режим работы (0 - слейв, 1 - мастер)
char wire1status;

// Текущий режим
// 0 - ничего не делаем, слушаем линию
// 1 - отправляем сигнал "присутствие" и ждем команду
// 2 - отправляем Search ROM ($F0)
// 3 - отвечаем на Search ROM своим адресом
// 4 - отправляем Match ROM ($55)
// 5 - принимаем команду от мастера (после сигнала "присутствие")
// 6 - отпправляем команду "передать частоту"
// 7 - передаем частоту (2 байта + CRC)
// 8 - запросить, есть ли протечка
// 9 - ответить на запрос о протечке
char mode;

char wire1SigCnt;		// Счетчик продолжительности текущего сигнала в переполнениях таймера (* 3.2 мкс)
char wire1PauseCnt;		// Счетчик промежутков между сигналами в переполнениях таймера (* 3.2 мкс)

void wire1_init();
void wire1_sendResetAndScan();

