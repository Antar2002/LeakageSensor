#include "nrf24/mirf.h"

//#define		TIMER_OVF_SEC	916			// Кол-во переполнений Timer0 за 1 секунду (делитель=256; цикл=6)
#define		TIMER_OVF_SEC	6510			// Кол-во переполнений Timer0 за 1 секунду (делитель=256; цикл=6)

#define		BUTTON_DDR		DDRD
#define		BUTTON_PORT		PORTD
#define		BUTTON_PIN		PIND
#define		BUTTON			PD2			// Must be INTO input

#define		LED_DDR			DDRC
#define		LED_PORT		PORTC
#define		LED_PIN			PC3

/*
	1 - замер фоновой емкости
	2 - замер текущей емкости
	3 - запрос адреса
*/
volatile char mode;
volatile char modeChanged;		// Флаг смены режима. Как только кто-то меняет режим - взводится до начала обработки режима. 
												// При начале обработки режима сбрасывается.

#define buffersize mirf_PAYLOAD
uint8_t buffer[buffersize];

extern char dev_address[2];
extern char noAddress;

extern char treshold;				// Разница между нормой и текущим уровнем, при которой будет тревога
extern int min_bat_level;			// Минимальный уровень батареи, после которого будет сигнал о необходимости замены
extern int alarm_ignore_time; 		// Время, на которое отключается тревога при наличии сигнала с сенсора (в сек * 4)
extern int alarm_time_limit;		// Продолжительность сигнала тревоги (в сек * 4). Если тревога длится это время - она будет отключена на alarm_ignore_time

volatile int hackLedCnt;
