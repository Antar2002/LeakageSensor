#include "nrf24/mirf.h"


/*
	1 - замер фоновой емкости
	2 - замер текущей емкости
	3 - запрос адреса
*/
static volatile char mode;

#define buffersize mirf_PAYLOAD
uint8_t buffer[buffersize];

extern char dev_address[2];
extern char noAddress;

extern char treshold;				// Разница между нормой и текущим уровнем, при которой будет тревога
extern int min_bat_level;			// Минимальный уровень батареи, после которого будет сигнал о необходимости замены
extern int alarm_ignore_time; 		// Время, на которое отключается тревога при наличии сигнала с сенсора (в сек * 4)
extern int alarm_time_limit;		// Продолжительность сигнала тревоги (в сек * 4). Если тревога длится это время - она будет отключена на alarm_ignore_time

volatile int hackLedCnt;
