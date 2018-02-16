#include "nrf24/mirf.h"


/*
	1 - замер фоновой емкости
	2 - замер текущей емкости
	3 - запрос адреса
*/
char mode;

#define buffersize mirf_PAYLOAD
uint8_t buffer[buffersize];

extern char dev_address[2];
extern char noAddress;

int hackLedCnt;
