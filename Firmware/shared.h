#include "nrf24/mirf.h"


/*
	1 - ����� ������� �������
	2 - ����� ������� �������
	3 - ������ ������
*/
char mode;

#define buffersize mirf_PAYLOAD
uint8_t buffer[buffersize];

extern char dev_address[2];
extern char noAddress;

int hackLedCnt;
