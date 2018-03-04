#include "nrf24/mirf.h"


/*
	1 - ����� ������� �������
	2 - ����� ������� �������
	3 - ������ ������
*/
static volatile char mode;

#define buffersize mirf_PAYLOAD
uint8_t buffer[buffersize];

extern char dev_address[2];
extern char noAddress;

extern char treshold;				// ������� ����� ������ � ������� �������, ��� ������� ����� �������
extern int min_bat_level;			// ����������� ������� �������, ����� �������� ����� ������ � ������������� ������
extern int alarm_ignore_time; 		// �����, �� ������� ����������� ������� ��� ������� ������� � ������� (� ��� * 4)
extern int alarm_time_limit;		// ����������������� ������� ������� (� ��� * 4). ���� ������� ������ ��� ����� - ��� ����� ��������� �� alarm_ignore_time

volatile int hackLedCnt;
