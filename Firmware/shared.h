#include "nrf24/mirf.h"

//#define		TIMER_OVF_SEC	916			// ���-�� ������������ Timer0 �� 1 ������� (��������=256; ����=6)
#define		TIMER_OVF_SEC	6510			// ���-�� ������������ Timer0 �� 1 ������� (��������=256; ����=6)

#define		BUTTON_DDR		DDRD
#define		BUTTON_PORT		PORTD
#define		BUTTON_PIN		PIND
#define		BUTTON			PD2			// Must be INTO input

#define		LED_DDR			DDRC
#define		LED_PORT		PORTC
#define		LED_PIN			PC3

/*
	1 - ����� ������� �������
	2 - ����� ������� �������
	3 - ������ ������
*/
volatile char mode;
volatile char modeChanged;		// ���� ����� ������. ��� ������ ���-�� ������ ����� - ��������� �� ������ ��������� ������. 
												// ��� ������ ��������� ������ ������������.

#define buffersize mirf_PAYLOAD
uint8_t buffer[buffersize];

extern char dev_address[2];
extern char noAddress;

extern char treshold;				// ������� ����� ������ � ������� �������, ��� ������� ����� �������
extern int min_bat_level;			// ����������� ������� �������, ����� �������� ����� ������ � ������������� ������
extern int alarm_ignore_time; 		// �����, �� ������� ����������� ������� ��� ������� ������� � ������� (� ��� * 4)
extern int alarm_time_limit;		// ����������������� ������� ������� (� ��� * 4). ���� ������� ������ ��� ����� - ��� ����� ��������� �� alarm_ignore_time

volatile int hackLedCnt;
