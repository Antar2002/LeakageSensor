#include "nrf24/mirf.h"

#define		SENSOR_DDR		DDRD
#define		SENSOR_PORT		PORTD
#define		SENSOR_PIN		PIND
#define		SENSOR			PD5
//#define		SENSOR			PB0			// Must be ICP1 (for ATMEGA88 it's PB0) - NOT ACTUAL

#define		LED_DDR			DDRC
#define		LED_PORT		PORTC
#define		LED_PIN			PC5

#define		BUZZER_DDR		DDRC
#define		BUZZER_PORT		PORTC
#define		BUZZER_PIN		PC3

#define		BUTTON_DDR		DDRD
#define		BUTTON_PORT		PORTD
#define		BUTTON_PIN		PIND
#define		BUTTON			PD2			// Must be INTO input

#define		HACK_LED_DDR			DDRC
#define		HACK_LED_PORT			PORTC
#define		HACK_LED_PIN			PC0


//#define		IMPULS_COUNT	30			// ���-�� �������� ������� ��� ������
#define		IMPULS_MES_TIME	100			// ���-�� ������������ �������, � ������� �������� ���� ��������� ��������
// 1/6510,4Hz = 0,0001536 ��� * 100 = 0,01536 ���

#define		MES_COUNT		10			// ���-�� ������� ������� ��� ����������
#define		TRESHOLD		0.05		// ������� ��������� ������� �������, ������������ �������
#define		ALARM_MIN_CNT	30			// ���-�� ����������, ����� �������� ����������� ������� (����� 6 ���)

#define		TIMER			250			// ������ ������������ Timer0 (256-6; OVF = 6510,4Hz)
#define		TIMER_OVF_SEC	916			// ���-�� ������������ Timer0 �� 1 ������� (��������=256; ����=6)

#define		BTN_NOIS_REDUCE	45			// ~0.05 ���
#define		BTN_SHORT_PRESS_TIME TIMER_OVF_SEC * 3	// ����������������� "������� ������� �� ������"

#define		LED_ALARM_PULSE	3000			// ������ ������� ���������� ��� ������� (� ������������� Timer0)
#define		ALARM_IGNORE_TIME	2		// ����� ������������� ������� ����� �� ������ �������� (� ��� * 4)

#define 	NRF_IRQ 		PD3			// ���������� �� NRF24L01
#define 	buffersize 		mirf_PAYLOAD
#define 	LED_ADDRESS_REQ_PULSE	6000	// ������ ������� ��� ������� ������ (1 ���)

#define		MIN_BAT_LEVEL		242		// ����������� ���������� ������� (2.42�), ����� �������� ������������ ������ � ������ ������ �������
#define		LED_LOWBAT_PULSE	1500	// ����������������� ������� ��� ������ ������ �������
#define		LOWBAT_PERIOD		4		// ������������� ������� ��� ������ ������ �������

#define		ENDLESS_LED_CYCLE	255		// �������� ��� ������������ ������� �����������
#define		ENDLESS_LED_PULSE	60000	// �������� ��� ������������ �������� ����������


//char freqImpulsFlag = 0;		// ����/������� ����, ��� ����� ������ ������ �������. ������������ � ����������
char impulsTimerCnt = IMPULS_MES_TIME;
uint16_t curFrequency = 0;		// ������� �������. ����������� � ����������.
char ovfCnt = 0;				// ������� ������������ ������� 1
uint16_t freqCnt = 0;			// ������� ���������. ���������������� MES_COUNT. ����� ��������� 0 - �������� ������� ��������� � curFrequency

char modeChanged = 0;		// ���� ����� ������. ��� ������ ���-�� ������ ����� - ��������� �� ������ ��������� ������. 
							// ��� ������ ��������� ������ ������������.

char measureCnt = 0;		// ������� ������� �������. ��� ��������� ������� ������� �������� (curFrequency) ����������� ���� ������ ���� �������.
char freqUpdated = 0;		// ����/�������, ��� ������ ����� ��������� �������

uint16_t tmpFrequency = 0;
uint16_t lastFrequency = 0;
uint16_t defaultFrequency = 0;

int timer0OvfSecCnt = 0;	// ������� ������������ Timer0 ��� ������� ������

char alarmCnt = 0;			// ������� ���������� ������ ������� �������
int alarmIgnoreCnt = 0;		// ������� ������ ������������� �������
char pressedForResetAlarm = 0; // ����/������� ����, ��� ������ ��������, ����� �������� �������

int btnNoiseTimer = 0;
char btnPressed = 0;
int btnPressTimer = 0;

int led_cnt = 0;
int led_cycle = 0;
int led_cnt_max = 0;

int sndFreqCnt = 0;
int sndCnt = 0;

char canSleep = 0;
char sleepsCnt = 0;			// ������� ���������

unsigned char adc_data;		// ������ ������, ��������� � ADC
char adcUpdated = 0;		// ���� ����, ��� ������ � ���������� ������� ����������
int vcc = 0;				// variable to hold the value of Vcc - ������� ���������� ������� * 100
char lowBat = 0;			// ���� ������� ������ �������
char lowBatSignalCnt = 0;	// ������� ����� ����� ��������� � ������ ������ ������� (� ������������� ������� watchdog)

// For NRF24L01
uint8_t sendBuf[buffersize];

uint8_t rxaddr[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t txaddr[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

int addressRequestPause = 0;
char sendInProgress = 0;
char dataReceived = 0;

void prepareStatus(uint8_t *sendBuf);
void sendStatus();
int initRadioAddress();

char wire1_slaves = 0;

char test = 0;
