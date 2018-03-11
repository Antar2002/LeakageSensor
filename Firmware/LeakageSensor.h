#include "nrf24/mirf.h"

#define		SENSOR_DDR		DDRD
#define		SENSOR_PORT		PORTD
#define		SENSOR_PIN		PIND
#define		SENSOR			PD5				// Must be TIMER1 input (T1)
//#define		SENSOR			PB0			// Must be ICP1 (for ATMEGA88 it's PB0) - NOT ACTUAL

#define		POWER555_DDR		DDRC
#define		POWER555_PORT		PORTC
#define		POWER555_PIN		PC4

#define		LED_DDR			DDRC
#define		LED_PORT		PORTC
#define		LED_PIN			PC3

#define		BUZZER_DDR		DDRC
#define		BUZZER_PORT		PORTC
#define		BUZZER_PIN		PC2

#define		BUTTON_DDR		DDRD
#define		BUTTON_PORT		PORTD
#define		BUTTON_PIN		PIND
#define		BUTTON			PD2			// Must be INTO input

#define		HACK_LED_DDR			DDRC
#define		HACK_LED_PORT			PORTC
#define		HACK_LED_PIN			PC5

// ���, ������������, ���� �� ������� �������
// ���� ���� ������� ������� - �� ����
#define		EXTERNAL_POWER_DDR		DDRD
#define		EXTERNAL_POWER_PORT		PORTD
#define		EXTERNAL_POWER_PIN		PIND
#define		EXTERNAL_POWER			PD4

#define		WIRE1_ADDR_DDR		DDRC
#define		WIRE1_ADDR_PORT		PORTC
#define		WIRE1_ADDR_PIN		PINC
#define		WIRE1_ADDR_1		PC0
#define		WIRE1_ADDR_2		PC1

// ��� ����������� ������� ������� �� �������� 1-wire ����������
#define		WIRE1_POWER_DDR		DDRD
#define		WIRE1_POWER_PORT	PORTD
#define		WIRE1_POWER			PD6


//#define		IMPULS_COUNT	30			// ���-�� �������� ������� ��� ������
#define		IMPULS_MES_TIME	100			// ���-�� ������������ �������, � ������� �������� ���� ��������� ��������
// 1/6510,4Hz = 0,0001536 ��� * 100 = 0,01536 ���

#define		MES_COUNT		10			// ���-�� ������� ������� ��� ����������
//#define		TRESHOLD		0.05
#define		ALARM_MIN_CNT	10			// ���-�� ����������, ����� �������� ����������� ������� (����� 6 ���)

#define		TIMER			250			// ������ ������������ Timer0 (256-6; OVF = 6510,4Hz)
#define		TIMER_OVF_SEC	916			// ���-�� ������������ Timer0 �� 1 ������� (��������=256; ����=6)

#define		BTN_NOIS_REDUCE	45			// ~0.05 ���
#define		BTN_SHORT_PRESS_TIME TIMER_OVF_SEC * 3	// ����������������� "������� ������� �� ������"

#define		LED_ALARM_PULSE	3000			// ������ ������� ���������� ��� ������� (� ������������� Timer0)
#define		MAX_ALARM_TIME		150		// ������������ ����������������� ������ ������� (� ��� * 4)

#define 	NRF_IRQ 		PD3			// ���������� �� NRF24L01
#define 	buffersize 		mirf_PAYLOAD
#define 	LED_ADDRESS_REQ_PULSE	6000	// ������ ������� ��� ������� ������ (1 ���)

#define		LED_LOWBAT_PULSE	1500	// ����������������� ������� ��� ������ ������ �������
#define		LOWBAT_PERIOD		4		// ������������� ������� ��� ������ ������ �������

#define		ENDLESS_LED_CYCLE	255		// �������� ��� ������������ ������� �����������
#define		ENDLESS_LED_PULSE	60000	// �������� ��� ������������ �������� ����������

#define		NEED_RESET_AFTER	5		// ���-�� �������� notMeasuredCnt, ����� �������� ��������� ������������


//char freqImpulsFlag = 0;		// ����/������� ����, ��� ����� ������ ������ �������. ������������ � ����������
static char impulsTimerCnt = IMPULS_MES_TIME;
static volatile uint16_t curFrequency = 0;		// ������� �������. ����������� � ����������.
//char ovfCnt = 0;				// ������� ������������ ������� 1
//uint16_t freqCnt = 0;			// ������� ���������. ���������������� MES_COUNT. ����� ��������� 0 - �������� ������� ��������� � curFrequency

static volatile char modeChanged = 0;		// ���� ����� ������. ��� ������ ���-�� ������ ����� - ��������� �� ������ ��������� ������. 
							// ��� ������ ��������� ������ ������������.

static volatile char measureCnt = 0;		// ������� ������� �������. ��� ��������� ������� ������� �������� (curFrequency) ����������� ���� ������ ���� �������.
static volatile char freqUpdated = 0;		// ����/�������, ��� ������ ����� ��������� �������

uint16_t lastFrequency = 0;
uint16_t defaultFrequency = 0;

//int timer0OvfSecCnt = 0;	// ������� ������������ Timer0 ��� ������� ������
volatile char needHandleTimer0 = 0;

volatile char alarmCnt = 0;			// ������� ���������� ������ ������� �������
volatile int alarmIgnoreCnt = 0;		// ������� ������� ������������� ������� (������ � watchdog)
volatile char pressedForResetAlarm = 0; // ����/������� ����, ��� ������ ��������, ����� �������� �������
volatile int alarmTimeCnt = 0;		// ������� ������� ������� (������ � watchdog)

volatile int btnNoiseTimer = 0;
volatile char btnPressed = 0;
volatile int btnPressTimer = 0;

volatile int led_cnt = 0;
volatile int led_cycle = 0;
volatile int led_cnt_max = 0;

static int sndFreqCnt = 0;
static volatile int sndCnt = 0;

static char canSleep = 0;
//char sleepsCnt = 0;			// ������� ���������

volatile unsigned char adc_data;		// ������ ������, ��������� � ADC
volatile char adcUpdated = 0;		// ���� ����, ��� ������ � ���������� ������� ����������
int vcc = 0;				// variable to hold the value of Vcc - ������� ���������� ������� * 100
volatile char lowBat = 0;			// ���� ������� ������ �������
volatile char lowBatSignalCnt = 0;	// ������� ����� ����� ��������� � ������ ������ ������� (� ������������� ������� watchdog)

// For NRF24L01
static char isMirfAvailable = 0;

uint8_t sendBuf[buffersize];

uint8_t rxaddr[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t txaddr[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

volatile int addressRequestPause = 0;
static volatile char sendInProgress = 0;
static volatile char dataReceived = 0;
static char txMode = 0;
//extern uint16_t txTimeoutCnt;		// ������� �������� ��������
static volatile uint16_t txTimeoutCnt;			// ������� �������� ��������
static uint16_t txPauseCnt;			// ���������� ����� ���������� (� ������������� TIMER0)

char wire1_slaves = 0;			// ����� ������������ 1-wire �����������
char wire1_last_status = 0;		// ��������� ������, ���������� �� 1-wire ����������� (���� ��� ���� - �������� ���)

char hasExternalPower = 0;	// ���� ����, ��� ���� ������� �������

static volatile char txQuality = 0;			// �������� �������� OBSERVE_TX ����� ��������� ��������

volatile char needHandleWatchdog = 0;
volatile char notMeasuredCnt = 0;	// ������� ���������� ����������. ������������ ��� ������ �������� ��������� �������. ���������������� � Watchdog.

char test = 0;


// Function declarations

void handleWatchDog();
void handleTimer0();

void disableAllBeforeSleep();
void enableAllAfterSleep();

void prepareStatus();
void preparePowerStatus();
void prepareResetStatus(char status);
void startSend();
//void prepareSendBuf(uint8_t *sendBuf, char cmd);

int initRadioAddress();

void ignoreAlarm();

void startLed(int cycles, int ledCnt);
void turnLedOff();
