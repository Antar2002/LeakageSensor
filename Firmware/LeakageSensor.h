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

// Пин, показывающий, есть ли внешнее питание
// Если есть внешнее питание - не спим
#define		EXTERNAL_POWER_DDR		DDRD
#define		EXTERNAL_POWER_PORT		PORTD
#define		EXTERNAL_POWER_PIN		PIND
#define		EXTERNAL_POWER			PD4

#define		WIRE1_ADDR_DDR		DDRC
#define		WIRE1_ADDR_PORT		PORTC
#define		WIRE1_ADDR_PIN		PINC
#define		WIRE1_ADDR_1		PC0
#define		WIRE1_ADDR_2		PC1

// Пин управляющей подачей питания на дочерние 1-wire устройства
#define		WIRE1_POWER_DDR		DDRD
#define		WIRE1_POWER_PORT	PORTD
#define		WIRE1_POWER			PD6


//#define		IMPULS_COUNT	30			// Кол-во периодов частоты при замере
#define		IMPULS_MES_TIME	100			// Кол-во переполнений таймера, в течение которого буду считаться импульсы
// 1/6510,4Hz = 0,0001536 сек * 100 = 0,01536 сек

#define		MES_COUNT		10			// Кол-во замеров частоты для усреднения
//#define		TRESHOLD		0.05
#define		ALARM_MIN_CNT	10			// Кол-во превышений, после которого объявляется тревога (около 6 сек)

#define		TIMER			250			// Период переполнения Timer0 (256-6; OVF = 6510,4Hz)
#define		TIMER_OVF_SEC	916			// Кол-во переполнений Timer0 за 1 секунду (делитель=256; цикл=6)

#define		BTN_NOIS_REDUCE	45			// ~0.05 сек
#define		BTN_SHORT_PRESS_TIME TIMER_OVF_SEC * 3	// Продолжительность "долгого нажатия на кнопку"

#define		LED_ALARM_PULSE	3000			// Период мигания светодиода при тревоге (в переполнениях Timer0)
#define		MAX_ALARM_TIME		150		// Максимальная продолжительность игнала тревоги (в сек * 4)

#define 	NRF_IRQ 		PD3			// Прерывание от NRF24L01
#define 	buffersize 		mirf_PAYLOAD
#define 	LED_ADDRESS_REQ_PULSE	6000	// Период мигания при запросе адреса (1 сек)

#define		LED_LOWBAT_PULSE	1500	// Продолжительность вспышки при низком уровне батареи
#define		LOWBAT_PERIOD		4		// Периодичность сигнала при низком уровне батареи

#define		ENDLESS_LED_CYCLE	255		// Значение для бесконечного мигания светодиодом
#define		ENDLESS_LED_PULSE	60000	// Значение для бесконечного свечения светодиода

#define		NEED_RESET_AFTER	5		// Кол-во отсчетов notMeasuredCnt, после которого требуется перезагрузка


//char freqImpulsFlag = 0;		// Флаг/признак того, что можно начать отсчет частоты. Используется в прерывании
static char impulsTimerCnt = IMPULS_MES_TIME;
static volatile uint16_t curFrequency = 0;		// Текущая частота. Заполняется в прерывании.
//char ovfCnt = 0;				// Счетчик переполнений таймера 1
//uint16_t freqCnt = 0;			// Счетчик импульсов. Инициализируется MES_COUNT. Когда достигнет 0 - значение таймера запишется в curFrequency

static volatile char modeChanged = 0;		// Флаг смены режима. Как только кто-то меняет режим - взводится до начала обработки режима. 
							// При начале обработки режима сбрасывается.

static volatile char measureCnt = 0;		// Счетчик замеров частоты. При измерении частоты текущее значение (curFrequency) усредняется пока тикает этот счетчик.
static volatile char freqUpdated = 0;		// Флаг/признак, что пришло новое измерение частоты

uint16_t lastFrequency = 0;
uint16_t defaultFrequency = 0;

//int timer0OvfSecCnt = 0;	// Счетчик переполнений Timer0 для отсчета секунд
volatile char needHandleTimer0 = 0;

volatile char alarmCnt = 0;			// Счетчик превышений порога фоновой частоты
volatile int alarmIgnoreCnt = 0;		// Счетчик времени игнорирования тревоги (тикает в watchdog)
volatile char pressedForResetAlarm = 0; // Флаг/признак того, что кнопку нажимали, чтобы сбросить тревогу
volatile int alarmTimeCnt = 0;		// Счетчик времени тревоги (тикает в watchdog)

volatile int btnNoiseTimer = 0;
volatile char btnPressed = 0;
volatile int btnPressTimer = 0;

volatile int led_cnt = 0;
volatile int led_cycle = 0;
volatile int led_cnt_max = 0;

static int sndFreqCnt = 0;
static volatile int sndCnt = 0;

static char canSleep = 0;
//char sleepsCnt = 0;			// Счетчик засыпаний

volatile unsigned char adc_data;		// Чистые данные, полученые с ADC
volatile char adcUpdated = 0;		// Флаг того, что данные о напряжении батареи обновились
int vcc = 0;				// variable to hold the value of Vcc - Текущее напряжение питания * 100
volatile char lowBat = 0;			// Флаг низкого заряда батареи
volatile char lowBatSignalCnt = 0;	// Счетчик паузы между сигналами о низком заряде батареи (в переполнениях таймера watchdog)

// For NRF24L01
static char isMirfAvailable = 0;

uint8_t sendBuf[buffersize];

uint8_t rxaddr[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t txaddr[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

volatile int addressRequestPause = 0;
static volatile char sendInProgress = 0;
static volatile char dataReceived = 0;
static char txMode = 0;
//extern uint16_t txTimeoutCnt;		// Счетчик ожидания передачи
static volatile uint16_t txTimeoutCnt;			// Счетчик ожидания передачи
static uint16_t txPauseCnt;			// Промежуток между передачами (в переполнениях TIMER0)

char wire1_slaves = 0;			// Маска подключенных 1-wire подчиненных
char wire1_last_status = 0;		// Последний статус, полученный от 1-wire подчиненных (если все нули - протечки нет)

char hasExternalPower = 0;	// Флаг того, что есть внешнее питание

static volatile char txQuality = 0;			// Значение регистра OBSERVE_TX после последней передачи

volatile char needHandleWatchdog = 0;
volatile char notMeasuredCnt = 0;	// Счетчик отсутствия активности. Сбрасывается при каждой проверке показаний сенсора. Инкрементируется в Watchdog.

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
