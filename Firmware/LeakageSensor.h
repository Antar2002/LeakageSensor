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


//#define		IMPULS_COUNT	30			// Кол-во периодов частоты при замере
#define		IMPULS_MES_TIME	100			// Кол-во переполнений таймера, в течение которого буду считаться импульсы
// 1/6510,4Hz = 0,0001536 сек * 100 = 0,01536 сек

#define		MES_COUNT		10			// Кол-во замеров частоты для усреднения
#define		TRESHOLD		0.05		// Процент изменения фоновой частоты, обозначающий тревогу
#define		ALARM_MIN_CNT	30			// Кол-во превышений, после которого объявляется тревога (около 6 сек)

#define		TIMER			250			// Период переполнения Timer0 (256-6; OVF = 6510,4Hz)
#define		TIMER_OVF_SEC	916			// Кол-во переполнений Timer0 за 1 секунду (делитель=256; цикл=6)

#define		BTN_NOIS_REDUCE	45			// ~0.05 сек
#define		BTN_SHORT_PRESS_TIME TIMER_OVF_SEC * 3	// Продолжительность "долгого нажатия на кнопку"

#define		LED_ALARM_PULSE	3000			// Период мигания светодиода при тревоге (в переполнениях Timer0)
#define		ALARM_IGNORE_TIME	2		// Время игнорирования тревоги после ее сброса выручную (в сек * 4)

#define 	NRF_IRQ 		PD3			// Прерывание от NRF24L01
#define 	buffersize 		mirf_PAYLOAD
#define 	LED_ADDRESS_REQ_PULSE	6000	// Период мигания при запросе адреса (1 сек)

#define		MIN_BAT_LEVEL		242		// Минимальное напряжение батареи (2.42В), после которого отправляется сигнал о низком заряде батареи
#define		LED_LOWBAT_PULSE	1500	// Продолжительность вспышки при низком уровне батареи
#define		LOWBAT_PERIOD		4		// Периодичность сигнала при низком уровне батареи

#define		ENDLESS_LED_CYCLE	255		// Значение для бесконечного мигания светодиодом
#define		ENDLESS_LED_PULSE	60000	// Значение для бесконечного свечения светодиода


//char freqImpulsFlag = 0;		// Флаг/признак того, что можно начать отсчет частоты. Используется в прерывании
char impulsTimerCnt = IMPULS_MES_TIME;
uint16_t curFrequency = 0;		// Текущая частота. Заполняется в прерывании.
char ovfCnt = 0;				// Счетчик переполнений таймера 1
uint16_t freqCnt = 0;			// Счетчик импульсов. Инициализируется MES_COUNT. Когда достигнет 0 - значение таймера запишется в curFrequency

char modeChanged = 0;		// Флаг смены режима. Как только кто-то меняет режим - взводится до начала обработки режима. 
							// При начале обработки режима сбрасывается.

char measureCnt = 0;		// Счетчик замеров частоты. При измерении частоты текущее значение (curFrequency) усредняется пока тикает этот счетчик.
char freqUpdated = 0;		// Флаг/признак, что пришло новое измерение частоты

uint16_t tmpFrequency = 0;
uint16_t lastFrequency = 0;
uint16_t defaultFrequency = 0;

int timer0OvfSecCnt = 0;	// Счетчик переполнений Timer0 для отсчета секунд

char alarmCnt = 0;			// Счетчик превышений порога фоновой частоты
int alarmIgnoreCnt = 0;		// Счетчик секунд игнорирования тревоги
char pressedForResetAlarm = 0; // Флаг/признак того, что кнопку нажимали, чтобы сбросить тревогу

int btnNoiseTimer = 0;
char btnPressed = 0;
int btnPressTimer = 0;

int led_cnt = 0;
int led_cycle = 0;
int led_cnt_max = 0;

int sndFreqCnt = 0;
int sndCnt = 0;

char canSleep = 0;
char sleepsCnt = 0;			// Счетчик засыпаний

unsigned char adc_data;		// Чистые данные, полученые с ADC
char adcUpdated = 0;		// Флаг того, что данные о напряжении батареи обновились
int vcc = 0;				// variable to hold the value of Vcc - Текущее напряжение питания * 100
char lowBat = 0;			// Флаг низкого заряда батареи
char lowBatSignalCnt = 0;	// Счетчик паузы между сигналами о низком заряде батареи (в переполнениях таймера watchdog)

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
