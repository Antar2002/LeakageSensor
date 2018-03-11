#if defined(__AVR_ATmega88__) \
|| defined(__AVR_ATmega88A__) \
|| defined(__AVR_ATmega88P__) \
|| defined(__AVR_ATmega88PA__)
#define		SENSOR_DDR		DDRD
#define		SENSOR_PORT		PORTD
#define		SENSOR_PIN		PIND
#define		SENSOR			PD5				// Must be TIMER1 input (T1)
//#define		SENSOR			PB0			// Must be ICP1 (for ATMEGA88 it's PB0) - NOT ACTUAL

#define		WIRE1_ADDR_DDR		DDRC
#define		WIRE1_ADDR_PORT		PORTC
#define		WIRE1_ADDR_PIN		PINC
#define		WIRE1_ADDR_1		PC0
#define		WIRE1_ADDR_2		PC1

#define		LED_DDR			DDRC
#define		LED_PORT		PORTC
#define		LED_PIN			PC3

#define		BUTTON_DDR		DDRD
#define		BUTTON_PORT		PORTD
#define		BUTTON_PIN		PIND
#define		BUTTON			PD2			// Must be INTO input
#endif

#if defined(__AVR_ATtiny24A__) \
|| defined(__AVR_ATtiny48__)
#define		SENSOR_DDR		DDRA
#define		SENSOR_PORT		PORTA
#define		SENSOR_PIN		PINA
#define		SENSOR			PA4				// Must be TIMER1 input (T1)
//#define		SENSOR			PB0			// Must be ICP1 (for ATMEGA88 it's PB0) - NOT ACTUAL

#define		WIRE1_ADDR_DDR		DDRA
#define		WIRE1_ADDR_PORT		PORTA
#define		WIRE1_ADDR_PIN		PINA
#define		WIRE1_ADDR_1		PA0
#define		WIRE1_ADDR_2		PA1
#endif


#define		IMPULS_MES_TIME	100			// ���-�� ������������ �������, � ������� �������� ���� ��������� ��������
#define		MES_COUNT		10			// ���-�� ������� ������� ��� ����������
//#define		TRESHOLD		0.05
#define		ALARM_MIN_CNT	1			// ���-�� ����������, ����� �������� ����������� ������� (����� 6 ���)

#define		TIMER			250			// ������ ������������ Timer0 (256-6; OVF = 6510,4Hz)

#define		TRESHOLD		5			// ������� ��������� ������� �������, ������������ �������

#define		ENDLESS_LED_CYCLE	255		// �������� ��� ������������ ������� �����������
#define		ENDLESS_LED_PULSE	60000	// �������� ��� ������������ �������� ����������
#define		LED_ALARM_PULSE	3000			// ������ ������� ���������� ��� ������� (� ������������� Timer0)

#define		BTN_NOIS_REDUCE	45			// ~0.05 ���

EEMEM	uint16_t 	default_frequency_stored = 65535;

static volatile char mode;
static volatile char modeChanged = 0;		// ���� ����� ������. ��� ������ ���-�� ������ ����� - ��������� �� ������ ��������� ������. 

static char impulsTimerCnt = IMPULS_MES_TIME;
static volatile uint16_t curFrequency = 0;		// ������� �������. ����������� � ����������.

static volatile char measureCnt = 0;		// ������� ������� �������. ��� ��������� ������� ������� �������� (curFrequency) ����������� ���� ������ ���� �������.
static volatile char freqUpdated = 0;		// ����/�������, ��� ������ ����� ��������� �������

uint16_t lastFrequency = 0;
uint16_t defaultFrequency = 0;

volatile char alarmCnt = 0;					// ������� ���������� ������ ������� �������

char treshold = TRESHOLD;					// ������� ����� ������ � ������� �������, ��� ������� ����� �������

volatile char needHandleTimer0 = 0;

volatile int btnNoiseTimer = 0;
volatile char btnPressed = 0;
volatile int btnPressTimer = 0;
volatile char pressedForResetAlarm = 0; // ����/������� ����, ��� ������ ��������, ����� �������� �������

volatile int led_cnt = 0;
volatile int led_cycle = 0;
volatile int led_cnt_max = 0;

void uart_printInt(uint16_t val);
