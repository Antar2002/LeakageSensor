//#define UART_ENABLED

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <util/delay.h>
#include "LeakageSensor_Slave.h"
#include "D:\_Projects\MK\LeakageSensor\Firmware\wire1\wire1_lite.h"

#ifdef UART_ENABLED
#include "D:\_Projects\MK\LeakageSensor\Firmware\uart.h"
#endif

#define UART_BAUD_RATE 56000
#define UART_RX_BUFFER_SIZE 40
#define UART_TX_BUFFER_SIZE 40


uint8_t mcusr_mirror __attribute__ ((section (".noinit")));


void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
	mcusr_mirror = MCUSR;
	MCUSR = 0;
	wdt_disable();
}


ISR(TIMER0_OVF_vect)
{

	// Отсчет времени, в течение которого считается кол-во импульсов с датчика
	impulsTimerCnt--;
	if(impulsTimerCnt==0){
		//curFrequency = freqCnt;
		//freqCnt = 0;
		curFrequency = TCNT1;
		TCNT1 = 0;
		impulsTimerCnt = IMPULS_MES_TIME;
		freqUpdated = 1;
	}

	needHandleTimer0 = 1;

	TCNT0 = TIMER;
}

void handleTimer0(){

	// Подавление дребезга кнопки
	if(btnNoiseTimer>0){
		btnNoiseTimer--;
		if(btnNoiseTimer==0){
			btnPressed = 1;
			btnPressTimer = 0;
		}
	}

	// Обработка мигания диодом
	if(led_cnt<=0){
		if(led_cycle<=0){
			LED_PORT &= ~_BV(LED_PIN);
		}
		else{
			LED_PORT ^= _BV(LED_PIN);
			// Если кол-во циклов ENDLESS_LED_CYCLE и более - бесконечное мигание, пока кто-то не обнулит led_cycle
			if(led_cycle < ENDLESS_LED_CYCLE)
				led_cycle--;
			led_cnt = led_cnt_max;
		}
	}
	else{
		// Считать время свечения диода только, если счетчик меньше 60 000. Иначе его можно включать и вылкючать извне
		if(led_cnt < ENDLESS_LED_PULSE)
			led_cnt--;
	}

}

// Обработка кнопки
ISR(INT0_vect)
{
	// Кнопку нажали
	if(~BUTTON_PIN & _BV(BUTTON)){
		if(btnNoiseTimer==0 && !btnPressed){
			btnNoiseTimer = BTN_NOIS_REDUCE;
		}
	}
	// Кнопку отпустили
	else{
		// Если кнопка сочтена нажатой и была отпущена - обработать время нажатия
		if(btnPressed && !pressedForResetAlarm){
			modeChanged = 1;
			mode = 1;
		}
		btnNoiseTimer = 0;
		btnPressed = 0;
		pressedForResetAlarm = 0;
	}
}

// Прерывание по 1-wire
ISR(PCINT2_vect)
{
	char cmd = 0xFF;


	if(WIRE1_PIN & _BV(WIRE1))
		return;

	// Выключить это прерывание
	PCICR &= ~_BV(PCIE2);

	cmd = wire1_listener();

	// If this is "set default" command (check with reversed value)
	if(cmd==0b10100110){
		sendByteSlave(~wire1_address);
		mode = 1;
		modeChanged = 1;
	}
	// If this is "check alarm state" command (check with reversed value)
	if(cmd==0b00000000 && alarmCnt>ALARM_MIN_CNT){
		sendByteSlave(~wire1_address);
	}

	// Включить это прерывание
	PCICR |= _BV(PCIE2);

}

char measureFrequency(){

	uint16_t tmpFrequency = 0;

	// Если это первый запуск - инициализировать счетчик замеров и т.п.
	if(measureCnt==0){
		measureCnt = MES_COUNT;
		cli();
		tmpFrequency = curFrequency;
		freqUpdated = 0;
		sei();
	}

	// Если получен очередной замер частоты
	if(freqUpdated){
		cli();
		// Усреднять, только если это не первый замер
		if(measureCnt < MES_COUNT)
			tmpFrequency = (tmpFrequency>>1) + (curFrequency>>1);
		freqUpdated = 0;
		measureCnt--;
		if(measureCnt==0){
			lastFrequency = tmpFrequency;
			return 0;
		}
		sei();		
	}
	return 1;
}

void portsInit()
{
	// Init sensor input
	SENSOR_DDR &= ~_BV(SENSOR);
	SENSOR_PORT |= _BV(SENSOR);

	// wire-1 address
	WIRE1_ADDR_DDR &= ~(_BV(WIRE1_ADDR_1) | _BV(WIRE1_ADDR_2));
	WIRE1_ADDR_PORT |= _BV(WIRE1_ADDR_1) | _BV(WIRE1_ADDR_2);

	// Initialize LED (PC5)
	LED_DDR |= _BV(LED_PIN);
	LED_PORT &= ~_BV(LED_PIN);

	// Initialize button (PD2)
	BUTTON_DDR &= ~(_BV(BUTTON));
	BUTTON_PORT |= _BV(BUTTON);
}


void timersInit()
{
	//Init Timer 0
	initTimer0ForSignal();

	// Init timer 1 (for sensor frequency measuring)
	TCCR1A = 0x00;
	TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10);
	TCNT1=0x00;
	TIMSK1 = 0x00;


}

void initTimer0ForSignal(){
	TCCR0B = (1<<CS02);		// /256 (39,0625 kHz / 256 = 152,588)
	TCNT0 = TIMER;
	TIMSK0 = (1<<TOIE0); // Включить прерывание по переполнению таймера
}

void initTimer0ForWire1(){
	TCCR0B = _BV(CS01);		// /8 (1250 kHz / 256 = 4882,8125)
	TCNT0 = 0;
	TIMSK0 = 0; // Выключить прерывание по переполнению таймера
}

void wire1_addr_init(){

	char offset;

	offset = (WIRE1_ADDR_PIN & _BV(WIRE1_ADDR_1))>>WIRE1_ADDR_1;
	offset |=  (WIRE1_ADDR_PIN & _BV(WIRE1_ADDR_2))>>WIRE1_ADDR_2-1;

	wire1_address = (1<<offset);

}


int main(void)
{

	DDRC = 0x00;
	PORTC = 0xFF;

	portsInit();

	cli();

	// Init INT0
	EICRA = 0x00;
	EICRA |= _BV(ISC00);
	EIMSK = 0x00;
	EIMSK |= _BV(INT0);

	// INT of 1-wire
	PCMSK2 |= _BV(PCINT23);
	PCICR |= _BV(PCIE2);

	// Disable TWI
	TWCR &= ~_BV(TWEN);

	timersInit();

	// Disable Analog Comparator
	ACSR &= ~_BV(ACIE);
	ACSR |= _BV(ACD);

	// Не делать так - отключает цифровые входы нафиг
	/*DIDR1 = 0xFF;
	DIDR0 = 0xFF;*/

	wire1_init();
	wire1_addr_init();


	SPCR &= ~_BV(SPE);

	// Load default frequency from EEPROM
	defaultFrequency = eeprom_read_word(&default_frequency_stored);

	if(defaultFrequency > 5000)
		mode = 1;
	else
		mode = 2;
	modeChanged = 1;

#ifdef UART_ENABLED
		// Init uart
		uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
#endif

	sei();

#ifdef UART_ENABLED
	uart_puts("Start level: ");
	uart_printInt(defaultFrequency);
	uart_putc('\r');
#endif


	treshold = TRESHOLD;

DDRB |= _BV(PB0);
PORTB &= ~_BV(PB0);



	while(1)
	{
		wdt_reset();

		if(needHandleTimer0){
			// Обработать то, что должно было быть сделано прерыванием TIMER0
			cli();
			needHandleTimer0 = 0;
			handleTimer0();
			sei();
		}

		// Основные режимы работы
		switch(mode){
			case 1:		// Замер фоновой частоты
				if(modeChanged){
					startLed(ENDLESS_LED_CYCLE, ENDLESS_LED_PULSE);
					modeChanged = 0;
				}
				if(!measureFrequency()){
					// Если измеренное значение похоже на правду
					if(lastFrequency>100){
						defaultFrequency = lastFrequency;
						// Вычислить и запомнить порог срабатывания, при превышении которого нужно поднять тревогу
						unsigned long volatile tmp = defaultFrequency;
						tmp *= (100 - treshold);
						tmp = tmp / 100; // Вроде работает, когда вот так растянуто по строчкам
						//unsigned long tmp = defaultFrequency * (1-0.05); // Работает
						defaultFrequency = tmp;

						// Save this value to EEPROM
						eeprom_write_word (&default_frequency_stored, defaultFrequency);

#ifdef UART_ENABLED
						uart_puts("Def updated: ");
						uart_printInt(defaultFrequency);
						uart_putc('-');
						uart_printInt(lastFrequency);
						uart_putc('\r');
#endif

						led_cycle = 0;

						// Перейти в режим мониторинга
						mode = 2;
						modeChanged = 1;
					}
				}
				break;
			case 2:		// Контроль протечки
				if(modeChanged){
					modeChanged = 0;
				}
				if(!measureFrequency()){
					startLed(1, 10);
#ifdef UART_ENABLED
						uart_printInt(lastFrequency);
						uart_puts(" - ");
						uart_printInt(defaultFrequency);
						uart_putc('\r');
#endif
					if(lastFrequency < defaultFrequency){
//startLed(ENDLESS_LED_CYCLE, ENDLESS_LED_PULSE);
						if(alarmCnt<ALARM_MIN_CNT*2){
							alarmCnt++;
						}
						if(alarmCnt>ALARM_MIN_CNT){
							if(led_cnt_max!=LED_ALARM_PULSE)
								startLed(ENDLESS_LED_CYCLE, LED_ALARM_PULSE);
						}
					}
					else{
						if(alarmCnt!=0){
							cli();
							alarmCnt--;
							sei();
						}
						else{
							// Выключить светодиод
							led_cycle = 0;
						}
//led_cycle = 0;
					}
				}
				break;
		}
	}

}

void startLed(int cycles, int ledCnt)
{
	LED_PORT |= _BV(LED_PIN);
	led_cycle = (cycles * 2) - 1;
	led_cnt_max = ledCnt;
	led_cnt = led_cnt_max;
}

void uart_printInt(uint16_t val){
	char res[5];
	utoa(val, res, 10);
	uart_puts(res);
}
