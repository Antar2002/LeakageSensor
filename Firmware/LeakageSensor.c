//#define UART_ENABLED
//#define WIRE1_DISABLED
//#define NRF_DISABLED

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <util/delay.h>
#include "LeakageSensor.h"
//#include <util/delay.h>

#ifdef UART_ENABLED
#include "uart.h"
#endif

#ifndef NRF_DISABLED
#include "nrf24/mirf.h"
#include "nrf24/nRF24L01.h"
#endif

#include "addressFn.h"
#include "buttonFn.h"

#ifndef WIRE1_DISABLED
#include "wire1/wire1_lite.h"
#endif

#include "paramsFn.h"
//#include "txQueueFn.h"
#include "uartFn.h"
#include "shared.h"


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


ISR(TIMER1_CAPT_vect)   
{
//	freqCnt++;
}

ISR(TIMER1_OVF_vect){
//	ovfCnt++;
}

ISR(TIMER0_OVF_vect)
{

	// ������ �������, � ������� �������� ��������� ���-�� ��������� � �������
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

	button_timerProc();

	// ��������� ������� ������
	if(led_cnt<=0){
		if(led_cycle<=0){
			LED_PORT &= ~_BV(LED_PIN);
		}
		else{
			LED_PORT ^= _BV(LED_PIN);
			// ���� ���-�� ������ ENDLESS_LED_CYCLE � ����� - ����������� �������, ���� ���-�� �� ������� led_cycle
			if(led_cycle < ENDLESS_LED_CYCLE)
				led_cycle--;
			led_cnt = led_cnt_max;
		}
	}
	else{
		// ������� ����� �������� ����� ������, ���� ������� ������ 60 000. ����� ��� ����� �������� � ��������� �����
		if(led_cnt < ENDLESS_LED_PULSE)
			led_cnt--;
	}

/*	if(hackLedCnt>0){
		HACK_LED_PORT |= _BV(HACK_LED_PIN);
		if(hackLedCnt < ENDLESS_LED_PULSE)
			hackLedCnt--;
	}
	else{
		HACK_LED_PORT &= ~_BV(HACK_LED_PIN);
	}*/

	if(addressRequestPause>0)
		addressRequestPause--;

	if(txPauseCnt>0)
		txPauseCnt--;

	if(txTimeoutCnt>0)
		txTimeoutCnt--;
}

// ��������� ������
ISR(INT0_vect)
{

	// ������ � ������� ���������, �� ������ ��� �� �������
/*	if((~BUTTON_PIN & _BV(BUTTON)) && !btnPressTimer){
		btnProcessed = 0;
		btnPressTimer = 1;
	}*/
/*
// �� ��������
	if(canSleep && (~BUTTON_PIN & _BV(BUTTON)))
		btnPressTimer = BTN_NOIS_REDUCE + 1;*/
}

ISR (WDT_vect)
{

	notMeasuredCnt++;

	needHandleWatchdog = 1;

	WDTCSR |= _BV(WDIE);

}

void handleWatchDog(){

	// ���� ������ ���� ������ ������������� ������� - ��������� �������� ����� ������������� �� 1
	if(alarmIgnoreCnt>0){
		alarmIgnoreCnt--;
	}

	// ���� ���� ������ ������� - ������� ���������������� ��� ������
	if(alarmTimeCnt>0 && alarmTimeCnt<MAX_ALARM_TIME){
		alarmTimeCnt++;
		// ���� ����������������� ������ �������� - ��������� ������ �� �����, �������� alarm_ignore_time
		if(alarmTimeCnt>alarm_time_limit){
			ignoreAlarm();
		}
	}

	/*sleepsCnt++;
	// ������ ������ ��������
	if(sleepsCnt>1){
		// ���� ������ �� ������ ���������� �������, ���������� ���������� �� �������
		if(alarmCnt==0){
			sleepsCnt = 0;
			ADCSRA |= (1<<ADSC);
		}
	}*/
	if(!adcUpdated){
		ADCSRA |= (1<<ADSC);
	}

	// ���� ������ �����, ������� ����� ����� ��������� � ������ ������
	if(lowBat==1 && lowBatSignalCnt>0)
		lowBatSignalCnt--;

}


ISR(ADC_vect) //ADC End of Conversion interrupt 
{
	adc_data = ADC>>2; //read 8 bit value 
	adcUpdated = 1;
}

#ifndef NRF_DISABLED
ISR(INT1_vect)	// Interupt from NRF
{

	uint8_t status = mirf_get_status();
#ifdef UART_ENABLED
test = status;
#endif
//hackLedCnt = 200;
	// ���� ���-�� ������ �� �����
	if(status & (1<<RX_DR)){
		if(dataReceived == 0){
			mirf_read(buffer);
			dataReceived = 1;
		}
	}

	// ���� ��� ���� ��������� ��������
	// TX_DS - �������� ������ �������
	// MAX_RT - �������� ����������� (� TX FIFO �������� ������������ ������)
	if((status & (1<<TX_DS)) || (status & (1<<MAX_RT))){
		txQuality = mirf_read_register(OBSERVE_TX);
		// ���� ����� ��� � �� ������� �������� - ������������������ �� ���� ��������
/*		if(status & (1<<MAX_RT)){
			for(int i=0; i< (txQuality>>4); i++){
				LED_PORT |= _BV(LED_PIN);
				_delay_ms(1000);
				LED_PORT &= ~_BV(LED_PIN);
				_delay_ms(500);
			}
		}*/
		// ��������� ��� � NRF, ���� ����� ���� ���������� ������
		afterWriteData(status);

		sendInProgress = 0;
		txTimeoutCnt = 0;
	}
	mirf_flush_rx();
	mirf_reset();
}
#endif

char receiveRadioData(){
	uint8_t status = mirf_get_status();
	if(status & (1<<RX_DR)){
		if(dataReceived == 0){
			mirf_read(buffer);
			dataReceived = 1;
		}
		//mirf_reset();
		mirf_reset_rx();
		return 1;
	}
	return 0;
}

char measureFrequency(){

	uint16_t tmpFrequency = 0;

	// ���� ��� ������ ������ - ���������������� ������� ������� � �.�.
	if(measureCnt==0){
		measureCnt = MES_COUNT;
		cli();
		tmpFrequency = curFrequency;
		freqUpdated = 0;
		sei();
	}

	// ���� ������� ��������� ����� �������
	if(freqUpdated){
		cli();
		// ���������, ������ ���� ��� �� ������ �����
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

	POWER555_DDR |= _BV(POWER555_PIN);
	POWER555_PORT |= _BV(POWER555_PIN);

	// Initialize LED (PC5)
	LED_DDR |= _BV(LED_PIN);
	LED_PORT &= ~_BV(LED_PIN);

	LED_DDR |= _BV(HACK_LED_PIN);
	LED_PORT &= ~_BV(HACK_LED_PIN);

	// Initialize button (PD2)
	BUTTON_DDR &= ~(_BV(BUTTON) | _BV(NRF_IRQ));
	BUTTON_PORT |= _BV(BUTTON) | _BV(NRF_IRQ);

	// Initialize buzzer (BUZZER_PIN)
	BUZZER_DDR |= _BV(BUZZER_PIN);
	BUZZER_PORT &= ~_BV(BUZZER_PIN);

	// Initialize external power signal
	EXTERNAL_POWER_DDR &= ~_BV(EXTERNAL_POWER);
	EXTERNAL_POWER_PORT &= ~_BV(EXTERNAL_POWER);

#ifndef WIRE1_DISABLED
	// wire-1 address
	WIRE1_ADDR_DDR &= ~(_BV(WIRE1_ADDR_1) | _BV(WIRE1_ADDR_2));
	WIRE1_ADDR_PORT |= _BV(WIRE1_ADDR_1) | _BV(WIRE1_ADDR_2);

	WIRE1_POWER_DDR |= _BV(WIRE1_POWER);
	WIRE1_POWER_PORT |= _BV(WIRE1_POWER);
#endif
}

void timersInit()
{
	//Init Timer 0
	TCCR0B = (1<<CS02);		// /256 (39,0625 kHz / 256 = 152,588)
	TCNT0 = TIMER;
	TIMSK0 = (1<<TOIE0); // �������� ���������� �� ������������ �������

	// Init timer 1 (for sensor frequency measuring)
	TCCR1A = 0x00;
	TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10);
	TCNT1=0x00;
	TIMSK1 = 0x00;
}

void WDT_Init()
{
	wdt_reset();
	wdt_enable(WDTO_4S);
	WDTCSR |= _BV(WDIE);
}

void ADC_init() 
{
	ADMUX |= (1<<MUX3)|(1<<MUX2)|(1<<MUX1); //Set the Band Gap voltage as the ADC input
	ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADSC); 
}

#ifndef WIRE1_DISABLED
void wire1_addr_init(){

	char offset = 0;

	offset = (WIRE1_ADDR_PIN & _BV(WIRE1_ADDR_1))>>WIRE1_ADDR_1;
	offset |=  (WIRE1_ADDR_PIN & _BV(WIRE1_ADDR_2))>>WIRE1_ADDR_2-1;

	wire1_address = (1<<offset);

}
#endif

int main(void)
{

	// Save reset source and disable watchdog ASAP
	//get_mcusr();
	/*mcusr_mirror = MCUSR;
	MCUSR = 0;
	wdt_disable();
	notMeasuredCnt = 0;*/

	portsInit();

	startLed(ENDLESS_LED_CYCLE, ENDLESS_LED_PULSE);

	cli();

#ifdef UART_ENABLED
	// Init uart
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
#endif

#ifndef NRF_DISABLED
	// Init NRF24
	mirf_init();
	_delay_ms(50);
	isMirfAvailable = mirf_is_available();
#else
	isMirfAvailable = 0;
#endif

	// Init INT0 & INT1
	//MCUCR &= ~_BV(ISC00) & ~_BV(ISC01);
	//MCUCR |= _BV(ISC00) | _BV(ISC01);
	//MCUCR &= _BV(ISC00) & _BV(ISC01);	// Low level rise interrupt
	//GIMSK |= _BV(INT0);
	EICRA = 0x00;
	//EICRA |= _BV(ISC00) | _BV(ISC11);	// Any level of INT0 change raise interrupt, falling edge on INT1 raise interupt
	EICRA |= _BV(ISC00);	// Any level of INT0 change raise interrupt
	EIMSK = 0x00;

#ifndef NRF_DISABLED
	if(isMirfAvailable)
		EIMSK |= _BV(INT0) | _BV(INT1);
	else
		EIMSK |= _BV(INT0);
#else
	EIMSK |= _BV(INT0);
#endif

	// Disable TWI
	TWCR &= ~_BV(TWEN);

	timersInit();

	// Disable Analog Comparator
	ACSR &= ~_BV(ACIE);
	ACSR |= _BV(ACD);

	ADC_init();


#ifndef WIRE1_DISABLED
	wire1_init();
	wire1_addr_init();

	// �������� ������� �������
	WIRE1_POWER_PORT &= ~_BV(WIRE1_POWER);
#endif

	button_init();

	sei();

#ifndef NRF_DISABLED
	if(isMirfAvailable){
		mirf_config();
		mirf_set_rxaddr(0, rxaddr);
		mirf_set_txaddr(txaddr);

		// ���������������� ����� ���������� � �����������
		initAddress();
		// ����� ����������
		sendBuf[0] = dev_address[0];
		sendBuf[1] = dev_address[1];

	}
#else
	SPCR &= ~_BV(SPE);
#endif

	// ��������� ��������� �� ������
	readParamsFromMem();

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	WDT_Init();

	// ���������� ���, ��� �� ����������
	URAT_PRINT_STR("Leakage Sensor v0.1\r\r");

	cli();
	turnLedOff();
	
	// ���������� ��������������� ��� ����������
/*	freqImpulsFlag = 0;
	curFrequency = 0;*/
	sendInProgress = 0;
//	txQueue_clear();
	txPauseCnt = 0;
	txTimeoutCnt = 0;

	adcUpdated = 0;
	dataReceived = 0;
	canSleep = 0;

	mode = 1;
	modeChanged = 1;

	sei();

	// ���� ��� ���� ������������ - �������� �� ���� �� ����� � �������� �����
	if(mcusr_mirror){
		//LED_PORT &= ~_BV(LED_PIN);
		prepareResetStatus(mcusr_mirror);
		startSend();

#ifdef UART_ENABLED
		uart_puts("Reset source: ");
		uart_putc(mcusr_mirror);
		uart_putc('\r');
#endif
	}


	while(1)
	{
HACK_LED_PORT ^= _BV(HACK_LED_PIN);

		// ���������������, ���� ������� ���������� �������� ������� ������ ��������� ��������
		if(notMeasuredCnt > NEED_RESET_AFTER){
			cli();
			canSleep = 0;
			wdt_reset();
			WDTCSR |= _BV(WDE) | _BV(WDCE);
			WDTCSR = _BV(WDE);		// Enable reset (and disable interrupt) and prescaler is 16ms
			wdt_reset();
			sei();
			_delay_ms(17);
		}

		if(needHandleWatchdog){
			// ���������� ��� ��, ��� ������ ���� ������� ���������� Watchdog'�
			cli();
			needHandleWatchdog = 0;
			sei();
			handleWatchDog();
		}

		if(needHandleTimer0){
			// ���������� ��, ��� ������ ���� ���� ������� ����������� TIMER0
			cli();
			needHandleTimer0 = 0;
			handleTimer0();
			sei();
		}

		// ��������� �����
		// ���� ������ ��������� ������� ��� ������ ������� ������ �������	
		if((alarmCnt>ALARM_MIN_CNT && alarmIgnoreCnt==0) || sndCnt>0){
			sndFreqCnt++;
			// ��� HCM1206A ����� 2400��
			if(sndFreqCnt>11){
				BUZZER_PORT ^= _BV(BUZZER_PIN);
				sndFreqCnt = 0;
			}
			if(sndCnt>0)
				sndCnt--;
		}

		cli();
		button_handleButton();
		sei();

		// ��������� ������� ������
		if(btnPressed){
			cli();
			// ���� ���� ������� - ��������� � ������ ������������
			if(alarmCnt>ALARM_MIN_CNT){
				ignoreAlarm();
				btnProcessed = 1;
			}
			else{
				// ���� ������ ������ ������� ����� - ������� � ����� ��������� ������, �������� ��������� � ������ �������������
				if(btnPressTimer > BTN_SHORT_PRESS_TIME && mode!=3 && !btnProcessed){
					treshold = TRESHOLD;
					min_bat_level = MIN_BAT_LEVEL;
					alarm_ignore_time = ALARM_IGNORE_TIME;
					alarm_time_limit = ALARM_TIME_LIMIT;

					saveParams();

					alarmIgnoreCnt = 0;

					modeChanged = 1;
					mode = 3;
					btnProcessed = 1;
				}
			}
			sei();
		}

		// �������� ���� "���� ������� �������"
		hasExternalPower = EXTERNAL_POWER_PIN & _BV(EXTERNAL_POWER);

#ifndef NRF_DISABLED
		// ������ ������
		if(dataReceived == 1)
		{
			URAT_PRINT_STR("NRF data received\r");
//hackLedCnt = 4000;
			//mirf_read(buffer);
			// ���������, ��� ��� ������� ��� ������� ����������
			if(checkAddress(buffer))
			{
				// Process command
				switch(buffer[2]){
					// Assign address
					case 0:
						if(mode==3){
							cli();
							assignAddress(buffer);
							sendBuf[0] = dev_address[0];
							sendBuf[1] = dev_address[1];
							modeChanged = 1;
							mode = 2;
							sei();
						}
						break;
					case 1:				// ������ ��������� ������� ������
						if(alarmCnt>ALARM_MIN_CNT){
							ignoreAlarm();
						}
						else{
							cli();
							modeChanged = 1;
							mode = 1;
							sei();
						}
						break;
					case 2:				// ��������������� ����������
						setParams(buffer);
						break;
				}
			}
			dataReceived = 0;
		}
#endif

		if(adcUpdated){
			URAT_PRINT_STR("Update VCC\r");
			vcc = 112 * 255 / adc_data;
			cli();
			adcUpdated = 0;
			sei();
#ifdef UART_ENABLED
			//uart_puts("ADC: ");
			//uart_putc(adc_data);
			//uart_putc('\r');
			//uart_puts("Vcc: ");
			//sprintf(buffer, "%f", vcc);
			//uart_puts(buffer);
			//uart_putc('\r');
#endif
			if(vcc<min_bat_level)
				lowBat = 1;
			else
				lowBat = 0;
			URAT_PRINT_STR("VCC updated\r");
		}

		// ���� ������� ������� ������ � ������� ����� �������� �� ���� �������
		if(lowBat==1 && lowBatSignalCnt==0){
			startLed(1, LED_LOWBAT_PULSE);
			lowBatSignalCnt = LOWBAT_PERIOD;
			sndCnt = 3000;
		}

#ifndef WIRE1_DISABLED
		// ���� ���� ������� ������� - ���������� �� ������� 1-wire, ��� �����������
		if(hasExternalPower==1){
			URAT_PRINT_STR("Listen 1-wire\r");
			char cmd = wire1_listener();
			// If this is "set default" command (check with reversed value)
			if(cmd==0b10100110){
				sendByteSlave(wire1_address);
				mode = 1;
				modeChanged = 1;
			}
			// If this is "check alarm state" command (check with reversed value)
			if(cmd==0b00000000 && alarmCnt>ALARM_MIN_CNT){
				sendByteSlave(wire1_address);
			}
		}
#endif

		if(!txTimeoutCnt){
			cli();
			sendInProgress = 0;
			sei();
		}


#ifndef NRF_DISABLED
		// ��������� ������� �������� �� NRF
		// �������� ��������� ������ � �������, ���� ��� �� ����� � 
/*		if(txQueue_notEmpty() && !sendInProgress){
			switch(txQueue_getNext()){
				case 1:
					prepareStatus();
					startSend();
					break;
				case 2:
					preparePowerStatus();
					startSend();
					break;
			}
		}*/
		if(txMode && !sendInProgress){
			switch(txMode){
				case 1:
					preparePowerStatus();
					startSend();
					txMode = 0;
					break;
				case 2:
					prepareStatus();
					startSend();
					txMode = 1;
					break;
				default:
					txMode = 0;
					break;
			}
		}			
#endif

		// �������� ������ ������
		switch(mode){
			case 1:		// ����� ������� �������
				if(modeChanged){
					URAT_PRINT_STR("Default level reinitializing.\r");
					//LED_PORT |= _BV(LED_PIN);
					startLed(ENDLESS_LED_CYCLE, ENDLESS_LED_PULSE);
					_delay_ms(3000);
					modeChanged = 0;
				}
				if(!measureFrequency()){
					// ���� ���������� �������� ������ �� ������
					if(lastFrequency>100){
						cli();
						notMeasuredCnt = 0;
						sei();
						alarmCnt = 0;

						URAT_PRINT_STR("Default level is ready.\r");
						defaultFrequency = lastFrequency;
						// ��������� � ��������� ����� ������������, ��� ���������� �������� ����� ������� �������
						unsigned long tmp = defaultFrequency;
						tmp *= (100 - treshold);
						tmp = tmp / 100; // ����� ��������, ����� ��� ��� ��������� �� ��������
						//unsigned long tmp = defaultFrequency * (1-0.05); // ��������
						defaultFrequency = tmp;
						//defaultFrequency = defaultFrequency * 0.95;		// ������������ ��������� ����� - �������� �� 8��
						// �������� ���������
						//LED_PORT &= ~_BV(LED_PIN);
						led_cycle = 0;
						// ��������� ������ � UART
						UART_PRINT_DEFAULT_FREQ(defaultFrequency);
#ifndef WIRE1_DISABLED
//						WIRE1_POWER_PORT &= ~_BV(WIRE1_POWER);
						// ����� ����������� � 1-wire, ����� ��� ���� ��������� ������� �������, ��� ������� � ��������� ����� ������������
						wire1_slaves = wire1_setDefault();
						if(wire1_slaves != 0)
							_delay_ms(1000);

//						if(!hasExternalPower)
//							WIRE1_POWER_PORT |= _BV(WIRE1_POWER);
#endif

						// ������� � ����� �����������
						mode = 2;
						modeChanged = 1;

					}
					else{
						// ���� �������� ������� ���� - �������� �������
						alarmCnt = ALARM_MIN_CNT * 2;
						if(led_cnt_max!=LED_ALARM_PULSE)
							startLed(ENDLESS_LED_CYCLE, LED_ALARM_PULSE);
					}
				}
#ifdef UART_ENABLED
else{
	uart_puts("Wait for default level.\r");
	_delay_ms(10);
}
#endif
				break;
			case 2:		// �������� ��������
				if(modeChanged){
					modeChanged = 0;
					URAT_PRINT_STR("Start sensor check\r");
				}
				if(!measureFrequency()){
#ifndef WIRE1_DISABLED
					// ��������� ����������� 1-wire
//					WIRE1_POWER_PORT &= ~_BV(WIRE1_POWER);

					wire1_last_status = wire1_checkAlarmStatus();

//					if(!hasExternalPower && !wire1_last_status)
//						WIRE1_POWER_PORT |= _BV(WIRE1_POWER);
#endif
					// ���� ���������� �������� ������ �� ������
					if(lastFrequency>100){
						// �������� �������, ������� ����� � ������������
						cli();
						notMeasuredCnt = 0;
						sei();
					}
					else{
						// ���� �������� ������� ���� - �������� �������
						alarmCnt = ALARM_MIN_CNT * 2;
					}
					// ���� ����� ��� � ����������� �����
					if((lastFrequency < defaultFrequency || wire1_last_status) && alarmIgnoreCnt==0){
						canSleep = 0;
						if(alarmCnt<ALARM_MIN_CNT*2){
							alarmCnt++;
						}
						// ���� ���-�� ���������� ������� ������� ��������� ����� - ������� �������
						if(alarmCnt>ALARM_MIN_CNT){
							// ������ ������� ����� ������ ������� �������, ���� ��� �� ������ � ���� ��� �������� �������
							if(alarmTimeCnt==0 && !hasExternalPower)
								alarmTimeCnt = 1;
							// ������ ������ �����������, ���� ��� �� ������
							if(led_cnt_max!=LED_ALARM_PULSE)
								startLed(ENDLESS_LED_CYCLE, LED_ALARM_PULSE);
							URAT_PRINT_STR("!!! ALARM !!!\r");
						}
					}
					else{
						if(alarmCnt!=0){
							cli();
							alarmCnt--;
							sei();
						}
						else{
							// ��������� ���������
							turnLedOff();
							// ���� ��� �������� (������� ��� ��� ��� �����������) ����� � �������
							if(!alarmCnt)
								canSleep = 1;
						}
					}

					uart_printCurFrequency(lastFrequency, defaultFrequency, vcc);


#ifndef NRF_DISABLED
					// �������� �� ����� �������� ������� (������ ���� ������ ����� ��� ��������)
					if(isMirfAvailable){
						if(!txPauseCnt){
							cli();
							// �������� NRF �� ����� �������������										
							//mirf_flush_tx();
							//mirf_reset_tx();
							// ������������ ����� ������ ������� ����� ����� ����������
							if(alarmCnt)
								txPauseCnt = 6500;	// ���������� ��� � �������
							else
								txPauseCnt = 26000;	// ���������� ��� � 4 �������
							//txQueue_add(1);
							//txQueue_add(2);
							txMode = 2;
							sei();
						}
					}
#endif
				}
				break;
			// ������ ������ � ���������
			case 3:
				if(modeChanged){
					URAT_PRINT_STR("Wait for address...\r");
					dev_address[0] = rand();
					dev_address[1] = rand();
					modeChanged = 0;
					startLed(ENDLESS_LED_CYCLE, LED_ADDRESS_REQ_PULSE);
				}
				if(!isMirfAvailable){
					// ����� ����������
					sendBuf[0] = dev_address[0];
					sendBuf[1] = dev_address[1];
					led_cycle = 0;
					mode = 2;
				}
				else{
					initRadioAddress();
				}
				break;
			case 4:
				break;
			default:
				cli();
				mode = 2;
				modeChanged = 1;
				sei();
				break;
		}

		if(canSleep){
			// ����� ������ ���� ��� �������� ������� � ��� ��������
			//if(!hasExternalPower && !txQueue_notEmpty() && !sendInProgress){
			if(!hasExternalPower && !txMode && !sendInProgress
				&& !btnPressed && btnPressTimer==0){
#ifdef UART_ENABLED
	uart_puts("Before sleep\r");
	_delay_ms(50);
#endif
				cli();
				disableAllBeforeSleep();
				sei();

				// ������
				sleep_enable();
				sleep_cpu();

				cli();
				// ����������
				sleep_disable();
				canSleep = 0;

				// Initialize LED (PC5)
				LED_DDR |= _BV(LED_PIN);
				// �������, ��� �� ��� ����
				LED_PORT |= _BV(LED_PIN);

				// ������ ��������� ������ ��� ����� ������
				// ����������!!!
				cli();
				button_handleButton();
				sei();

				enableAllAfterSleep();

				// ����� �������� ��� NRF - ����� �� ��������� ���������� ����� ��������� �����
//				_delay_ms(50);
				LED_PORT &= ~_BV(LED_PIN);
				//startLed(1, 600);

				// ��������� ����� �������� ������ �� �����
				txPauseCnt = 0;
				//txQueue_clear();
				txMode = 0;

				sei();

				if(isMirfAvailable){
					mirf_flush_tx();
					mirf_reset_tx();
				}

			}
			else{
				//_delay_ms(100);
				URAT_PRINT_STR("Can't sleep\r");
			}
		}

	}
}

void disableAllBeforeSleep(){

#ifndef NRF_DISABLED
	if(isMirfAvailable){
		// ��������� NRF24
		mirf_power_down;
		// ��������� ���������� �� NRF24
		EIMSK &= ~_BV(INT1);
	}
	SPCR &= ~_BV(SPE);
#endif

#ifndef WIRE1_DISABLED
	WIRE1_POWER_PORT |= _BV(WIRE1_POWER);
#endif

	// ��������� ���������� � �������
	startLed(0, 0);
	hackLedCnt = 0;
	LED_PORT &= ~_BV(LED_PIN);
	HACK_LED_PORT &= ~_BV(HACK_LED_PIN);
	BUZZER_PORT &= ~_BV(BUZZER_PIN);
	/*DDRB = 0x00;
	PORTB = 0xFF;
	DDRC = 0x00;
	PORTC = 0xFF;
	DDRD = 0x00;
	PORTD = 0xFF;*/

	// ��������� ������/������� �������
	TIMSK0 &= ~(1<<TOIE0); // �������� ���������� �� ������������ �������
	TCNT1 = 0x00;
	//TIMSK1 &= ~(1<<ICIE1) | (1<<TOIE1);		// �������� ���������� �� ������������ ������� � �� �������� �������

	// ��������� ������ (555)
	POWER555_PORT &= ~_BV(POWER555_PIN);

	// ��������� ADC
	ADCSRA &= ~(1<<ADEN) & ~(1<<ADIE); 
	//ADMUX |= (1<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0);

}

void enableAllAfterSleep(){

	// �������� �����
	// Init sensor input
	//SENSOR_DDR &= ~_BV(SENSOR);
	//SENSOR_PORT |= _BV(SENSOR);

#ifndef WIRE1_DISABLED
	// wire-1 address
	WIRE1_ADDR_DDR &= ~(_BV(WIRE1_ADDR_1) | _BV(WIRE1_ADDR_2));
	WIRE1_ADDR_PORT |= _BV(WIRE1_ADDR_1) | _BV(WIRE1_ADDR_2);

	WIRE1_POWER_DDR |= _BV(WIRE1_POWER);
//	WIRE1_POWER_PORT |= _BV(WIRE1_POWER);

	// �������� ������� 1-wire slave
	WIRE1_POWER_PORT &= ~_BV(WIRE1_POWER);
	_delay_ms(100);

	// ���� ����� ��������� �������, ���� ����
	if(wire1_sendReset())
		_delay_ms(500);
#endif


	POWER555_DDR |= _BV(POWER555_PIN);
	POWER555_PORT |= _BV(POWER555_PIN);


	LED_DDR |= _BV(HACK_LED_PIN);
	LED_PORT &= ~_BV(HACK_LED_PIN);

	// Initialize button (PD2)
	//BUTTON_DDR &= ~(_BV(BUTTON) | _BV(NRF_IRQ));
	//BUTTON_PORT |= _BV(BUTTON) | _BV(NRF_IRQ);

	// Initialize buzzer (BUZZER_PIN)
	BUZZER_DDR |= _BV(BUZZER_PIN);
	BUZZER_PORT &= ~_BV(BUZZER_PIN);

	// Initialize external power signal
	//EXTERNAL_POWER_DDR &= ~_BV(EXTERNAL_POWER);
	//EXTERNAL_POWER_PORT &= ~_BV(EXTERNAL_POWER);

#ifndef NRF_DISABLED
	SPCR |= _BV(SPE);
	if(isMirfAvailable){
		// �������� ���������� �� NRF24
		EIMSK |= _BV(INT1);

		// ��������� NRF24
		mirf_setTX;
		//_delay_ms(2);
		_delay_us(100);
	}
#endif	

	// �������� ADC
	//ADMUX |= (1<<MUX3)|(1<<MUX2)|(1<<MUX1); //Set the Band Gap voltage as the ADC input
	ADCSRA |= (1<<ADEN)|(1<<ADIE)|(1<<ADIF); 
	//ADC_init();

	// �������� ������ (555)
	POWER555_PORT |= _BV(POWER555_PIN);

	// �������� ������/������� �������
	TCNT0 = TIMER;
	TIMSK0 |= (1<<TOIE0); // �������� ���������� �� ������������ �������
	TCNT1=0x00;
	//TIMSK1 |= (1<<ICIE1) | (1<<TOIE1);		// �������� ���������� �� ������������ ������� � �� �������� �������

}

void prepareStatus(){

	sendBuf[2] = 0x01;

	// ������� �������
	sendBuf[3] = (lastFrequency>>8);
	sendBuf[4] = lastFrequency;

	// ������� �����
	sendBuf[5] = (defaultFrequency>>8);
	sendBuf[6] = defaultFrequency;

	// ������� ����� �������
	sendBuf[7] = (vcc>>8);
	sendBuf[8] = vcc;

	// ��������� ������� �������
	if(alarmCnt>ALARM_MIN_CNT)
		sendBuf[9] = 1;
	else
		sendBuf[9] = 0;

	// ���������� ����� ���������� � ������������ �������
	sendBuf[10] = treshold;

	// ����������������� ������ �������
	sendBuf[11] = (alarm_time_limit>>8);
	sendBuf[12] = alarm_time_limit;

	// ����������������� �������������
	sendBuf[13] = (alarm_ignore_time>>8);
	sendBuf[14] = alarm_ignore_time;

	sendBuf[15] = wire1_last_status;

}

void preparePowerStatus(){

	sendBuf[2] = 0x02;

	// ����������� ������� ������ �������
	sendBuf[3] = (min_bat_level>>8);
	sendBuf[4] = min_bat_level;

	// ���� ������� �������� �������
	sendBuf[5] = hasExternalPower;

	// �������� �������� �� ����� (���������� OBSERVER_TX �� ������ ��������� ���������� ��������)
	sendBuf[6] = txQuality;


}

// ��������� ��������� � ������������ � �������
void prepareResetStatus(char status){

	sendBuf[2] = 0x03;
	sendBuf[3] = status;

}

/*void prepareSendBuf(uint8_t *sendBuf, char cmd){

	// ����� ����������
	sendBuf[0] = dev_address[0];
	sendBuf[1] = dev_address[1];

	// ��� ��������
	sendBuf[2] = cmd;

}*/

void startSend(){

	if(!isMirfAvailable || sendInProgress)
		return;

	//prepareSendBuf(sendBuf, cmd);

	cli();
	txTimeoutCnt = 600;		// ����� ��������� �������� 0.1 ���
	sendInProgress = 1;
	sei();
	mirf_write(sendBuf);

}

int initRadioAddress(){

	if(addressRequestPause==0){
		sendAddressRequest();
		addressRequestPause = 1200;
	}

	return 0;
}

void ignoreAlarm(){
	alarmIgnoreCnt = alarm_ignore_time;
	alarmCnt = 0;
	alarmTimeCnt = 0;
	LED_PORT &= ~_BV(LED_PIN);
}

void startLed(int cycles, int ledCnt)
{
	LED_PORT |= _BV(LED_PIN);
	led_cycle = (cycles * 2) - 1;
	led_cnt_max = ledCnt;
	led_cnt = led_cnt_max;
}

void turnLedOff(){
	led_cycle = 0;
	led_cnt_max = 0;
	led_cnt = 0;
	LED_PORT &= ~_BV(LED_PIN);
}
