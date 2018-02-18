//#define UART_ENABLED;
#define WIRE1_DISABLED

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
#include "nrf24/mirf.h"
#include "nrf24/nRF24L01.h"
#include "addressFn.h"
#ifndef WIRE1_DISABLED
#include "wire1/wire1_lite.h"
#endif
#include "shared.h"


EEMEM	uint8_t 	treshold_stored = TRESHOLD;
EEMEM	uint16_t 	min_bat_level_stored = MIN_BAT_LEVEL;
EEMEM	uint16_t 	alarm_ignore_time_stored = ALARM_IGNORE_TIME;
EEMEM	uint16_t 	alarm_time_limit_stored = ALARM_TIME_LIMIT;


#define UART_BAUD_RATE 56000
#define UART_RX_BUFFER_SIZE 40
#define UART_TX_BUFFER_SIZE 40
char uartData[18];
char res[18];


/*ISR(TIMER1_CAPT_vect)   
{
	freqCnt++;
}*/

/*ISR(TIMER1_OVF_vect){
	ovfCnt++;
}*/

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

	// ���������� �������� ������
	if(btnNoiseTimer>0){
		btnNoiseTimer--;
		if(btnNoiseTimer==0){
			btnPressed = 1;
			btnPressTimer = 0;
		}
	}

	// ��������� ������� ������ - ������� ����������������� �������
	if(btnPressed==1){
		if(btnPressTimer < BTN_SHORT_PRESS_TIME + 2)
			btnPressTimer++;
	}

	// ������� ������� ������������� ��������� � Watchdog
	/*timer0OvfSecCnt++;
	if(timer0OvfSecCnt>TIMER_OVF_SEC){
		// ���� ������ ���� ������ ������������� ������� - ��������� �������� ����� ������������� �� 1
		if(alarmIgnoreCnt>0)
			alarmIgnoreCnt--;
		timer0OvfSecCnt = 0;
	}*/


	// ���� ������ ��������� ������� ��� ������ ������� ������ �������	
	if((alarmCnt>ALARM_MIN_CNT && alarmIgnoreCnt==0) || sndCnt>0){
		sndFreqCnt++;
		if(sndFreqCnt>4){
			BUZZER_PORT ^= _BV(BUZZER_PIN);
			sndFreqCnt = 0;
		}
		if(sndCnt>0)
			sndCnt--;
	}


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
		if(led_cnt < 60000)
			led_cnt--;
	}

	if(hackLedCnt>0){
		HACK_LED_PORT |= _BV(HACK_LED_PIN);
		hackLedCnt--;
	}
	else{
		HACK_LED_PORT &= ~_BV(HACK_LED_PIN);
	}

	if(addressRequestPause>0)
		addressRequestPause--;

	TCNT0 = TIMER;
}

// ��������� ������
ISR (INT0_vect)
{
	// ������ ������
	if(~BUTTON_PIN & _BV(BUTTON)){
		if(btnNoiseTimer==0 && !btnPressed){
			btnNoiseTimer = BTN_NOIS_REDUCE;
		}
	}
	// ������ ���������
	else{
		// ���� ������ ������� ������� � ���� �������� - ���������� ����� �������
		if(btnPressed && !pressedForResetAlarm){
			// ���� ������ ���� ������ �������
			if(btnPressTimer<BTN_SHORT_PRESS_TIME){
				modeChanged = 1;
				mode = 1;
			}
			else{
				// ���� ������ ������ ������� ����� - ������� � ����� ��������� ������
				if(mode!=3){
					modeChanged = 1;
					mode = 3;
				}
			}
		}
		btnNoiseTimer = 0;
		btnPressed = 0;
		pressedForResetAlarm = 0;
	}
}

ISR (WDT_vect)
{

	WDTCSR |= _BV(WDIE);

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

	sleepsCnt++;
	// ������ ������ ��������
	if(sleepsCnt>1){
		// ���� ������ �� ������ ���������� �������, ���������� ���������� �� �������
		if(alarmCnt==0){
			sleepsCnt = 0;
			ADCSRA |= (1<<ADSC);
		}
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

ISR(INT1_vect)	// Interupt from NRF
{

	uint8_t status = mirf_get_status();
#ifdef UART_ENABLED
test = status;
#endif
//hackLedCnt = 200;
	if(status & (1<<RX_DR)){
hackLedCnt = 1000;
		//mirf_write_register(STATUS, (1<<RX_DR));
		//mirf_flush_rx();
		if(dataReceived == 0){
			mirf_read(buffer);
			dataReceived = 1;
		}
		//mirf_write_register(STATUS, (1<<RX_DR));
		//mirf_reset();
		mirf_write_register(STATUS, mirf_read_register(STATUS) | (1<<RX_DR));
//		mirf_flush_rx();
//		sendStatus();
	}
	else if((status & (1<<TX_DS)) || (status & (1<<MAX_RT))){
		afterWriteData(status);
		sendInProgress = 0;
	}
	mirf_flush_rx();
	mirf_reset();
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
#endif
}

void timersInit()
{
	//Init Timer 0
	// Mega8 TCCR0
	TCCR0B &= 0x00;
	//TCCR0 |= (1<<CS00);	// (8 MHz / 256 = 31250)
	//TCCR0 |= (1<<CS01);	// /8 (1 MHz / 256 = 3906,25)
	//TCCR0 |= (1<<CS01) | (1<<CS00);	// /64 (156.250 kHz / 256 = 610,3515625)
	// Mega8 TCCR0 |= (1<<CS02);		// /256 (39,0625 kHz / 256 = 152,588)
	TCCR0B |= (1<<CS02);		// /256 (39,0625 kHz / 256 = 152,588)
	//TCCR0B |= (1<<CS02) | (1<<CS00);	// /1024 (9375 Hz)
	TCNT0 = TIMER;
	TIMSK0 = 0x00;
	TIMSK0 = (1<<TOIE0); // �������� ���������� �� ������������ �������
	// Mega8 TIMSK |= (1<<TICIE1) | (1<<TOIE1) | (1<<TOIE0);		// �������� ���������� �� ������������ ������� � �� �������� �������
	//TIMSK |= (1<<TOIE0);


	// Init timer 1 (for sensor frequency measuring)
	TCCR1A = 0x00;
	TCCR1B = 0x00;
	//TCCR1B = (1<<ICNC1) | (1<<ICES1) | (1<<CS10);		// For capture signal using Input Capture Unit of Timemr1
	TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10);
	TCNT1=0x00;
	//ICR1=0x00;
	TIMSK1 = 0x00;
	TIMSK1 |= (1<<ICIE1) | (1<<TOIE1);		// �������� ���������� �� ������������ ������� � �� �������� �������
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

	portsInit();

	cli();

#ifdef UART_ENABLED
	// Init uart
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
#endif

	// Init INT0 & INT1
	//MCUCR &= ~_BV(ISC00) & ~_BV(ISC01);
	//MCUCR |= _BV(ISC00) | _BV(ISC01);
	//MCUCR &= _BV(ISC00) & _BV(ISC01);	// Low level rise interrupt
	//GIMSK |= _BV(INT0);
	EICRA = 0x00;
	EICRA |= _BV(ISC00) | _BV(ISC11);	// Any level of INT0 change raise interrupt, falling edge on INT1 raise interupt
	EIMSK = 0x00;
	EIMSK |= _BV(INT0) | _BV(INT1);


	timersInit();

	ADC_init();

	WDT_Init();

	mirf_init();
	_delay_ms(50);

#ifndef WIRE1_DISABLED
	wire1_init();
	wire1_addr_init();
#endif

	sei();

//	hackLedCnt = 900;

	mirf_config();
	mirf_set_rxaddr(0, rxaddr);
	mirf_set_txaddr(txaddr);

// ���������������� ����� ���������� � �����������
	initAddress();

	// ��������� ��������� �� ������
	readParamsFromMem();

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	// ���������� ���, ��� �� ����������
#ifdef UART_ENABLED
	uart_puts("Leakage Sensor v0.1\r\r");
#endif
	startLed(1, 450);
/**	LED_PORT |= _BV(LED_PIN);
	_delay_ms(500);
	LED_PORT &= ~_BV(LED_PIN);
	_delay_ms(500);*/

	
	// ���������� ��������������� ��� ����������
	mode = 1;
/*	freqImpulsFlag = 0;
	curFrequency = 0;*/
	sendInProgress = 0;
	dataReceived = 0;

	modeChanged = 1;

	while(1)
	{
#ifdef UART_ENABLED
if(test!=0){
uart_putc(test);
uart_putc('\r');
test = 0;
}
#endif
		/*// ���������� ������� ������
		if(~BUTTON_PIN & _BV(BUTTON)){
			if(btnNoiseTimer==0 && !btnPressed){
				btnNoiseTimer = BTN_NOIS_REDUCE;
			}
			// ���� ������ ������ ������� ����� - ������� � ����� ��������� ������
			if(btnPressTimer > BTN_SHORT_PRESS_TIME){
				initRadioAddress();
			}
		}
		else{
			// ���� ������ ������� ������� � ���� �������� - ���������� ����� �������
			if(btnPressed && !pressedForResetAlarm){
				// ���� ������ ���� ������ �������
				if(btnPressTimer<BTN_SHORT_PRESS_TIME){
					LED_PORT |= _BV(LED_PIN);
					uart_puts("Default level reinitializing.\r");
					mode = 1;
					_delay_ms(1000);
				}
			}
			btnNoiseTimer = 0;
			btnPressed = 0;
			pressedForResetAlarm = 0;
		}*/

		// ��������� ������� ������
		if(btnPressed){
			// ���� ���� ������� - ��������� � ������ ������������
			if(alarmCnt>ALARM_MIN_CNT){
				ignoreAlarm();
				pressedForResetAlarm = 1;
			}
			else{
				// ���� ������ ������ ������� ����� - ������� � ����� ��������� ������, �������� ��������� � ������ �������������
				if(btnPressTimer > BTN_SHORT_PRESS_TIME && mode!=3){

					treshold = TRESHOLD;
					min_bat_level = MIN_BAT_LEVEL;
					alarm_ignore_time = ALARM_IGNORE_TIME;
					alarm_time_limit = ALARM_TIME_LIMIT;

					saveParams();

					alarmIgnoreCnt = 0;
						
					modeChanged = 1;
					mode = 3;
				}
			}
		}

		// ������ ������
		if(dataReceived == 1)
		{
//hackLedCnt = 4000;
			//mirf_read(buffer);
			// ���������, ��� ��� ������� ��� ������� ����������
			if(checkAddress(buffer))
			{
				// Assign address
				if(mode==3)
				{
					if(buffer[2] == 0){		// ��������� ������ ���-��
						assignAddress(buffer);
						mode = 2;
					}
				}
				else{
					// Process command
					switch(buffer[2]){
						case 1:				// ������ ��������� ������� ������
							if(alarmCnt>ALARM_MIN_CNT){
								ignoreAlarm();
							}
							else{
								modeChanged = 1;
								mode = 1;
							}
							break;
						case 2:				// ��������������� ����������
							setParams(buffer);
							break;
					}
				}
			}
			dataReceived = 0;
		}

		if(adcUpdated){
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


		// �������� ������ ������
		switch(mode){
			case 1:		// ����� ������� �������
				if(modeChanged){
#ifdef UART_ENABLED
					uart_puts("Default level reinitializing.\r");
#endif
					//LED_PORT |= _BV(LED_PIN);
					startLed(ENDLESS_LED_CYCLE, ENDLESS_LED_PULSE);
					_delay_ms(1500);
					modeChanged = 0;
				}
				if(!measureFrequency()){
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
#ifdef UART_ENABLED
					uart_puts("DefaultFrequency: ");
					utoa(defaultFrequency, res, 10);
					uart_puts(res);
					uart_putc('\r');
#endif
#ifndef WIRE1_DISABLED
					// ����� ����������� � 1-wire, ����� ��� ���� ��������� ������� �������, ��� ������� � ��������� ����� ������������
					wire1_slaves = wire1_setDefault();
#endif

					// ������� � ����� �����������
					mode = 2;
					modeChanged = 1;
				}
				break;
			case 2:		// �������� ��������
				if(modeChanged)
					modeChanged = 0;
				if(!measureFrequency()){
#ifndef WIRE1_DISABLED
					// ��������� ����������� 1-wire
					wire1_last_status = wire1_checkAlarmStatus();
#endif
					// ���� ����� ��� � ����������� �����
					if((lastFrequency < defaultFrequency || wire1_last_status!=0) && alarmIgnoreCnt==0){
						canSleep = 0;
						if(alarmCnt<ALARM_MIN_CNT*2){
							alarmCnt++;
							//LED_PORT |= _BV(LED_PIN);
						}
						// ���� ���-�� ���������� ������� ������� ��������� ����� - ������� �������
						if(alarmCnt>ALARM_MIN_CNT){
							// ������ ������� ����� ������ ������� �������, ���� ��� �� ������ � ���� ��� �������� �������
							if(alarmTimeCnt==0 && !hasExternalPower)
								alarmTimeCnt = 1;
							// ������ ������ �����������, ���� ��� �� ������
							if(led_cnt_max!=LED_ALARM_PULSE)
								startLed(ENDLESS_LED_CYCLE, LED_ALARM_PULSE);
#ifdef UART_ENABLED
							uart_puts("!!! ALARM !!!");
							uart_puts(")\r");
#endif
						}
					}
					else{
						if(alarmCnt!=0)
							alarmCnt--;
						else{
							// ��������� ���������
//							LED_PORT &= ~_BV(LED_PIN);
							led_cycle = 0;
							// ���� ��� �������� (������� ��� ��� ��� �����������) ����� � �������
							if(alarmCnt==0)
								canSleep = 1;
						}
					}

#ifdef UART_ENABLED
					//uart_putc('.');
					// TEST ����� ������� �������
					utoa(lastFrequency, res, 10);
					uart_puts(res);
					uart_puts(" (");
					utoa(defaultFrequency, res, 10);
					uart_puts(res);
					if(alarmCnt<ALARM_MIN_CNT){
						uart_puts(") Vcc: ");
						utoa(vcc, res, 10);
						uart_puts(res);
					}
					else
						uart_putc(')');
					//uart_putc(' ');
					//uart_putc(adc_data);
					uart_putc('\r');
#endif

					// �������� �� �����
					canSendStatus = 1;
					canSendPower = 1;
				}
				if(canSendStatus==1 && sendInProgress==0){
					sendStatus();
					canSendStatus = 0;
				}
				if(canSendPower==1 && sendInProgress==0){
					sendPowerStatus();
					canSendPower = 0;
				}
				break;
			// ������ ������ � ���������
			case 3:
				if(modeChanged){
#ifdef UART_ENABLED
					uart_puts("Wait for address...\r");
#endif
					dev_address[0] = rand();
					dev_address[1] = rand();
					modeChanged = 0;
					startLed(ENDLESS_LED_CYCLE, LED_ADDRESS_REQ_PULSE);
				}
				if(initRadioAddress()!=0){
					led_cycle = 0;
					mode = 2;
				}
				break;
			case 4:
				// ������ ����� ��� ������������ NRF24
				break;
			default:
				break;
		}

		if(canSleep){
			hasExternalPower = EXTERNAL_POWER_PIN & _BV(EXTERNAL_POWER);
			// ����� ������ ���� ��� �������� ������� � ��� ��������
			if(hasExternalPower==0 && canSendStatus==0 && canSendPower==0){
				// ��������� NRF24
				//mirf_write_register(CONFIG, mirf_read_register(CONFIG) & ~(1<<PWR_UP));

				cli();
				// ��������� ���������� �� NRF24
				EIMSK &= ~_BV(INT1);
				// ��������� ���������� � �������
				LED_PORT &= ~_BV(LED_PIN);
				HACK_LED_PORT &= ~_BV(HACK_LED_PIN);
				BUZZER_PORT &= ~_BV(BUZZER_PIN);

				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
				// �������, ��� �� ��� ����
				//LED_PORT |= _BV(LED_PIN);
				//_delay_ms(50);
				//LED_PORT &= ~_BV(LED_PIN);
				startLed(1, 600);
				canSleep = 0;
				EIMSK |= _BV(INT1);
				// ��������� NRF24
				//mirf_write_register(CONFIG, mirf_read_register(CONFIG) | (1<<PWR_UP));
				_delay_ms(2);
			}
			else{
//hackLedCnt = 900;
// ����������, ����� ������������ 1-wire ��� ������, � �������� � ���� - ����
				_delay_ms(100);
			}
		}

/*
#ifdef UART_ENABLED
// TEST ����� ������� �������
		cli();
		tmpFrequency = curFrequency;
		sei();
		if(tmpFrequency>0){
			//LED_PORT ^= _BV(LED_PIN);
			uart_puts("tmpFrequency: ");
			utoa(tmpFrequency, res, 10);
			uart_puts(res);
			uart_puts(" - ovfCnt: ");
			utoa(ovfCnt, res, 10);
			uart_puts(res);			
			uart_putc('\r');
			_delay_ms(100);
		}
#endif
*/
	}
}

void prepareSendBuf(uint8_t *sendBuf, char cmd){

	// ����� ����������
	sendBuf[0] = dev_address[0];
	sendBuf[1] = dev_address[1];

	// ��� ��������
	sendBuf[2] = cmd;

}

void prepareStatus(uint8_t *sendBuf){

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
	sendBuf[16] = 0;
	
}

void sendStatus(){

	if(sendInProgress == 1)
		return;

	sendInProgress = 1;

	uint8_t sendBuf[mirf_PAYLOAD];

	prepareSendBuf(sendBuf, 0x01);
	prepareStatus(sendBuf);

	mirf_write(sendBuf);

}

void sendPowerStatus(){

	if(sendInProgress == 1)
		return;
	//while(sendInProgress == 1){
	//	_delay_ms(10);
	//}

	sendInProgress = 1;

	uint8_t sendBuf[mirf_PAYLOAD];

	prepareSendBuf(sendBuf, 0x02);

	// ����������� ������� ������ �������
	sendBuf[3] = (min_bat_level>>8);
	sendBuf[4] = min_bat_level;

	// ���� ������� �������� �������
	sendBuf[5] = hasExternalPower;

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

void readParamsFromMem(){

	treshold = eeprom_read_byte(&treshold_stored);
	min_bat_level = eeprom_read_word(&min_bat_level_stored);
	alarm_ignore_time = eeprom_read_word(&alarm_ignore_time_stored);
	alarm_time_limit = eeprom_read_word(&alarm_time_limit_stored);;

	verifyParams();

}

void verifyParams(){
	if(treshold>50)					// ������� ������ ������� �������� � �������� ������� �� ������ 50%
		treshold = 50;
	if(min_bat_level > 450)			// ����������� ������� ������� �� ���� 4.5 �
		min_bat_level = 450;
	if(alarm_ignore_time > 900)		// ���������� ������� �� ������ ��� �� 60 �����
		alarm_ignore_time = 900;
}

void setParams(uint8_t *data){

	treshold = data[3];

	min_bat_level = data[4];
	min_bat_level <<=8;
	min_bat_level |= data[5];

	alarm_ignore_time = data[6];
	alarm_ignore_time <<=8;
	alarm_ignore_time |= data[7];

	alarm_time_limit = data[8];
	alarm_time_limit <<=8;
	alarm_time_limit |= data[9];

	verifyParams();
	saveParams();

}

void saveParams(){
	eeprom_write_byte(&treshold_stored, treshold);
	eeprom_write_word (&min_bat_level_stored, min_bat_level);
	eeprom_write_word (&alarm_ignore_time_stored, alarm_ignore_time);
	eeprom_write_word (&alarm_time_limit_stored, alarm_time_limit);
}
