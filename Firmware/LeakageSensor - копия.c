#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <util/delay.h>
//#include <util/delay.h>
#include "uart.h"
#include "LeakageSensor.h"

#define UART_BAUD_RATE 56000
#define UART_RX_BUFFER_SIZE 40
#define UART_TX_BUFFER_SIZE 40
char uartData[18];
char res[10];


ISR(TIMER1_CAPT_vect)   
{
    /*curFrequency = ICR1+2;
	TIMSK &= ~(1<<TICIE1);		// Выключить это прерывание
	freqImpulsFlag = 1;
    //TCNT1 = 0;*/

	/*uint16_t curTime = ICR1;
	if(freqCnt>0){
		curFrequency = curTime - prevTime + ovfCnt;
		ovfCnt = 0;
		if(tmpFrequency==0)
			tmpFrequency = curFrequency;
		else
			tmpFrequency = (tmpFrequency + curFrequency)/2;
		prevTime = curTime;
		freqCnt--;
	}
	else{
		TIMSK &= ~(1<<TICIE1) & ~(1<<TOIE1);		// Выключить это прерывание
		//freqImpulsFlag = 1;
		mesMode = 2;
	}
*/
TCNT1 = 0;
lastFrequency = ICR1;
}

ISR(TIMER1_OVF_vect){
	ovfCnt = (65535 - prevTime);
	prevTime = 0;
	//LED_PORT ^= _BV(LED_PIN);
}

ISR (TIMER0_OVF_vect)
{

	/*sndCnt++;
	if(sndCnt>5){
		BUZZER_PORT ^= _BV(BUZZER_PIN);
		sndCnt = 0;
	}*/

	TCNT0 = TIMER;
}

ISR (INT0_vect)
{
}

void initCounting(){
	prevTime = 0;
	freqCnt = DEFAULT_MES_COUNT;
	mesMode = 1;
	TIMSK |= (1<<TICIE1) | (1<<TOIE1);	// Включить прерывание таймера по событию и переполнению
	TCNT1 = 0x00;
	ICR1 = 0x00;
}

char measureDefaultFrequency(){
	if(mesMode==0){
		initCounting();
	}

	/*if(freqCnt > 0){
		if(freqImpulsFlag){
			if(defaultFrequency==0)
				defaultFrequency = curFrequency;
			else
				defaultFrequency = (defaultFrequency + curFrequency) / 2;
			freqImpulsFlag = 0;
			TCNT1 = 0;
			TIMSK |= (1<<TICIE1);	// Включить прерывание таймера по событию
		}
		freqCnt--;
	}*/
	if(mesMode==2){
		defaultFrequency = tmpFrequency;
	}
	return freqCnt > 0;
}

char measureFrequency(){
	if(mesMode==0){
		initCounting();
	}

	/*if(freqCnt > 0){
		if(freqImpulsFlag){
			if(tmpFrequency==0)
				tmpFrequency = curFrequency;
			else
				tmpFrequency = (tmpFrequency + curFrequency) / 2;
			freqImpulsFlag = 0;
			TCNT1 = 0;
			TIMSK |= (1<<TICIE1);	// Включить прерывание таймера по событию
		}
		freqCnt--;
		if(freqCnt==0)
			lastFrequency = tmpFrequency;
	}*/
	if(mesMode==2){
		lastFrequency = tmpFrequency;
	}
	return freqCnt > 0;
}

void main()
{

	// Init sensor input
	SENSOR_DDR &= ~_BV(SENSOR);
	SENSOR_PORT |= _BV(SENSOR);


	// Initialize LED (PC5)
	LED_DDR |= _BV(LED_PIN);
	LED_PORT &= ~_BV(LED_PIN);

	// Initialize button (PD2)
	DDRD &= ~_BV(PD2);
	PORTD |= _BV(PD2);

	// Initialize buzzer (BUZZER_PIN)
	BUZZER_DDR |= _BV(BUZZER_PIN);
	BUZZER_PORT &= ~_BV(BUZZER_PIN);

	cli();

	// Init uart
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );

	// Init INT0
	//MCUCR &= ~_BV(ISC00) & ~_BV(ISC01);
	//MCUCR |= _BV(ISC00) | _BV(ISC01);

	//MCUCR &= _BV(ISC00) & _BV(ISC01);	// Low level rise interrupt
	//GIMSK |= _BV(INT0);

	//Init Timer 0
	TCCR0 &= 0x00;
	//TCCR0 |= (1<<CS00);	// (8 MHz / 256 = 31250)
	//TCCR0 |= (1<<CS01);	// /8 (1 MHz / 256 = 3906,25)
	TCCR0 |= (1<<CS01) | (1<<CS00);	// /64 (125 kHz / 256 = 488,28125)
	//TCCR0B |= (1<<CS02);	// /256
	//TCCR0B |= (1<<CS02) | (1<<CS00);	// /1024 (9375 Hz)
	TCNT0 = TIMER;
	TIMSK = 0x00;
	TIMSK |= (1<<TICIE1) | (1<<TOIE1) | (1<<TOIE0);		// Включить прерывание по переполнению таймера
	//TIMSK |= (1<<TOIE0);


	// Init timer 1 (for sensor frequency measuring)
	TCCR1A = 0x00;
	TCCR1B = 0x00;
	TCCR1B = (1<<ICNC1) | (1<<ICES1) | (1<<CS10);
	//TCNT1=0x00;
	//ICR1=0x00;

	sei();

	//calcCap = 0;
	mode = 3;

	uart_puts("Leakage Sensor v0.1\r\r");
	LED_PORT |= _BV(LED_PIN);
	_delay_ms(1000);
	LED_PORT &= ~_BV(LED_PIN);
	_delay_ms(500);

	return;

	while(1)
	{

/*
// Test UART output
itoa(12345, res, 10);
uart_puts(res);
uart_putc('\r');
_delay_ms(100);*/

		switch(mode){
			case 1:
				//LED_PORT |= _BV(LED_PIN);
				mesMode = 0;
				if(!measureDefaultFrequency()){
					// Switch LED off
					//LED_PORT &= ~_BV(LED_PIN);
					mode = 2;
				}
				/* // Test: Check sensor input port
				if(SENSOR_PIN & _BV(SENSOR))
					LED_PORT |= _BV(LED_PIN);
				else
					LED_PORT &= ~_BV(LED_PIN);
				break;*/
			case 2:
				mesMode = 0;
				if(!measureFrequency()){
					//LED_PORT ^= _BV(LED_PIN);
					// Print last measured frequency to UART
					uart_puts("freq: ");
					utoa(lastFrequency, res, 10);
					uart_puts(res);
					uart_putc('\r');
					/*if(lastFrequency < (defaultFrequency * TRESHOLD))
						LED_PORT |= _BV(LED_PIN);
					else
						LED_PORT &= ~_BV(LED_PIN);*/
				}
				_delay_ms(100);
				break;
			default:
				uart_puts("lastFrequency: ");
				utoa(lastFrequency, res, 10);
				uart_puts(res);
				uart_putc('\r');
				_delay_ms(100);
				break;
		}
	}
}
