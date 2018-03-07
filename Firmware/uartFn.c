#include <stdio.h>
#include <stdlib.h>

void uart_printCurFrequency(uint16_t lastFrequency, uint16_t defaultFrequency, int vcc){

#ifdef UART_ENABLED
		char res[5];

		//uart_putc('.');
		utoa(lastFrequency, res, 10);
		uart_puts(res);
		uart_puts(" (");
		utoa(defaultFrequency, res, 10);
		uart_puts(res);
		/*if(alarmCnt<ALARM_MIN_CNT){
			uart_puts(") Vcc: ");
			utoa(vcc, res, 10);
			uart_puts(res);
		}
		else
			uart_putc(')');*/
		uart_puts(") Vcc: ");
		utoa(vcc, res, 10);
		uart_puts(res);
		//uart_putc(' ');
		//uart_putc(adc_data);
		uart_putc('\r');
#endif

}

void uart_printDefaultFrequency(uint16_t defaultFrequency){
	char res[5];
	uart_puts("DefaultFrequency: ");
	utoa(defaultFrequency, res, 10);
	uart_puts(res);
	uart_putc('\r');
}

void uart_printCurVcc(){
}

void uart_printStr(const char *s){
	uart_puts(*s);
}
