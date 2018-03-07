
#ifdef UART_ENABLED
#define URAT_PRINT_STR(str) 		uart_printStr(str)
#define UART_PRINT_DEFAULT_FREQ(defFreq)	uart_printDefaultFrequency(defFreq)
#else
#define URAT_PRINT_STR(str)
#define UART_PRINT_DEFAULT_FREQ(defFreq)
#endif

void uart_printCurFrequency(
	uint16_t lastFrequency,
	uint16_t defaultFrequency,
	int vcc
	);

void uart_printDefaultFrequency(uint16_t defaultFrequency);

void uart_printStr(const char *s);
