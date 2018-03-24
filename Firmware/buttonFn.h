#include "shared.h"

#define		BTN_NOIS_REDUCE	45			// ~0.05 сек
#define		BTN_SHORT_PRESS_TIME (TIMER_OVF_SEC * 3)	// Продолжительность "долгого нажатия на кнопку"

volatile int btnNoiseTimer;
volatile char btnPressed;
volatile int btnPressTimer;

volatile char btnProcessed; // Флаг/признак того, что кнопку уже обработали

volatile char prevMode;

void button_init();
void button_handleButton();
void button_timerProc();
