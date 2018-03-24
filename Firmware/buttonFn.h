#include "shared.h"

#define		BTN_NOIS_REDUCE	45			// ~0.05 ���
#define		BTN_SHORT_PRESS_TIME (TIMER_OVF_SEC * 3)	// ����������������� "������� ������� �� ������"

volatile int btnNoiseTimer;
volatile char btnPressed;
volatile int btnPressTimer;

volatile char btnProcessed; // ����/������� ����, ��� ������ ��� ����������

volatile char prevMode;

void button_init();
void button_handleButton();
void button_timerProc();
