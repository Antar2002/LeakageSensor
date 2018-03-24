#include "buttonFn.h"
//#include "shared.h"

void button_init()
{
	btnPressed = 0;
	btnPressTimer = 0;

	btnProcessed = 1;
}

void button_handleButton()
{

	if((~BUTTON_PIN & _BV(BUTTON)) && !btnPressTimer){
		btnProcessed = 0;
		btnPressTimer = 1;
	}

	if(btnPressTimer > BTN_NOIS_REDUCE + 1){
		btnPressed = 1;

		// ����� ������ ���������
		if(BUTTON_PIN & _BV(BUTTON)){
			// ���� ��� ��� �� ���� ���������� �� �����
			if(!btnProcessed){
				if(btnPressTimer < BTN_SHORT_PRESS_TIME){
					mode = 1;
				}
				else{
					mode = 3;
				}
				modeChanged = 1;
				btnProcessed = 1;
			}
			// �������� �������, ���� ������ ��� ��������
			btnPressTimer = 0;
			btnPressed = 0;
		}
	}
}

void button_timerProc()
{
	if(btnPressTimer > 0 && btnPressTimer < BTN_SHORT_PRESS_TIME + 1){
		btnPressTimer++;
	}
}

