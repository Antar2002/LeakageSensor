#include "wire1.h"

void wire1_init(){

	WIRE1_PORT &= ~_BV(WIRE1_PIN);
	WIRE1_PORT_TO_RECEIVE;

	// Init Timer2
	TCCR2A = 0;
	TCCR2B = 0;
	TCCR2B |= _BV(CS21) | _BV(CS20);	// / delim 32 = 1250 kHz = 3.2 uS
	TCNT2 = 254;	// ������������� ������������ = 6.4 uS
	TIMSK2 = _BV(TOIE2);


	wire1status = 0;
	wire1status |= (1<<0);	// ���������� ������� ������� ������� �� ��������� � 1
}

ISR(TIMER2_OVF_vect){
	// ���� �� � ������ ��������
	if(WIRE1_IS_SEND_MODE){
		if(wire1SigCnt>0)
			wire1SigCnt--;
		// ���� ���������� ������ - ��������� �������� � ��������� �� �����
		else{
			WIRE1_PORT_TO_RECEIVE;
			wire1PauseCnt = 0;
			wire1_nextStepAfterTime();
		}
	}
	else{
		// ���������, �� ��������� �� ������� (����, "����������")

		// ���� ������� ����� "0" � ���������� ������� "1"
		// ������ ���-�� ������� �����
		if(~WIRE1_PIN | _BV(WIRE1) | wire1status){
			// ��������� ������� ������� �����
			wire1status &= ~_BV(WIRE1);
			wire1SigCnt = 1;
		}
		else{
			// ���� ����� ���� ������� � ��������� � "1"
			if(WIRE1_PIN | _BV(WIRE1) | ~wire1status){
				wire1status |= _BV(WIRE1);
				wire1PauseCnt = 0;
				wire1_nextStepAfterTime();
			}
			else
				// ����� ������ ������� ����������������� ��������
				wire1SigCnt++;
		}
		wire1PauseCnt++;
	}
}

void wire1_nextStepAfterTime(){

	switch(mode){
		case 0:
			if(wire1SigCnt>=150){
				mode = 1;
				// ���-�� �������� ������� "�����" - �� ������ �����
				wire1status &= ~(1<<7);
				wire1_nextStepAfterTime();
			}
			break;
		case 1:
			wire1_sendPresence();
			mode = 0;
			break;
		case 6:		// ��������� ������� �������, ��� �������			
			break;
		case 8:		// ���������, ���� �� ��������
			break;
	}
}

void wire1_sendReset(){
	// ��������� ������ "�����" �� 480 ���
	WIRE1_PORT_TO_SEND;
	wire1SigCnt = 150;
}

void wire1_sendPresence(){
	// ��������� ������ "�����������" �� 60 ���
	WIRE1_PORT_TO_SEND;
	wire1SigCnt = 20;
}

// "public" ����� "���������� ������� ������� ��� �������"
void wire1_setDefaultFreq(){
	wire1_sendByteMaster(0b01100101);
}

char wire1_checkAlarm(){
	wire1_sendByteMaster(0x00);
}


void wire1_sendByteMaster(char data){

	
}

void wire1_sendBit0Master(){
	WIRE1_PORT_TO_SEND;
	wire1SigCnt = 20;
}

void wire1_sendBit1Master(){
	WIRE1_PORT_TO_SEND;
	wire1SigCnt = 4;
}
