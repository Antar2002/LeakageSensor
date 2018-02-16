#include <avr/io.h>

#define WIRE1_DDR		DDRB
#define WIRE1_PORT		PORTB
#define WIRE1_PIN		PINB
#define WIRE1			PB0

#define WIRE1_PORT_TO_RECEIVE	WIRE1_DDR &= ~_BV(WIRE1_PIN); wire1status &= ~(1<<3)
#define WIRE1_PORT_TO_SEND		WIRE1_DDR |= _BV(WIRE1_PIN); wire1status |= (1<<3)
#define WIRE1_IS_SEND_MODE		wire1status | (1<<3)

char wire1address[3];

// (1<<WIRE1) - ���������� �������
// ��������� ���� ������������� � ����� ����
// (1<<3) - ����� ��� �������� (0 - �����, 1 - ��������)
// (1<<7) - ����� ������ (0 - �����, 1 - ������)
char wire1status;

// ������� �����
// 0 - ������ �� ������, ������� �����
// 1 - ���������� ������ "�����������" � ���� �������
// 2 - ���������� Search ROM ($F0)
// 3 - �������� �� Search ROM ����� �������
// 4 - ���������� Match ROM ($55)
// 5 - ��������� ������� �� ������� (����� ������� "�����������")
// 6 - ����������� ������� "�������� �������"
// 7 - �������� ������� (2 ����� + CRC)
// 8 - ���������, ���� �� ��������
// 9 - �������� �� ������ � ��������
char mode;

char wire1SigCnt;		// ������� ����������������� �������� ������� � ������������� ������� (* 3.2 ���)
char wire1PauseCnt;		// ������� ����������� ����� ��������� � ������������� ������� (* 3.2 ���)

void wire1_init();
void wire1_sendResetAndScan();

