void readParamsFromMem();
void setParams(uint8_t *data);
void verifyParams();
void saveParams();

#define		TRESHOLD		5			// ������� ��������� ������� �������, ������������ �������
#define		MIN_BAT_LEVEL		242		// ����������� ���������� ������� (2.42�), ����� �������� ������������ ������ � ������ ������ �������
#define		ALARM_IGNORE_TIME	2		// ����� ������������� ������� ����� �� ������ �������� (� ��� * 4)
#define		ALARM_TIME_LIMIT	2		// ����������� ����������������� ������ ������� ������� (� ��� * 4)

char treshold;				// ������� ����� ������ � ������� �������, ��� ������� ����� �������
int min_bat_level;			// ����������� ������� �������, ����� �������� ����� ������ � ������������� ������
int alarm_ignore_time; 		// �����, �� ������� ����������� ������� ��� ������� ������� � ������� (� ��� * 4)
int alarm_time_limit;		// ����������������� ������� ������� (� ��� * 4). ���� ������� ������ ��� ����� - ��� ����� ��������� �� alarm_ignore_time
