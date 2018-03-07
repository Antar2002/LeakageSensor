#include <stdio.h>
#include <avr/eeprom.h>
#include "paramsFn.h"
#include "shared.h"

EEMEM	uint8_t 	treshold_stored = TRESHOLD;
EEMEM	uint16_t 	min_bat_level_stored = MIN_BAT_LEVEL;
EEMEM	uint16_t 	alarm_ignore_time_stored = ALARM_IGNORE_TIME;
EEMEM	uint16_t 	alarm_time_limit_stored = ALARM_TIME_LIMIT;

void readParamsFromMem(){

	treshold = eeprom_read_byte(&treshold_stored);
	if(treshold<50){
		min_bat_level = eeprom_read_word(&min_bat_level_stored);
		alarm_ignore_time = eeprom_read_word(&alarm_ignore_time_stored);
		alarm_time_limit = eeprom_read_word(&alarm_time_limit_stored);;
	}
	else{
		treshold = TRESHOLD;
		min_bat_level = MIN_BAT_LEVEL;
		alarm_ignore_time = ALARM_IGNORE_TIME;
		alarm_time_limit = ALARM_TIME_LIMIT;
	}

	verifyParams();

}

void verifyParams(){
	if(treshold>50)					// Разница меджду фоновой частотой и частотой тревоги не больше 50%
		treshold = 50;
	if(min_bat_level > 450)			// Минимальный уровень батареи не ниже 4.5 В
		min_bat_level = 450;
	if(alarm_ignore_time > 900)		// Отключение тревоги не больше чем на 60 минут
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
