#include <stdio.h>
#include "addressFn.h"
#include "nrf24/mirf.h"
#include "shared.h"

char dev_address[2];
EEMEM	uint8_t		dev_address_stored[2] = {0,0};

void initAddress()
{
	//dev_address[0] = 0x00;
	//dev_address[1] = 0x10;
	eeprom_read_block(dev_address, dev_address_stored, 2);
}

void sendAddressRequest()
{
	// Сформировать буфер передачи
	uint8_t sendBuf[buffersize];
	// Адрес устройства
	sendBuf[0] = dev_address[0];
	sendBuf[1] = dev_address[1];
	sendBuf[2] = 0;	// Команда на получение адреса
	sendBuf[3] = 3;	// Тип устройства
	sendBuf[4] = 0; // Тип устроства
	sendBuf[5] = 0; // Тип устроства
	sendBuf[6] = 0; // Тип устроства
	sendBuf[7] = 0; // Тип устроства
	sendBuf[8] = 0; // Тип устроства
	sendBuf[9] = 0; // Тип устроства
	sendBuf[10] = 0; // Тип устроства


	// Отправить данные
	mirf_write(sendBuf);
//hackLedCnt = 650;
}

// Назначение нового адреса устройству
void assignAddress(uint8_t *buffer)
{
	dev_address[0] = buffer[3];
	dev_address[1] = buffer[4];
	eeprom_update_block(dev_address, dev_address_stored, 2);
	//noAddress = 0;
}

// Принимать команды для этого устройства или если адрес 0 и уст-во находится в режиме получения адреса
char checkAddress(uint8_t *buffer)
{
	//int sum = 0;
	for(char i=0;i<2;i++)
	{
		//sum += dev_address[i];
		if((mode == 3 && buffer[i] != dev_address[i]) 
			//|| (mode == 1 && buffer[i] != 0)
			)
			return 0;
	}

	// Если адрес устройства не задан и оно не находится в режиме получения адреса
	/*if(mode == 0 && sum == 0)
		return 0;*/

	return 1;
}
