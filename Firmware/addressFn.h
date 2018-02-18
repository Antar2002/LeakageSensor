#include <avr/eeprom.h>

void initAddress();
void sendAddressRequest();
void assignAddress(uint8_t *buffer);
char checkAddress(uint8_t *buffer);
