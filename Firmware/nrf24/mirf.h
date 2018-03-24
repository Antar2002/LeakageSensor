#ifndef _MIRF_H_
#define _MIRF_H_

#include <avr/io.h>

#define CE  PB1
#define CSN PB2

#define mirf_USE_IRQ

#define PORT_SPI  PORTB
#define DDR_SPI   DDRB
#define SPI_MISO  DDB4
#define SPI_MOSI  DDB3
#define SPI_SCK   DDB5
#define SPI_SS    DDB2
/*#define SPI_MISO  DDB4
#define SPI_MOSI  DDB3
#define SPI_SCK   DDB5
#define SPI_SS    DDB1*/

#define mirf_CH      2 //transmission channel
#define mirf_PAYLOAD 16 //payload lenght

//default configuration
#ifdef mirf_USE_IRQ
	#define mirf_CONFIG ( (1<<EN_CRC) | (0<<CRCO) | (0<<MASK_RX_DR) | (0<<MASK_TX_DS) | (0<<MASK_TX_DS) )
#else
	#define mirf_CONFIG ( (1<<EN_CRC) | (0<<CRCO) )
#endif

#define mirf_ACK 1 //auto ack enabled
#define mirf_RETR 0x06 //auto ack enabled
#define mirf_ENABLED_P0 1
#define mirf_ENABLED_P1 0
#define mirf_ENABLED_P2 0
#define mirf_ENABLED_P3 0
#define mirf_ENABLED_P4 0
#define mirf_ENABLED_P5 0


#define mirf_CSN_hi PORTB |=  (1<<CSN);
#define mirf_CSN_lo PORTB &= ~(1<<CSN);
#define mirf_CE_hi PORTB |=  (1<<CE);
#define mirf_CE_lo PORTB &= ~(1<<CE);


#define mirf_setTX mirf_write_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define mirf_setRX mirf_write_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )

#define mirf_power_down mirf_write_register(CONFIG, mirf_CONFIG )


extern void spi_init();
extern uint8_t spi_writeread (uint8_t data);

extern void mirf_init();
extern void mirf_config();
extern uint8_t mirf_get_status();
extern void mirf_set_rxaddr(uint8_t channel, uint8_t *addr);
extern void mirf_set_txaddr(uint8_t *addr);
extern uint8_t mirf_read_ready();
extern void mirf_read(uint8_t *data);
extern void mirf_write(uint8_t *data);

extern void mirf_write_register(uint8_t reg, uint8_t value);
extern void mirf_write_registers(uint8_t reg, uint8_t *value, uint8_t len);
extern uint8_t mirf_read_register(uint8_t reg);
extern void mirf_read_registers(uint8_t reg, uint8_t *value, uint8_t len);

extern uint8_t check_rx_fifo();

extern void mirf_flush_tx();
extern void mirf_flush_rx();

extern void afterWriteData(uint8_t status);

extern void mirf_reset();
extern void mirf_reset_rx();
extern void mirf_reset_tx();
extern char mirf_is_available();

#endif /* _MIRF_H_ */
