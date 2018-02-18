/**********************************************************************
* nrf24l01 lib                                                        *
*                                                                     *
* Copyright (c) Davide Gironi, 2012                                   *
* http://davidegironi.blogspot.com/                                   *
*                                                                     *
* References:                                                         *
*    This library is based upon nRF24L01 avr lib by Stefan Engelke    *
*    http://www.tinkerer.eu/AVRLib/nRF24L01                           *
**********************************************************************/

/**********************************************************************
* This library is free software; you can redistribute it and/or       *
* modify it under the terms of the GNU Lesser General Public          *
* License as published by the Free Software Foundation.               *
*                                                                     *
* This library is distributed in the hope that it will be useful,     *
* but WITHOUT ANY WARRANTY; without even the implied warranty of      *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU   *
* Lesser General Public License for more details.                     *
*                                                                     *
* You should have received a copy of the GNU Lesser General Public    *
* License along with this library; if not, write to the               *
* Free Software Foundation, Inc.,                                     *
* 59 Temple Place, Suite 330,                                         *
* Boston, MA                                                          *
* 02111-1307                                                          *
* USA                                                                 *
*                                                                     *
* For further information, visit www.gnu.org                          *
**********************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

#include "mirf.h"
#include "nRF24L01.h"

//initialize spi
void spi_init()
{
    DDR_SPI &= ~( (1<<SPI_MOSI) | (1<<SPI_MISO) | (1<<SPI_SS) | (1<<SPI_SCK) );
    DDR_SPI |= ( (1<<SPI_MOSI) | (1<<SPI_SS) | (1<<SPI_SCK) );

    SPCR = ((1<<SPE)|               // SPI Enable
            (0<<SPIE)|              // SPI Interupt Enable
            (0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
            (1<<MSTR)|              // Master/Slave select
            (0<<SPR1)|(1<<SPR0)|    // SPI Clock Rate
            (0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
            (0<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)

    SPSR = (1<<SPI2X); // double spi speed bit
}

//spi write one byte and read it back
uint8_t spi_writeread(uint8_t data)
{
    SPDR = data;
    while((SPSR & (1<<SPIF)) == 0);
    return SPDR;
}

//initialize mirf
void mirf_init() 
{
    DDRB |= ( (1<<CSN) | (1<<CE) );

    mirf_CE_lo;
    mirf_CSN_hi;

    spi_init();
}

//config mirf
void mirf_config() 
{
    mirf_write_register(RF_CH, mirf_CH); //set RF channel
    mirf_write_register(RX_PW_P0, mirf_PAYLOAD); //length of incoming payload
    mirf_write_register(RX_PW_P1, mirf_PAYLOAD);
    mirf_write_register(RX_PW_P2, mirf_PAYLOAD);
    mirf_write_register(RX_PW_P3, mirf_PAYLOAD);
    mirf_write_register(RX_PW_P4, mirf_PAYLOAD);
    mirf_write_register(RX_PW_P5, mirf_PAYLOAD);
    mirf_write_register(SETUP_RETR, mirf_RETR); // set retries
	#if mirf_ACK == 1  //auto acknowledgement
    	mirf_write_register(EN_AA, 0);
		#if mirf_ENABLED_P0 == 1
    		mirf_write_register(EN_AA, mirf_read_register(EN_AA) | (1<<ENAA_P0));
		#endif
		#if mirf_ENABLED_P1 == 1
    		mirf_write_register(EN_AA, mirf_read_register(EN_AA) | (1<<ENAA_P1));
		#endif
		#if mirf_ENABLED_P2 == 1
			mirf_write_register(EN_AA, mirf_read_register(EN_AA) | (1<<ENAA_P2));
		#endif
		#if mirf_ENABLED_P3 == 1
			mirf_write_register(EN_AA, mirf_read_register(EN_AA) | (1<<ENAA_P3));
		#endif
		#if mirf_ENABLED_P4 == 1
			mirf_write_register(EN_AA, mirf_read_register(EN_AA) | (1<<ENAA_P4));
		#endif
		#if mirf_ENABLED_P5 == 1
			mirf_write_register(EN_AA, mirf_read_register(EN_AA) | (1<<ENAA_P5));
		#endif
	#else
		mirf_write_register(EN_AA, 0); //auto acknowledgement
	#endif

	mirf_write_register(EN_RXADDR, 0);
	#if mirf_ENABLED_P0 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P0));
	#endif
	#if mirf_ENABLED_P1 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P1));
	#endif
	#if mirf_ENABLED_P2 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P2));
	#endif
	#if mirf_ENABLED_P3 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P3));
	#endif
	#if mirf_ENABLED_P4 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P4));
	#endif
	#if mirf_ENABLED_P5 == 1
		mirf_write_register(EN_RXADDR, mirf_read_register(EN_RXADDR) | (1<<ERX_P5));
	#endif

	/*#if mirf_USE_IRQ
		mirf_write_register(CONFIG, mirf_read_register(CONFIG) & ~(1<<MASK_RX_DR));
		mirf_write_register(CONFIG, mirf_read_register(CONFIG) & ~(1<<MASK_TX_DS));
	#endif*/

    mirf_setRX; // rx mode
    mirf_CE_hi;

	
	mirf_write_register(STATUS, (1<<RX_DR));
}

//set rx address
void mirf_set_rxaddr(uint8_t channel, uint8_t *addr)
{
    mirf_CE_lo;
    switch(channel)
    {
    	case 0:
    	    mirf_write_registers(RX_ADDR_P0, addr, 5);
    		break;
    	case 1:
    		mirf_write_registers(RX_ADDR_P1, addr, 5);
    	    break;
    	case 2:
    		mirf_write_registers(RX_ADDR_P2, addr, 5);
    	    break;
    	case 3:
    		mirf_write_registers(RX_ADDR_P3, addr, 5);
    	    break;
    	case 4:
    		mirf_write_registers(RX_ADDR_P4, addr, 5);
    	   break;
    	case 5:
    	   mirf_write_registers(RX_ADDR_P5, addr, 5);
    	   break;

    }
    mirf_CE_hi;
}

//set tx address
void mirf_set_txaddr(uint8_t *addr)
{
    mirf_write_registers(TX_ADDR, addr, 5);
}

//get status
uint8_t mirf_get_status()
{
	uint8_t status = 0;
	mirf_CSN_lo;
	status = spi_writeread(NOP);
	mirf_CSN_hi;
	return status;
}

//read one register
uint8_t mirf_read_register(uint8_t reg)
{
	mirf_CSN_lo;
	spi_writeread(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t result = spi_writeread(NOP);
    mirf_CSN_hi;
    return result;
}

//read many registers
void mirf_read_registers(uint8_t reg, uint8_t *value, uint8_t len)
{
    mirf_CSN_lo;
    spi_writeread(R_REGISTER | (REGISTER_MASK & reg));
	for (uint8_t i = 0; i < len; i++)
		value[i] = spi_writeread(NOP);
    mirf_CSN_hi;
}

//write one register
void mirf_write_register(uint8_t reg, uint8_t value)
{
	mirf_CSN_lo;
	spi_writeread(W_REGISTER | (REGISTER_MASK & reg));
	spi_writeread(value);
	mirf_CSN_hi;
}

//write many registers
void mirf_write_registers(uint8_t reg, uint8_t *value, uint8_t len)
{
    mirf_CSN_lo;
    spi_writeread(W_REGISTER | (REGISTER_MASK & reg));
	for (uint8_t i = 0; i < len; i++)
		 spi_writeread(value[i]);
    mirf_CSN_hi;
}

//check if there is rx data
uint8_t mirf_read_ready()
{
    uint8_t status = mirf_get_status();
    return status & (1<<RX_DR);
}

//read data
void mirf_read(uint8_t *data)
{
	//read rx register
mirf_CE_lo;
    mirf_CSN_lo;
    spi_writeread( R_RX_PAYLOAD );
    for (uint8_t i = 0; i < mirf_PAYLOAD; i++)
    	data[i] = spi_writeread(NOP);
    mirf_CSN_hi;
    //reset register
    mirf_write_register(STATUS, (1<<RX_DR));
mirf_CE_hi;
}

void mirf_printstatus()
{

}

//write data
void mirf_write(uint8_t *data)
{
	//set tx mode
	mirf_CE_lo;
	mirf_setTX;
	_delay_us(130);

	//flush tx fifo
	mirf_CSN_lo;
	spi_writeread(FLUSH_TX);
	mirf_CSN_hi;

	//write data
	mirf_CSN_lo;
	spi_writeread( W_TX_PAYLOAD );
	for (uint8_t i = 0; i < mirf_PAYLOAD; i++)
		spi_writeread(data[i]);
	mirf_CSN_hi;

	//start transmission
	mirf_CE_hi;
	_delay_us(10);
	mirf_CE_lo;

	//wait for the transmission to stop
	#if mirf_USE_IRQ == 0
		uint8_t breaked = 0;
		uint8_t tmp = 0;
		int timeout = 0;
		do
		{
			//stop if max_retries reached
			/*int a = mirf_read_register(OBSERVE_TX) & 0b1111;
			if((a) == mirf_RETR)
			{
				breaked = 1;
				break;
			}*/
			_delay_us(10);
			tmp = mirf_get_status();
			timeout++;
		}
		while( !(tmp & _BV(TX_DS)) && !(tmp & _BV(MAX_RT)) && timeout < 10000 );
		afterWriteData(tmp);
	#endif
}

void afterWriteData(uint8_t status){
	//reset OBSERVE_TX
	//if(breaked == 1)
	if(status & _BV(MAX_RT))
		mirf_write_register(RF_CH, mirf_CH);

	//reset registers
	mirf_write_register(mirf_read_register(STATUS), (1<<TX_DS)|(1<<MAX_RT));
	//mirf_reset();

	//set rx mode
	mirf_setRX;
	mirf_CE_hi;
}

uint8_t check_rx_fifo()
{
	//char fifo_reg = mirf_read_register(FIFO_STATUS);
	//return (fifo_reg & _BV(RX_FULL));
	return spi_writeread(R_RX_PL_WID);
}


void mirf_flush_tx()
{
	mirf_CSN_lo;
	spi_writeread(FLUSH_TX);
	mirf_CSN_hi;
}

void mirf_flush_rx()
{
	mirf_CSN_lo;
	spi_writeread(FLUSH_RX);
	mirf_CSN_hi;
}

void mirf_reset(){
	mirf_write_register(STATUS, (1<<TX_DS)|(1<<MAX_RT)|(1<<RX_DR));
}
