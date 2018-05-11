#ifndef  F_CPU
#define F_CPU 8000000UL
#endif

#include	<avr/io.h>
#include	<avr/pgmspace.h>
#include	<avr/interrupt.h>
#include	<avr/wdt.h>
#include	<util/delay.h>

#include "i2c_master.h"

#define F_SCL 200000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

#define TW_START		0x08
#define TW_MT_SLA_ACK	0x18
#define TW_MR_SLA_ACK	0x40
#define TW_MT_DATA_ACK	0x28

void init_i2c(void)
{
	TWBR = (unsigned char)TWBR_val;
	TWCR = _BV(TWEN);						//Enable TWI-interface
}

unsigned char i2c_start(unsigned char address)
{
	// reset TWI control register
	TWCR = 0;
	// transmit START condition 
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) )
	{
			wdt_reset();
	}
	
	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){ return 1; }
	
	// load slave address into data register
	TWDR = address;
	// start transmission of address
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) )
	{
			wdt_reset();
	}
	
	// check if the device has acknowledged the READ / WRITE mode
	unsigned char twst = TWSR & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
	
	return 0;
}

unsigned char i2c_write(unsigned char data)
{
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) )
	{
			wdt_reset();
	}
	
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
	
	return 0;
}

unsigned char i2c_read_ack(void)
{
	
	// start TWI module and acknowledge data after reception
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA); 
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) )
	{
			wdt_reset();
	}
	// return received data from TWDR
	return TWDR;
}

unsigned char i2c_read_nack(void)
{
	
	// start receiving without acknowledging reception
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	while( !(TWCR & _BV(TWINT)) )
	{
			wdt_reset();
	}
	// return received data from TWDR
	return TWDR;
}

unsigned char i2c_transmit(unsigned char address, unsigned char* data, unsigned int length)
{
	if (i2c_start(address | I2C_WRITE)) return 1;
	
	for (unsigned int i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return 1;
	}
	
	i2c_stop();
	
	return 0;
}

unsigned char i2c_receive(unsigned char address, unsigned char* data, unsigned int length)

{
	if (i2c_start(address | I2C_READ)) return 1;
	
	for (unsigned int i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();
	
	i2c_stop();
	
	return 0;
}

unsigned char i2c_writeReg(unsigned char devaddr, unsigned int regaddr, unsigned char* data, unsigned int length)
{
	unsigned char i, j;
	for(i = 0 ; i < 10 ; i ++)
	{
		if (!(i2c_start(devaddr | 0x00))) // ACK
		{
			break;
		}
		else
		{
			//for(j = 0 ; j < 255 ; j++) _NOP();
			_delay_ms(1);
		}
	}

	if(i >= 10) return 1;

	i2c_write((unsigned char)((regaddr >> 8)&0x00ff));
	i2c_write((unsigned char)(regaddr & 0x00ff));

	for (unsigned int i = 0; i < length; i++)
	{
		if (i2c_write(data[i])) return 1;
	}

	i2c_stop();
//	for(i = 0 ; i < 40; i++)
//		for(j = 0 ; j < 255 ; j++) _NOP();
	_delay_ms(1);
	return 0;
}

unsigned char i2c_readReg(unsigned char devaddr, unsigned int regaddr, unsigned char* data, unsigned int length)
{
	if (i2c_start(devaddr)) return 1;

	i2c_write((unsigned char)((regaddr >> 8)&0x00ff));
	i2c_write((unsigned char)(regaddr & 0x00ff));

	if (i2c_start(devaddr | 0x01)) return 1;

	for (unsigned int i = 0; i < (length-1); i++)
	{
		data[i] = i2c_read_ack();
	}
	data[(length-1)] = i2c_read_nack();

	i2c_stop();

	return 0;
}

void i2c_stop(void)
{
	// transmit STOP condition
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}
