//***************************************************************************
//  File Name	 : I2Cvb1ioexp.c
//  Version	 : 1.0
//  Description  : AVR I2C Bus Master
//  IDE          : Atmel AVR Studio
//  Programmer   : Stokkink
//               :
//  Last Updated : April 2016
//***************************************************************************

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <compat/twi.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define MAX_TRIES 50

#define MCP23008_ID    0x40  // MCP23008 Device Identifier
#define MCP23008_ADDR  0x00  // MCP23008 Device Address

#define MCP9800_ID	   0x92  // MCP3221 Device Identifier
#define MCP9800_ADDR   0x02  // MCP3221 Device Address

#define IODIR 0x00           // MCP23008 I/O Direction Register
#define GPIO  0x09           // MCP23008 General Purpose I/O Register
#define OLAT  0x0A           // MCP23008 Output Latch Register
#define I2C_START 0
#define I2C_DATA 1
#define I2C_DATA_ACK 2
#define I2C_STOP 3
#define ACK 1
#define NACK 0
#define DATASIZE 32
#define UART_BUFF_SIZE 10

void USART_Transmit(char data[]);

uint8_t TX_buf[UART_BUFF_SIZE];
uint8_t TX_index = 0;
		float x = 0;
void USART_Init(void)
{
	
	// Set baud rate:
	UBRR0=103;                 //UBRR= Fosc /(16*9600) -1 =103.166= 103

	// enable receiver and transmitter
	UCSR0B |=(1<<RXEN0 |(1 <<TXEN0));

	// Set frame format : 8 data 2 stop bit

	UCSR0C = (1<<USBS0 )|(3<<UCSZ00);
}

void USART_Transmit(char data[])
{
	for(int i = 0; i < 5; i++)
	{
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) );
		/* Put data into buffer, sends the data */
		UDR0 = data[i];
	}
}

/* START I2C Routine */
unsigned char i2c_transmit(unsigned char type) {
	switch(type) {
		case I2C_START:    // Send Start Condition
		TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
		break;
		case I2C_DATA:     // Send Data with No-Acknowledge
		TWCR = (1 << TWINT) | (1 << TWEN);
		break;
		case I2C_DATA_ACK: // Send Data with Acknowledge
		TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
		break;
		case I2C_STOP:     // Send Stop Condition
		TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
		return 0;
	}
	// Wait for TWINT flag set on Register TWCR
	while (!(TWCR & (1 << TWINT)));
	// Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
	return (TWSR & 0xF8);
}

char i2c_start(unsigned int dev_id, unsigned int dev_addr, unsigned char rw_type)
{
	unsigned char n = 0;
	unsigned char twi_status;
	char r_val = -1;
	i2c_retry:
	if (n++ >= MAX_TRIES) return r_val;
	// Transmit Start Condition
	twi_status=i2c_transmit(I2C_START);

	// Check the TWI Status
	if (twi_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ((twi_status != TW_START) && (twi_status != TW_REP_START)) goto i2c_quit;
	// Send slave address (SLA_W)
	TWDR = (dev_id & 0xF0) | (dev_addr & 0x0E) | rw_type;
	//TWDR = (dev_addr & 0x0E) | rw_type;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (twi_status != TW_MT_SLA_ACK) goto i2c_quit;
	r_val=0;
	i2c_quit:
	return r_val;
}

void i2c_stop(void)
{
	unsigned char twi_status;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_STOP);
}

char i2c_write(char data)
{
	unsigned char twi_status;
	char r_val = -1;
	// Send the Data to I2C Bus
	TWDR = data;
	// Transmit I2C Data
	twi_status=i2c_transmit(I2C_DATA);
	// Check the TWSR status
	if (twi_status != TW_MT_DATA_ACK) goto i2c_quit;
	r_val=0;
	i2c_quit:
	return r_val;
}

char i2c_read(char *data,char ack_type)
{
	unsigned char twi_status;
	char r_val = -1;

	if (ack_type) {
		// Read I2C Data and Send Acknowledge
		twi_status=i2c_transmit(I2C_DATA_ACK);
		if (twi_status != TW_MR_DATA_ACK) goto i2c_quit;
		} else {
		// Read I2C Data and Send No Acknowledge
		twi_status=i2c_transmit(I2C_DATA);
		if (twi_status != TW_MR_DATA_NACK) goto i2c_quit;
	}
	// Get the Data
	*data=TWDR;
	r_val=0;
	i2c_quit:
	return r_val;
}

void Write_MCP23008(unsigned char reg_addr,unsigned char data)
{
	// Start the I2C Write Transmission
	i2c_start(MCP23008_ID,MCP23008_ADDR,TW_WRITE);
	// Sending the Register Address
	i2c_write(reg_addr);
	// Write data to MCP23008 Register
	i2c_write(data);
	// Stop I2C Transmission
	i2c_stop();
}

unsigned char Read_MCP23008(unsigned char reg_addr)
{
	char data;
	// Start the I2C Write Transmission
	i2c_start(MCP23008_ID,MCP23008_ADDR,TW_WRITE);
	// Read data from MCP23008 Register Address
	i2c_write(reg_addr);
	// Stop I2C Transmission
	i2c_stop();

	// Re-Start the I2C Read Transmission
	i2c_start(MCP23008_ID,MCP23008_ADDR,TW_READ);
	i2c_read(&data,NACK);

	// Stop I2C Transmission
	i2c_stop();

	return data;
}

unsigned char Read_MCP9800(unsigned char reg_addr)
{
	char data1, data2;
	// Start the I2C Write Transmission
	i2c_start(MCP9800_ID,MCP9800_ADDR,TW_WRITE);
	// Read data from MCP23008 Register Address
	i2c_write(reg_addr);
	// Stop I2C Transmission
	i2c_stop();

	// Start the I2C Write Transmission
	i2c_start(MCP9800_ID,MCP9800_ADDR,TW_READ);
	i2c_read(&data2,ACK);
	i2c_read(&data1,NACK);
	x = data2;
	// Stop I2C Transmission
	i2c_stop();
	return data1;
}

void Write_MCP9800(unsigned char reg_addr,unsigned char data)
{
	// Start the I2C Write Transmission
	i2c_start(MCP9800_ID,MCP9800_ADDR,TW_WRITE);
	// Sending the Register Address
	i2c_write(reg_addr);
	// Stop I2C Transmission
	i2c_stop();
}

void i2c_init(void)
{
	// Initial ATMega328P TWI/I2C Peripheral
	TWSR = 0x00;         // Select Prescaler of 1
	// SCL frequency = 11059200 / (16 + 2 * 48 * 1) = 98.743 kHz
	TWBR = 0x30;        // 48 Decimal
}

int main(void)
{
	char temp[10];
	char temp2[10];
	
	float y = 0;
	USART_Init();
	
	/* Clearing buffers */
	memset(TX_buf,0,sizeof(TX_buf));
	
	// Initial Master I2C
	i2c_init();
	// Initial the MCP23008 GP0 to GP7 as Output
	Write_MCP23008(IODIR,0b00000000);
	Write_MCP23008(GPIO,0b00000000);    // Reset all the Output Port
	
	// Loop Forever
	while(1) {
		
		// Write to MCP23008 GPIO Register sensor value
		Write_MCP23008(GPIO,x);
		y = Read_MCP9800(0x00);			//read LSB sensor value, also assign MSB value to x
		y *= 0.125;						//using equation to convert to temperature value
		sprintf(temp,"%.0f",x);			
		sprintf(temp2,"%.0f\n",y);
		USART_Transmit(temp);			//transmit MSB
		USART_Transmit(temp2);			//transmit LSB
		_delay_ms(100);
	}

	return 0;
}
/* EOF: I2Cvb1ioexp.c */

