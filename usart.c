#define F_CPU 16000000UL // Defining the CPU Frequency
#include <avr/io.h>      // Contains all the I/O Register Macros
#include <util/delay.h>  // Generates a Blocking Delay
#include <avr/interrupt.h> // Contains all interrupt vectors
#include <stdlib.h>
#include "usart.h"
#define USART_BAUDRATE 9600 // Desired Baud Rate
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

volatile uint8_t USART_ReceiveBuffer; // Global Buffer
volatile uint8_t USART_TransmitBuffer; // Global Buffer
void USART_Init()
{
	// Set Baud Rate
	UBRRH = BAUD_PRESCALER >> 8;
	UBRRL = BAUD_PRESCALER;
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit character sizes */
	// Enable  Transmitter
	UCSRB = (1<<TXEN);
	//Enable Global Interrupts
	sei();
}
char USART_RxChar()									/* Data receiving function */
{
	while (!(UCSRA & (1 << RXC)));					/* Wait until new data receive */
	return(UDR);									/* Get and return received data */
}
void USART_TxChar(char data)						/* Data transmitting function */
{
	UDR = data;										/* Write data to be transmitting in UDR */
	while (!(UCSRA & (1<<UDRE)));					/* Wait until data transmit and buffer get empty */
}
void USART_SendString(char *str)					/* Send string of USART data function */
{
	int i=0;
	while (str[i]!=0)
	{
		USART_TxChar(str[i]);						/* Send each char of string till the NULL */
		i++;
	}
}
uint8_t USART_ReceivePolling()
{
	uint8_t DataByte;
	while (( UCSRA & (1<<RXC)) == 0) {}; // Do nothing until data have been received
	DataByte = UDR ;
	return DataByte;
}

void USART_TransmitPolling(uint8_t DataByte)
{
	while (( UCSRA & (1<<UDRE)) == 0) {}; // Do nothing until UDR is ready
	UDR = DataByte;
}


void readString(char myString[], uint8_t maxLength) {
	//char response;
	uint8_t i;
	i = 0;
	while (i < (maxLength - 1)) {                   /* prevent over-runs */
		//response = receiveByte();
		USART_TransmitPolling(USART_ReceiveBuffer);                                    /* echo */
		if (USART_ReceiveBuffer == '\r') {                     /* enter marks the end */
			break;
		}
		else {
			myString[i] = USART_ReceiveBuffer;                       /* add in a letter */
			i++;
		}
	}
	myString[i] = 0;                          /* terminal NULL character */
}
void USART_sendHex(uint8_t znak)
{
	
	unsigned char mn = 0xF0;
	unsigned char ln = 0x0F;
	
	mn &= znak;
	mn >>= 4;
	
	ln &= znak;
	
	if (mn > 9)
	mn = 'A'+mn-10;
	else
	mn += '0';
	if (ln > 9)
	ln ='A'+ln-10;
	else
	ln += '0';
	
	USART_TxChar(mn);
	USART_TxChar(ln);
	
	return;
}
void USART_Write_Int(uint8_t f_data)
{
	int x,y,i;
	unsigned char c[10];
	y=f_data;
	if(y==0)
	{
		USART_TxChar(0x30);
	}
	else
	{
		for(i=0;y>0;i++)
		{
			x=y%10;
			y=y/10;
			c[i]=x+'0';
		}
		
		i--;
		for(;i>=0;i--)
		{
			USART_TxChar(c[i]);
			_delay_ms(10);
		}
	}
}

void USART_Write_Word(uint16_t val)
{
	uint8_t dig1 = '0', dig2 = '0', dig3 = '0', dig4 = '0';
	// count value in 10000s place
	while(val >= 10000)
	{
		val -= 10000;
		dig1++;
	}
	// count value in 1000s place
	while(val >= 1000)
	{
		val -= 1000;
		dig2++;
	}
	// count value in 100s place
	while(val >= 100)
	{
		val -= 100;
		dig3++;
	}
	// count value in 10s place
	while(val >= 10)
	{
		val -= 10;
		dig4++;
	}
	// was previous value printed?
	uint8_t prevPrinted = 0;
	// print first digit (or ignore leading zeros)
	if(dig1 != '0') {USART_TxChar(dig1); prevPrinted = 1;}
	// print second digit (or ignore leading zeros)
	if(prevPrinted || (dig2 != '0')) {USART_TxChar(dig2); prevPrinted = 1;}
	// print third digit (or ignore leading zeros)
	if(prevPrinted || (dig3 != '0')) {USART_TxChar(dig3); prevPrinted = 1;}
	// print third digit (or ignore leading zeros)
	if(prevPrinted || (dig4 != '0')) {USART_TxChar(dig4); prevPrinted = 1;}
	// print final digit
	USART_TxChar(val + '0');
}
void USART_Write_Long(int32_t data)
{
	char u_tmp_buff[12]; // heading, 10 digit bytes, NULL		
	ltoa(data, u_tmp_buff, 10);
	USART_SendString(u_tmp_buff);
}
void USART_Write_Float(float f_data)
{
	unsigned char f_buf;
	unsigned int temp;
	temp=(int)f_data;
	USART_Write_Int(temp);
	f_data=(f_data-temp);
	if(f_data>0)
	{
		USART_TxChar('.');
		int i;
		for(i=0;i<4;i++)
		{
			f_data=f_data*10;
			temp=(int)f_data;
			temp=(temp%10);
			if(temp==0)
			{
				USART_TxChar(0x30);
			}
			else
			{
				f_buf=temp+'0';
				USART_TxChar(f_buf);
			}
		}
	}
}

// ISR(USART_UDRE_vect)
// {
// 	UDR0 = USART_TransmitBuffer;
// }
