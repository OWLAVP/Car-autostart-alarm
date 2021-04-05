
#ifndef USART_H_
#define USART_H_
// #define F_CPU 16000000UL						/* Define CPU clock Frequency e.g. here its 8MHz */
// #include <avr/io.h>							/* Include AVR std. library file */
// #include <avr/interrupt.h>
// #include "usart.h"

//#define BAUD_PRESCALE (((F_CPU / (BAUDRATE * 16UL))) - 1)
// #define   USART_HAS_DATA   bit_is_set(UCSR0A, RXC0)
// #define   USART_READY      bit_is_set(UCSR0A, UDRE0)
void USART_Init();
void USART_TransmitInterrupt(uint8_t Buffer);
uint8_t USART_ReceivePolling();
void USART_TransmitPolling(uint8_t DataByte);
char USART_RxChar();						/* Data receiving function */
void USART_TxChar(char);					/* Data transmitting function */
void USART_SendString(char*);				/* Send string of USART data function */
void USART_sendHex(uint8_t znak);
void USART_Write_Int(uint8_t n);
void USART_Write_Float(float x);
void USART_Write_Word(uint16_t word);
void USART_Write_Long(int32_t data);
#endif /* USART_H_ */
