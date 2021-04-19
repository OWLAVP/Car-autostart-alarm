/*
 * usart.h
 *
 * Created: 19.02.2021 19:24:22
 *  Author: Userr
 */ 


#ifndef USART_H_
#define USART_H_
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