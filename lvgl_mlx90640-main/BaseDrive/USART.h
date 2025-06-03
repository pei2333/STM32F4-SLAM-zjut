#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include <stdint.h>

//  USARTݰ
struct UsartData                                                        
{		
	unsigned char *Rxbuf;
    unsigned int   RXlenth;
    unsigned char  Time;
    unsigned char  ReceiveFinish;
};
typedef  struct UsartData USARTDATA;
typedef  USARTDATA       *PUSARTDATA;

extern USARTDATA   Uart3;
extern USARTDATA   Uart6;
extern USARTDATA   Uart5;

// ��������
void UART3_Configuration(void);
void USART3_Senddata(unsigned char *Data, unsigned int length);
void UART6_Configuration(unsigned int baud);
void USART6_Senddata(unsigned char *Data, unsigned int length);
void UART5_Configuration(uint32_t baudrate);
void UART5_SendByte(uint8_t ch);
void UART5_SendString(char *str);

#endif
