#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"
#include <stdint.h>

//  定义USART接收数据包
struct UsartData                                                        
{		
	unsigned char *Rxbuf;
    unsigned int   RXlenth;
    unsigned char  Time;
    unsigned char  ReceiveFinish;
};
typedef  struct UsartData USARTDATA;
typedef  USARTDATA       *PUSARTDATA;

// 外部声明
extern USARTDATA   Uart3;
extern USARTDATA   Uart6;
extern USARTDATA   Uart5;

// 函数声明
void UART3_Init(void);
void USART3_Send(unsigned char *p, unsigned int length);
void UART6_Init(unsigned int baud);
void USART6_Send(unsigned char *p, unsigned int length);
void UART5_Init(uint32_t baudrate);

// 新增的UART5功能函数
void UART5_SendByte(uint8_t ch);
void UART5_SendString(char *str);
void UART5_SendBuffer(uint8_t *buffer, uint16_t length);
uint16_t UART5_GetReceiveLength(void);
uint8_t UART5_IsReceiveFinished(void);
void UART5_ClearReceiveFlag(void);
uint8_t* UART5_GetReceiveBuffer(void);
void UART5_ErrorHandler(void);

#endif
