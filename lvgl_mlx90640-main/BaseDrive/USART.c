#include "stm32f4xx.h"
#include "USART.h"
#include "stdio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

unsigned char Uart3ReceiveBuf[300] = {0};
unsigned char Uart5ReceiveBuf[1544] = {0};

//  结构体定义
USARTDATA   Uart3;
USARTDATA   Uart6;
USARTDATA   Uart5;
/**********************************************************************************************************
函数名称：UART3初始化
输入参数：无
输出参数：无
**********************************************************************************************************/
// USART3_TX	 PB10	//  out
// USART3_RX	 PB11	//  in
void UART3_Init(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
	USART_InitTypeDef   USART_InitStructure;

    //Uart3初始化 
    Uart3.ReceiveFinish = 0;// 将ReceiveFinish标志位置设为0，表示未接收完成
    Uart3.RXlenth = 0;// 接收长度计数清零为0
    Uart3.Time = 0;// 超时计数器清零为0
    Uart3.Rxbuf = Uart3ReceiveBuf;// 接收缓冲区指向ReceiveBuffer

	//  使能GPIO_D时钟 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
	//  使能USART3时钟 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);


	USART_InitStructure.USART_BaudRate   = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;
	USART_InitStructure.USART_Parity     = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(USART3, &USART_InitStructure);

	/* 启用中断3 */
	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/**********************************************************************************************************
函数名称：putchar重定向函数
输入参数：无
输出参数：无
**********************************************************************************************************/
int fputc(int ch, FILE *f)
{
    USART3->SR;                                                         // 清除发送完成标志位，不能发送字符
    
    USART_SendData(USART3, (u8) ch);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
        ; 
    }
    
    return (ch);
}
// int fputc(int ch, FILE *f)
// {
//     UART5->SR;// 获取状态寄存器，清除发送完成标志位，不能发送字符                                                         // 清除发送完成标志位，不能发送字符
    
//     USART_SendData(UART5, (u8) ch); // 通过USART3发送一个字符
//     while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET)// 等待发送完成
//     {
//         ; 
//     }
    
//     return (ch); // 返回发送的字符
// }


/**********************************************************************************************************
函数名称：USART3发送数据函数
输入参数：数据指针，数据长度
输出参数：无
**********************************************************************************************************/
void USART3_Senddata(unsigned char *Data, unsigned int length)
{
    while(length--)
    {
        USART_SendData(USART3,*Data++);
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET);
    }
}

/**********************************************************************************************************
函数名称：UART6初始化
输入参数：波特率
输出参数：无
**********************************************************************************************************/
// USART6_TX	 PC6	//  out
// USART6_RX	 PC7	//  in
void UART6_Init(unsigned int baud)
{
	//  GPIO初始化
	//  USART初始化
	//  NVIC初始化
    
    // Uart6.ReceiveFinish = 0;
    // Uart6.RXlenth = 0;
    // Uart6.Time = 0;
    // Uart6.Rxbuf = Uart6ReceiveBuf;

	// //  使用GPIOA时钟 
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
	//  使用USART6时钟 
	
	/*  使用
			
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
			
			GPIO初始化GPIO6和7
			
			GPIO配置

			USART初始化
	USART_InitStructure.USART_BaudRate   = baud;
	

	USART_Init(USART6, &USART_InitStructure);

	/* 启用中断6 */
	

    /* NVIC configuration */
    /* Configure the Priority Group to 2 bits */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USARTx Interrupt */
    

    /* Enable USART */
    
}

/**********************************************************************************************************
函数名称：USART6发送数据函数
输入参数：数据指针，数据长度
输出参数：无
**********************************************************************************************************/
void USART6_Senddata(unsigned char *Data, unsigned int length)
{
    while(length--)
    {
        USART_SendData(USART6,*Data++);
        while (USART_GetFlagStatus(USART6, USART_FLAG_TC)==RESET);
    }
}

/**********************************************************************************************************
函数名称：UART1	初始化
输入参数：无
输出参数：无
**********************************************************************************************************/
// USART5_TX	 PC12	//  out
// USART5_RX	 PD2	//  in

/**
 * @brief 配置UART5
 * @param baudrate 波特率
 */
void UART5_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 初始化UART5数据结构
    Uart5.ReceiveFinish = 0;
    Uart5.RXlenth = 0;
    Uart5.Time = 0;
    Uart5.Rxbuf = Uart5ReceiveBuf;

    // 启用时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    // 配置GPIO引脚 - TX: PC12, RX: PD2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 配置引脚复用
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    // 配置UART5参数
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    // 配置中断
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 启用UART5和中断
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_ITConfig(UART5, USART_IT_ORE, ENABLE);  // 溢出错误中断
    USART_Cmd(UART5, ENABLE);
}

/**
 * @brief 发送一个字节
 * @param ch 要发送的字节
 */
void UART5_SendByte(uint8_t ch)
{
    // 等待发送缓冲区空
    while(USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
    
    // 发送数据
    USART_SendData(UART5, ch);
    
    // 等待发送完成
    while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
}

/**
 * @brief 发送字符串
 * @param str 要发送的字符串
 */
void UART5_SendString(char *str)
{
    if(str == NULL) return;
    
    while(*str)
    {
        UART5_SendByte(*str);
        str++;
    }
}

/**********************************************************************************************************
函数名称：UART5发送数据缓冲区
输入参数：数据指针，数据长度
输出参数：无
**********************************************************************************************************/
void UART5_SendBuffer(uint8_t *buffer, uint16_t length)
{
    if(buffer == NULL || length == 0) return;
    
    for(uint16_t i = 0; i < length; i++)
    {
        UART5_SendByte(buffer[i]);
    }
}

/**********************************************************************************************************
函数名称：获取UART5接收数据长度
输入参数：无
输出参数：接收到的数据长度
**********************************************************************************************************/
uint16_t UART5_GetReceiveLength(void)
{
    return Uart5.RXlenth;
}

/**********************************************************************************************************
函数名称：检查UART5是否接收完成
输入参数：无
输出参数：1-接收完成，0-未完成
**********************************************************************************************************/
uint8_t UART5_IsReceiveFinished(void)
{
    return Uart5.ReceiveFinish;
}

/**********************************************************************************************************
函数名称：清除UART5接收完成标志
输入参数：无
输出参数：无
**********************************************************************************************************/
void UART5_ClearReceiveFlag(void)
{
    Uart5.ReceiveFinish = 0;
    Uart5.RXlenth = 0;
}

/**********************************************************************************************************
函数名称：获取UART5接收缓冲区指针
输入参数：无
输出参数：接收缓冲区指针
**********************************************************************************************************/
uint8_t* UART5_GetReceiveBuffer(void)
{
    return Uart5.Rxbuf;
}

/**********************************************************************************************************
函数名称：UART5错误处理
输入参数：无
输出参数：无
**********************************************************************************************************/
void UART5_ErrorHandler(void)
{
    // 清除所有错误标志
    if(USART_GetFlagStatus(UART5, USART_FLAG_ORE) == SET)
    {
        USART_ReceiveData(UART5);  // 读取数据寄存器清除ORE标志
    }
    
    if(USART_GetFlagStatus(UART5, USART_FLAG_NE) == SET)
    {
        USART_ClearFlag(UART5, USART_FLAG_NE);
    }
    
    if(USART_GetFlagStatus(UART5, USART_FLAG_FE) == SET)
    {
        USART_ClearFlag(UART5, USART_FLAG_FE);
    }
    
    if(USART_GetFlagStatus(UART5, USART_FLAG_PE) == SET)
    {
        USART_ClearFlag(UART5, USART_FLAG_PE);
    }
    
    // 重置接收状态
    Uart5.RXlenth = 0;
    Uart5.ReceiveFinish = 0;
    Uart5.Time = 0;
}

