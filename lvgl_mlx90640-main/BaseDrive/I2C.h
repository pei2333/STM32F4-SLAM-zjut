// #ifndef I2C_H
// #define I2C_H
// #include "stm32f4xx.h"

// /*********************************************************************************************************
// //I2C引脚定义
// //SCL	PA1
// //SDA	PA0
// *********************************************************************************************************/
// //SCL		PA1
// #define  	SCL_L      		GPIO_ResetBits(GPIOG, GPIO_Pin_2)
// #define  	SCL_H      		GPIO_SetBits(GPIOG, GPIO_Pin_2)

// //SDA		PA0
// #define  	SDA_L      		GPIO_ResetBits(GPIOG, GPIO_Pin_4)
// #define  	SDA_H      		GPIO_SetBits(GPIOG, GPIO_Pin_4)

// #define  	SDA_READ()     	GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_4)

// //  函数声明
// void I2C_GPIO_Configuration(void);
// void I2C_SDA_Input(void);
// void I2C_SDA_Output(void);
// void I2C_delay(void);
// void I2C_Start(void);
// void I2C_Stop(void);
// void I2C_SendACK(unsigned char ack);
// void I2C_ReceiveACK(void);
// void I2C_SendByte(unsigned char dat);
// unsigned char I2C_ReceiveByte(void);
// #endif
