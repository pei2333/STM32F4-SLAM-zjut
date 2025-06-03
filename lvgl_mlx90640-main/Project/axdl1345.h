#ifndef __ADXL345_H
#define __ADXL345_H

#include "stm32f4xx.h"

#define DEVICE_ID		0X00 	//器件ID,0XE5
#define THRESH_TAP		0X1D   	//敲击阀值
#define OFSX			0X1E
#define OFSY			0X1F
#define OFSZ			0X20
#define DUR				0X21
#define Latent			0X22
#define Window  		0X23 
#define THRESH_ACK		0X24
#define THRESH_INACT	0X25 
#define TIME_INACT		0X26
#define ACT_INACT_CTL	0X27	 
#define THRESH_FF		0X28	
#define TIME_FF			0X29 
#define TAP_AXES		0X2A  
#define ACT_TAP_STATUS  0X2B 
#define BW_RATE			0X2C 
#define POWER_CTL		0X2D 

#define INT_ENABLE		0X2E
#define INT_MAP			0X2F
#define INT_SOURCE  	0X30
#define DATA_FORMAT	    0X31
#define DATA_X0			0X32
#define DATA_X1			0X33
#define DATA_Y0			0X34
#define DATA_Y1			0X35
#define DATA_Z0			0X36
#define DATA_Z1			0X37
#define FIFO_CTL		0X38
#define FIFO_STATUS		0X39

//#define     NOP             0xFF    // Define No Operation, might be used to read status register
#define     NOP             0    // Define No Operation, might be used to read status register


/*********************************************************************************************************	
//ADXL345硬件资源引脚定义
//ADXL345_CS(PF14)     OUT
//ADXL345_CLK(PF11)    OUT
//ADXL345_DIN(PF12)    OUT
//ADXL345_DO(PF13)     IN
*********************************************************************************************************/
//ADXL345_CS(PF14)     OUT
#define  	ADXL345_CS_L          GPIO_ResetBits(GPIOF, GPIO_Pin_14)
#define  	ADXL345_CS_H          GPIO_SetBits(GPIOF, GPIO_Pin_14)

//ADXL345_CLK(PF11)    OUT
#define  	ADXL345_CLK_L         GPIO_ResetBits(GPIOF, GPIO_Pin_11)
#define  	ADXL345_CLK_H         GPIO_SetBits(GPIOF, GPIO_Pin_11)

//ADXL345_DIN(PF12)     OUT
#define  	ADXL345_DIN_L          GPIO_ResetBits(GPIOF, GPIO_Pin_12)
#define  	ADXL345_DIN_H          GPIO_SetBits(GPIOF, GPIO_Pin_12)

//ADXL345_DO(PF13)      IN
#define  	ADXL345_DO   	      GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_13)



// 函数声明
void ADXL345_ISP_Init(void);
void spi_clk(void);
unsigned char SPI_RW_Byte(unsigned char dat);
void ADXL345WriteReg(unsigned char addr, unsigned char val);
unsigned char ADXL345_S_WriteBuf(unsigned char reg , unsigned char *pBuf, unsigned char bytes);
unsigned char ADXL345ReadReg(unsigned char addr);
unsigned char ADXL345_Init(void);
void ADXL345ReadAvval(short *x, short *y, short *z);
void ADXL345_AUTO_Adjust(char *xval, char *yval, char *zval);
void ADXL345Read_XYZ(short *x, short *y, short *z);
short ADXL345Get_Angle(float x, float y, float z, unsigned char dir);
#endif
