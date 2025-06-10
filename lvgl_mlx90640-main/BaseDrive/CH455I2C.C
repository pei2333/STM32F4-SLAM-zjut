/* ***************************
 Website:  http://wch.cn
 Email:    tech@wch.cn
 Author:   W.ch 2008.7
* ****************************/
#include "CH455I2C.H"		// 修改该文件以适应硬件环境/单片机型号等
#include "I2C.h"

u8 Alarm_flag = 0;

/**********************************************************************************************************
函数名称：报警输入引脚配置
输入参数：无
输出参数：无
函数返回：无
//报警信号输入引脚
//ALARM(PA0)
**********************************************************************************************************/
void Alarm_GPIO_Configuration(void)
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);               //  使能IO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);              //  使能SYSCFG时钟

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;                        //  输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //  设置上接
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                           //  IO口为0
    GPIO_Init(GPIOG, &GPIO_InitStructure);                             

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource5);       //  初始化中断线0

    EXTI_InitStructure.EXTI_Line = EXTI_Line5;                          //  配置中断线为中断线0
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;                 //  配置中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;             //  配置为下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                           //  配置中断线使能
    EXTI_Init(&EXTI_InitStructure);                              

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;             
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void init_CH455( void ) //初始化CH455
{
    I2C_GPIO_Configuration();
    Alarm_GPIO_Configuration();
}

void CH455_Write( u16 cmd )	//写命令
{
	I2C_Start();               //启动总线
   	I2C_SendByte(((u8)(cmd>>7)&CH455_I2C_MASK)|CH455_I2C_ADDR);
   	I2C_SendByte((u8)cmd);               //发送数据
  	I2C_Stop();                 //结束总线 
}

u8 CH455_Read(void)		//读取按键
{
	u8 keycode;
   	I2C_Start();                //启动总线
   	I2C_SendByte((u8)(CH455_GET_KEY>>7)&CH455_I2C_MASK|0x01|CH455_I2C_ADDR);
   	keycode=I2C_ReceiveByte();      //读取数据
	I2C_Stop();                //结束总线
	return keycode;
}

