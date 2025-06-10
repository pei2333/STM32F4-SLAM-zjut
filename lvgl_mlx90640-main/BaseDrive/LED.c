/**-------------------------------------------------------------------------------------------
** Created by:          qinyx
** Last modified Date:  2014-02-28
** Last Version:        V1.00
** Descriptions:        STM32F407Ƕ��ʽʵ����
**	  Gpio�����ļ�
**
**-------------------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "LED.h"

/*************************************************************************
*函数名称：void LED_Init(void)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：LED初始化
//LED1      PF0
//LED2      PF1
//LED3      PF2
//LED4      PF3
//LED5      PF4
//LED6      PF5
//LED7      PF6
//LED8      PF7
*************************************************************************/
void LED_Init(void)
{    
    GPIO_InitTypeDef  GPIO_InitStructure;
    
    //  LED
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);   //ʹ��GPIOBʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//LED��Ӧ����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;           //ͨ�����ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //������������
    GPIO_Init(GPIOF, &GPIO_InitStructure);                  //��ʼ�� GPIO
    
    LED1_OFF;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;
	LED5_OFF;
    LED6_OFF;
    LED7_OFF;
    LED8_OFF;
}
