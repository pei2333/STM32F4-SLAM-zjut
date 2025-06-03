#include "stm32f4xx.h"
#include "DCMotor.h"
#include "delay.h"
#include "PWM.h"

/******************************************************************************************
*函数名称：void RelayGpio_Init(void)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：LED初始化
//DCMOTOR_A      PC13           //  反转
//DCMOTOR_B      PE2            //  正转
//DCMOTOR_S      PE4            //  电机测速引脚
*******************************************************************************************/
void DCMotorGpio_Init(void)
{    
    GPIO_InitTypeDef   GPIO_InitStructure;
    
    //DCMOTOR_A      PC13
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
//    DCMOTOR_A_L;
    DCMOTOR_B_L;
    
    TIM10_PWM_Init(1000 - 1, 84 - 1);                                   //  84M/84=1Mhz的计数频率,重装载值1000，所以PWM频率为 1M/1000 = 1Khz.
    
    TIM_SetCompare1(TIM10, 0);	                                    //  修改比较值，修改占空比
}

/**********************************************************************************************************
函数名称：直流电机测试函数
输入参数：模式，0为停止，1为正转，2为反转
输出参数：无
**********************************************************************************************************/
void DCMotorTest(unsigned char mode)
{
    if (mode == 0)
    {
        DCMOTOR_A_L;
        DCMOTOR_B_L;
    }
    else if (mode == 1)
    {
        DCMOTOR_A_L;
        DCMOTOR_B_H;
    }
    else
    {
        DCMOTOR_A_H;
        DCMOTOR_B_L;
    }
}
