#include "PWM.h"

/**********************************************************************************************************
函数名称：TIM10_PWM_Init初始化函数
输入参数：自动重装值、时钟预分频数
输出参数：无
函数返回：无
TIM10_CH1    PB8
**********************************************************************************************************/
void TIM10_PWM_Init(unsigned int arr, unsigned int psc)
{		 					 
	GPIO_InitTypeDef            GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	            //  TIM10时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	            //  使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);            //  GPIOF9复用为定时器10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                           //  GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //  复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                //  速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //  推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //  上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);                              //  初始化PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler = psc;                          //  定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;         //  向上计数模式
	TIM_TimeBaseStructure.TIM_Period = arr;                             //  自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);                    //  初始化定时器10
	
	//  初始化TIM10 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                   //  选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;       //  比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           //  输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM10, &TIM_OCInitStructure);                           //  根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);                  //  使能TIM10在CCR1上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM10, ENABLE);                                //  ARPE使能 
	
	TIM_Cmd(TIM10, ENABLE);                                             //  使能TIM10
}
