#include "servo.h"

static void TIM10_PWM_Init(unsigned int arr, unsigned int psc);

// servo 0: PA8
// servo 1: PB8
void ServoInit(void){
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
	
	//复用不能写成下面,会出问题
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource8|GPIO_PinSource11,GPIO_AF_TIM1); //GPIO复用为定时器1
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1); //复用GPIOA_Pin8为TIM1_Ch1, 
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化P
	
    TIM_TimeBaseStructure.TIM_Period=20000 - 1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_Prescaler=168 - 1;  //定时器分频
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//默认就为0
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
 
	//初始化TIM1  PWM模式	 
	//PWM 模式 1–– 在递增计数模式下，只要 TIMx_CNT<TIMx_CCR1，通道 1 便为有效状态，否则为无效状态。在递减计数模式下，
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM1为正常占空比模式，PWM2为反极性模式
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低,有效电平为低电平
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;//在空闲时输出     低,这里的设置可以改变TIM_OCPolarity 如果没这句，第1通道有问题
	TIM_OCInitStructure.TIM_Pulse = 500; //输入通道1 CCR1（占空比数值）
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //Ch1初始化
	
//	TIM_OCInitStructure.TIM_Pulse = 5000;
//	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//通道4
	
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器,CCR自动装载默认也是打开的
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题
    

    // TIM10_PWM_Init(20000-1, 168-1);
    GPIO_InitTypeDef GPIO_InitStructure_10;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure_10;
	TIM_OCInitTypeDef  TIM_OCInitStructure_10;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	//TIM10时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTB时钟	
	
	//复用不能写成下面,会出问题
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource8|GPIO_PinSource11,GPIO_AF_TIM1); //GPIO复用为定时器1
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10); //复用GPIOA_Pin8为TIM1_Ch1, 
    
	GPIO_InitStructure_10.GPIO_Pin = GPIO_Pin_8;           //GPIO
	GPIO_InitStructure_10.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure_10.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure_10.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure_10.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure_10);              //初始化P
	
    TIM_TimeBaseStructure_10.TIM_Period=20000 - 1;   //自动重装载值
	TIM_TimeBaseStructure_10.TIM_Prescaler=168 - 1;  //定时器分频
    TIM_TimeBaseStructure_10.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure_10.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure_10.TIM_RepetitionCounter = 0x0;//默认就为0
	TIM_TimeBaseInit(TIM10,&TIM_TimeBaseStructure_10);//初始化定时器1
 
	//初始化TIM1  PWM模式	 
	//PWM 模式 1–– 在递增计数模式下，只要 TIMx_CNT<TIMx_CCR1，通道 1 便为有效状态，否则为无效状态。在递减计数模式下，
	TIM_OCInitStructure_10.TIM_OCMode = TIM_OCMode_PWM2; //PWM1为正常占空比模式，PWM2为反极性模式
 	TIM_OCInitStructure_10.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure_10.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低,有效电平为低电平
    TIM_OCInitStructure_10.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure_10.TIM_OCIdleState = TIM_OCNIdleState_Reset;//在空闲时输出     低,这里的设置可以改变TIM_OCPolarity 如果没这句，第1通道有问题
	TIM_OCInitStructure_10.TIM_Pulse = 500; //输入通道1 CCR1（占空比数值）
	TIM_OC1Init(TIM10, &TIM_OCInitStructure_10); //Ch1初始化
	
//	TIM_OCInitStructure.TIM_Pulse = 5000;
//	TIM_OC4Init(TIM1, &TIM_OCInitStructure);//通道4
	
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器,CCR自动装载默认也是打开的
	
	TIM_ARRPreloadConfig(TIM10,ENABLE);//ARPE使能 
	TIM_Cmd(TIM10, ENABLE);  //使能TIM1
	TIM_CtrlPWMOutputs(TIM10, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题
    


    SetServoAngle(0, 45);
    SetServoAngle(1, 0);
    
}



void SetServoAngle(uint8_t id, uint8_t angle){
    uint16_t pulse = 1500;
    if(id == 0){
		if(angle > 90){
			angle = 90;
		}else if(angle < 0){
			angle = 0;
		}
        pulse += 11 * angle;
        TIM_SetCompare1(TIM1, pulse);
    }else if(id == 1){
        if(angle > 45){
			angle = 45;
		}else if(angle < 0){
			angle = 0;
		}
		pulse += 11 * (45-angle);
        TIM_SetCompare1(TIM10, pulse); //  修改比较值，修改占空比
    }
}


void TIM10_PWM_Init(unsigned int arr, unsigned int psc)
{
	GPIO_InitTypeDef            GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  	            //  TIM10时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	            //  使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);            //  GPIOB8复用为定时器10
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                           //  GPIOB8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                        //  复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                //  速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //  推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                        //  上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);                              //  初始化
	  
	TIM_TimeBaseStructure.TIM_Prescaler = psc;                          //  定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;         //  向上计数模式
	TIM_TimeBaseStructure.TIM_Period = arr;                             //  自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;//默认就为0
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);                    //  初始化定时器10
	
	//  初始化TIM10 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                   //  选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低,有效电平为低电平
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;//在空闲时输出     低,这里的设置可以改变TIM_OCPolarity 如果没这句，第1通道有问题
	TIM_OCInitStructure.TIM_Pulse = 500; //输入通道1 CCR1（占空比数值）
    TIM_OC1Init(TIM10, &TIM_OCInitStructure);                           //  根据T指定的参数初始化外设TIM1 4OC1

    // TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器,CCR自动装载默认也是打开的
	
	// TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	// TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	// TIM_CtrlPWMOutputs(TIM1, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题
    

	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);                  //  使能TIM10在CCR1上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM10, ENABLE);                                //  ARPE使能 
	
	TIM_Cmd(TIM10, ENABLE);                                             //  使能TIM10
    TIM_CtrlPWMOutputs(TIM10, ENABLE);
}

