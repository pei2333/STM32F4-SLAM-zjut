/**-------------------------------------------------------------------------------------------
** Created by:          qinyx
** Last modified Date:  2014-02-28
** Last Version:        V1.00
** Descriptions:        STM32F407嵌入式实验箱
**	  GPIO驱动文件
**
**-------------------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "TIME.h"

/**********************************************************************************************************
函数名称：定时器初始化函数
输入参数：无
输出参数：无
定时器：TIM2（1ms中断）
**********************************************************************************************************/
void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 设置中断优先级分组 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* 配置Timer2中断 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能TIM2时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* 配置TIM2的基本定时参数 */
    // 定时器时钟为84MHz，预分频和自动重装值设置如下：
    // t = 1 / (定时器频率) * 预分频值 * 自动重装值
    // t = 1 / (84M) * 8400 * 10000 = 1ms
    TIM_TimeBaseStructure.TIM_Period = 10000 - 1;                        // 自动重装值
    TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;                      // 预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;              // 时钟分频因子为1
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;          // 向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* 使能自动重装值预装载 */
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* 清除更新中断标志位 */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    /* 配置定时器更新中断（溢出中断） */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* 启动定时器 */
    TIM_Cmd(TIM2, ENABLE);
}

/**********************************************************************************************************
函数名称：TIM3定时器配置（200Hz，5ms中断用于ADXL345数据采集）
输入参数：无
输出参数：无
**********************************************************************************************************/
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 配置Timer3中断 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;            // 高优先级用于传感器采集
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 使能TIM3时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 配置TIM3的基本定时参数 - 200Hz（5ms） */
    // 定时器时钟为84MHz，预分频和自动重装值设置如下：
    // t = 1 / (84M) * 8400 * 50 = 5ms = 1/200Hz
    TIM_TimeBaseStructure.TIM_Period = 50 - 1;                          // 自动重装值（200Hz）
    TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;                     // 预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;             // 时钟分频因子为1
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;         // 向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* 使能自动重装值预装载 */
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    /* 清除更新中断标志位 */
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    /* 使能更新中断 */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* 使能定时器 */
    TIM_Cmd(TIM3, ENABLE);
}

/**********************************************************************************************************
函数名称：停止TIM3定时器
输入参数：无
输出参数：无
**********************************************************************************************************/
void TIM3_Stop(void)
{
    TIM_Cmd(TIM3, DISABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
}

/**********************************************************************************************************
函数名称：启动TIM3定时器
输入参数：无
输出参数：无
**********************************************************************************************************/
void TIM3_Start(void)
{
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

/**********************************************************************************************************
函数名称：重新配置TIM3定时器频率
输入参数：frequency_hz - 目标频率（Hz）
输出参数：0-成功，-1-失败
**********************************************************************************************************/
int8_t TIM3_SetFrequency(uint32_t frequency_hz)
{
    if(frequency_hz == 0 || frequency_hz > 10000) {
        return -1;  // 频率范围检查
    }
    
    // 停止定时器
    TIM3_Stop();
    
    // 计算新的重装值
    // t = 1/frequency_hz, period = t * 84MHz / 8400 = 84000000 / (8400 * frequency_hz)
    uint32_t period = 84000000 / (8400 * frequency_hz);
    
    if(period == 0 || period > 65535) {
        return -1;  // 重装值超出范围
    }
    
    // 更新重装值
    TIM_SetAutoreload(TIM3, period - 1);
    
    // 重新启动定时器
    TIM3_Start();
    
    return 0;
}

/**********************************************************************************************************
函数名称：获取TIM3计数值
输入参数：无
输出参数：当前计数值
**********************************************************************************************************/
uint32_t TIM3_GetCounter(void)
{
    return TIM_GetCounter(TIM3);
}

/**********************************************************************************************************
函数名称：TIM3去初始化
输入参数：无
输出参数：无
**********************************************************************************************************/
void TIM3_Deinit(void)
{
    TIM3_Stop();
    TIM_DeInit(TIM3);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
}

