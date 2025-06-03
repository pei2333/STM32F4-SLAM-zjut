#ifndef _TIME_H_
#define _TIME_H_

#include <stdint.h>

// TIM2定时器函数（1ms中断）
void TIM2_Configuration(void);

// TIM3定时器函数（可配置频率，默认200Hz）
void TIM3_Configuration(void);
void TIM3_Stop(void);
void TIM3_Start(void);
int8_t TIM3_SetFrequency(uint32_t frequency_hz);
uint32_t TIM3_GetCounter(void);
void TIM3_Deinit(void);

#endif
