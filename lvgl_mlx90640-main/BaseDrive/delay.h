#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

// 函数声明
void delay_init(void);          // 延时初始化函数（统一接口）
void SysTick_Init(void);        // SysTick初始化
void delay_ms(uint32_t time);   // 毫秒延时
void delay_us(uint32_t time);   // 微秒延时
void TimingDelay_Decrement(void);  // 延时递减函数

// 获取系统时钟节拍
uint32_t get_system_ms(void);

#endif
