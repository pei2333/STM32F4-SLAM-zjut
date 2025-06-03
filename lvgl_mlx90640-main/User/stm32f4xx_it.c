/**
  ******************************************************************************
  * @file    SysTick/SysTick_Example/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "../BaseDrive/USART.h"
#include "string.h"
#include "../BaseDrive/mlx90640/mlx90640.h"
#include "delay.h"
#include "LED.h"
#include "../lvgl/src/hal/lv_hal_tick.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup SysTick_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    // 调用延时递减函数（包含系统时钟更新）
    TimingDelay_Decrement();
    
    // LVGL时钟节拍更新
    lv_tick_inc(1);
    
    // LED4状态指示（500ms闪烁）
    static unsigned int led_cnt = 0;
    led_cnt++;
    
    if(led_cnt == 500)
    {
        led_cnt = 0;
        LED4_REVERSE;
    }
    
    // UART5接收超时处理
    if(0 != Uart5.Time)
    {
        Uart5.Time--;
        if(Uart5.Time == 0)
        {
            Uart5.ReceiveFinish = 1;
        }
    }
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        // LED1状态指示
        static unsigned int cnt = 0;
        cnt++;
        
        if(cnt == 500)
        {
            cnt = 0;
            LED1_REVERSE;
        }
    }
}

void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    {        
        EXTI_ClearITPendingBit(EXTI_Line5);
        
        // GPIO按键处理或其他外部中断处理
        // TODO: 添加具体的处理逻辑
    }
}

void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
        
        // 接收数据
        uint8_t received_byte = USART_ReceiveData(UART5);
        
        // 防止缓冲区溢出
        if(Uart5.RXlenth >= sizeof(Uart5ReceiveBuf))
        {
            Uart5.RXlenth = 0;  // 重置接收
            return;
        }
        
        Uart5.Rxbuf[Uart5.RXlenth] = received_byte;
        
        switch (Uart5.RXlenth)
        {
            case 0:
                if(Uart5.Rxbuf[0] != 0xA5) // N10P帧头1
                {
                    Uart5.RXlenth = 0;
                }else{
                    Uart5.RXlenth++;
                }
                break;
            case 1:
                if(Uart5.Rxbuf[1] != 0x5A) // N10P帧头2
                {
                    Uart5.RXlenth = 0;
                }else{
                    Uart5.RXlenth++;
                }
                break;
            case 2:
                if(Uart5.Rxbuf[2] != 108) // N10P数据长度
                {
                    Uart5.RXlenth = 0;
                }else{
                    Uart5.RXlenth++;
                }
                break;
            default:
                Uart5.RXlenth++;
                if(Uart5.RXlenth >= 108) // N10P完整帧长度
                {
                    // 计算CRC校验
                    uint8_t crc = 0;
                    for(int i = 0; i < 107; i++)
                    {
                        crc += Uart5.Rxbuf[i];
                    }
                    
                    // 校验通过
                    if(crc == Uart5.Rxbuf[107])
                    {
                        Uart5.ReceiveFinish = 1;
                        LED2_REVERSE;  // 数据接收成功指示
                    }
                    else
                    {
                        // CRC校验失败，可以记录错误
                        LED3_ON;  // 错误指示
                    }
                    
                    Uart5.RXlenth = 0;
                }
                break;
        }
        
        // 设置接收超时
        Uart5.Time = 10;  // 10ms超时
    }
    
    // 处理其他UART5中断（如发送完成、错误等）
    if (USART_GetITStatus(UART5, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(UART5, USART_IT_TC);
        // 发送完成处理
    }
    
    if (USART_GetITStatus(UART5, USART_IT_ORE) != RESET)
    {
        USART_ClearITPendingBit(UART5, USART_IT_ORE);
        // 溢出错误处理
        USART_ReceiveData(UART5);  // 清除溢出标志
        LED3_ON;  // 错误指示
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
        // 定时读取ADXL345数据（200Hz）
        static uint8_t adxl_read_counter = 0;
        adxl_read_counter++;
        
        // 每5次TIM3中断读取一次ADXL345（如果TIM3是1kHz，则200Hz读取）
        if(adxl_read_counter >= 5)
        {
            adxl_read_counter = 0;
            
            // 使用新的ADXL345接口
            if(ADXL345_ReadData() != ADXL345_SUCCESS)
            {
                // ADXL345读取失败处理
                static uint8_t adxl_error_count = 0;
                adxl_error_count++;
                
                if(adxl_error_count > 10)
                {
                    LED3_ON;  // 持续错误指示
                    adxl_error_count = 0;
                }
            }
            else
            {
                LED3_OFF;  // 清除错误指示
            }
        }
    }
}

// 添加ADXL345中断处理（如果使用中断模式）
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
        
        // ADXL345数据就绪中断处理
        ADXL345_IRQHandler();
    }
}

// 添加I2C错误中断处理
void I2C1_ER_IRQHandler(void)
{
    // I2C错误处理
    if(I2C_GetITStatus(I2C1, I2C_IT_AF))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
        // 应答失败处理
    }
    
    if(I2C_GetITStatus(I2C1, I2C_IT_BERR))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
        // 总线错误处理
    }
    
    if(I2C_GetITStatus(I2C1, I2C_IT_ARLO))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_ARLO);
        // 仲裁丢失处理
    }
    
    if(I2C_GetITStatus(I2C1, I2C_IT_OVR))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
        // 溢出错误处理
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
