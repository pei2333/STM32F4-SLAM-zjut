/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "../BaseDrive/USART.h"
#include "../BaseDrive/adxl345.h"
#include "delay.h"
#include "LED.h"
#include "../lvgl/lvgl.h"

extern void lv_tick_inc(uint32_t tick_period);

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t system_ms_counter = 0;
static volatile uint32_t tim3_adxl345_errors = 0;

typedef struct {
    uint32_t overflow_count;
    uint32_t overrun_count;
    uint32_t crc_error_count;
} UART5_ErrorStats;

static volatile UART5_ErrorStats uart5_errors = {0};

/* Exception Handlers --------------------------------------------------------*/
void NMI_Handler(void) {}
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    TimingDelay_Decrement();
    lv_tick_inc(1);
    
    if(system_ms_counter < 0xFFFFFFFF) {
        system_ms_counter++;
    } else {
        system_ms_counter = 0;
    }
    
    static unsigned int led_cnt = 0;
    led_cnt++;
    if(led_cnt == 500)
    {
        led_cnt = 0;
        LED4_REVERSE;
    }
    
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
        
        static unsigned int cnt = 0;
        cnt++;
        if(cnt == 500)
        {
            cnt = 0;
            LED1_REVERSE;
        }
    }
}

void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
        
        uint8_t received_byte = USART_ReceiveData(UART5);
        
        if(Uart5.RXlenth >= 1544)
        {
            Uart5.RXlenth = 0;
            uart5_errors.overflow_count++;
            return;
        }
        
        switch (Uart5.RXlenth)
        {
            case 0:
                if(received_byte == 0xA5)
                {
                    Uart5.Rxbuf[0] = received_byte;
                    Uart5.RXlenth = 1;
                }
                break;
                
            case 1:
                if(received_byte == 0x5A)
                {
                    Uart5.Rxbuf[1] = received_byte;
                    Uart5.RXlenth = 2;
                }
                else if(received_byte == 0xA5)
                {
                    Uart5.Rxbuf[0] = received_byte;
                    Uart5.RXlenth = 1;
                }
                else
                {
                    Uart5.RXlenth = 0;
                }
                break;
                
            case 2:
                Uart5.Rxbuf[2] = received_byte;
                Uart5.RXlenth = 3;
                break;
                
            default:
                Uart5.Rxbuf[Uart5.RXlenth] = received_byte;
                Uart5.RXlenth++;
                
                uint8_t frame_len = Uart5.Rxbuf[2];
                if(frame_len > 10 && Uart5.RXlenth >= frame_len) 
                {
                    uint8_t crc = 0;
                    for(int i = 0; i < frame_len - 1; i++)
                    {
                        crc += Uart5.Rxbuf[i];
                    }
                    
                    if(crc == Uart5.Rxbuf[frame_len - 1])
                    {
                        Uart5.ReceiveFinish = 1;
                        LED2_REVERSE;
                    }
                    else
                    {
                        LED3_REVERSE;
                        uart5_errors.crc_error_count++;
                    }
                    
                    Uart5.RXlenth = 0;
                    Uart5.Time = 0;
                }
                break;
        }
        
        Uart5.Time = 200;
    }
    
    if (USART_GetITStatus(UART5, USART_IT_ORE) != RESET)
    {
        USART_ClearITPendingBit(UART5, USART_IT_ORE);
        USART_ReceiveData(UART5);
        LED3_ON;
        uart5_errors.overrun_count++;
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        
        static uint8_t adxl_read_counter = 0;
        adxl_read_counter++;
        
        if(adxl_read_counter >= 5)
        {
            adxl_read_counter = 0;
            
            int16_t dummy_x, dummy_y, dummy_z;
            if(ADXL345_ReadRawData(&dummy_x, &dummy_y, &dummy_z) != 0)
            {
                tim3_adxl345_errors++;
                
                static uint8_t adxl_error_count = 0;
                adxl_error_count++;
                
                if(adxl_error_count > 10)
                {
                    LED4_ON;
                    adxl_error_count = 0;
                }
            }
            else
            {
                LED4_OFF;
            }
        }
        
        if(Uart5.Time > 0)
        {
            Uart5.Time--;
            if(Uart5.Time == 0)
            {
                if(Uart5.RXlenth > 0)
                {
                    Uart5.ReceiveFinish = 1;
                }
            }
        }
    }
} 