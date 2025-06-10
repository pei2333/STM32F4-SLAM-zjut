#include "delay.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <stdlib.h>  // 添加标准库头文件以使用abs函数
#include <string.h>
#include <math.h>
#include <stdbool.h>  

#include "../BaseDrive/TIME.h"
#include "../BaseDrive/KEY.h" 
#include "../BaseDrive/TOUCH/touch.h"  // 添加触摸库头文件

// LCD和LED函数声明
extern void LCD_Init(void);
extern void LCD_Clear(uint16_t color);
extern void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
extern void LCD_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t size, uint8_t *p);
extern void LED_Init(void);

// LCD背光控制声明
extern void PA9_Init_Output(void);
extern void PA9_Set_High(void);

// LED控制宏
#define LED1_ON         GPIO_ResetBits(GPIOF, GPIO_Pin_9)
#define LED1_OFF        GPIO_SetBits(GPIOF, GPIO_Pin_9)
#define LED1_REVERSE    GPIO_WriteBit(GPIOF, GPIO_Pin_9, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_9)))

#define LED2_ON         GPIO_ResetBits(GPIOF, GPIO_Pin_10)
#define LED2_OFF        GPIO_SetBits(GPIOF, GPIO_Pin_10)
#define LED2_REVERSE    GPIO_WriteBit(GPIOF, GPIO_Pin_10, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_10)))

#define LED3_ON         GPIO_ResetBits(GPIOE, GPIO_Pin_13)
#define LED3_OFF        GPIO_SetBits(GPIOE, GPIO_Pin_13)
#define LED3_REVERSE    GPIO_WriteBit(GPIOE, GPIO_Pin_13, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_13)))

#define LED4_ON         GPIO_ResetBits(GPIOE, GPIO_Pin_14)
#define LED4_OFF        GPIO_SetBits(GPIOE, GPIO_Pin_14)
#define LED4_REVERSE    GPIO_WriteBit(GPIOE, GPIO_Pin_14, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_14)))

// ADXL345函数声明 - 修复返回值类型
extern int8_t ADXL345_Init(void);
extern int8_t ADXL345_ReadRawData(int16_t *x, int16_t *y, int16_t *z);

// 全局变量，提前声明用于不同模式访问
short adxl_x_g = 0, adxl_y_g = 0, adxl_z_g = 0;

// ADXL345包装函数
void ADXL345_RD_XYZ(short *x, short *y, short *z) {
    int16_t raw_x, raw_y, raw_z;
    if(ADXL345_ReadRawData(&raw_x, &raw_y, &raw_z) == 0) {
        *x = (short)raw_x;
        *y = (short)raw_y;
        *z = (short)raw_z;
    } else {
        *x = 0;
        *y = 0;
        *z = 0;
    }
}

// UART5函数声明
extern void UART5_Init(uint32_t baudrate);
extern void UART5_SendBuffer(uint8_t *buffer, uint16_t length);
extern uint16_t UART5_GetReceiveLength(void);
extern uint8_t UART5_IsReceiveFinished(void);
extern void UART5_ClearReceiveFlag(void);
extern unsigned char Uart5ReceiveBuf[1544];

// I2C诊断函数
int8_t I2C1_ScanDevice(uint8_t addr);
uint8_t I2C1_ReadDeviceID(uint8_t addr, uint8_t reg);

// 新增：详细I2C设备诊断
void I2C1_DiagnoseDevice(uint8_t addr)
{
    char buffer[100];
    
    // 读取多个常见寄存器
    uint8_t reg_00 = I2C1_ReadDeviceID(addr, 0x00);  // WHO_AM_I
    uint8_t reg_0F = I2C1_ReadDeviceID(addr, 0x0F);  // WHO_AM_I (备用)
    uint8_t reg_75 = I2C1_ReadDeviceID(addr, 0x75);  // WHO_AM_I (MPU系列)
    uint8_t reg_2D = I2C1_ReadDeviceID(addr, 0x2D);  // POWER_CTL (ADXL345)
    
    sprintf(buffer, "0x%02X: 00=%02X 0F=%02X 75=%02X 2D=%02X", 
           addr, reg_00, reg_0F, reg_75, reg_2D);
    LCD_ShowString(20, 240 + (addr & 0x0F) * 15, 350, 16, 12, (u8*)buffer);
    
    // 判断设备类型
    if(reg_00 == 0xE5) {
        LCD_ShowString(400, 240 + (addr & 0x0F) * 15, 200, 16, 12, (u8*)"-> ADXL345");
    } else if(reg_75 == 0x68 || reg_75 == 0x71) {
        LCD_ShowString(400, 240 + (addr & 0x0F) * 15, 200, 16, 12, (u8*)"-> MPU6050/9250");
    } else if(reg_0F == 0x33) {
        LCD_ShowString(400, 240 + (addr & 0x0F) * 15, 200, 16, 12, (u8*)"-> MLX90640");
    } else if(addr >= 0x08 && addr <= 0x17) {
        // 对0x08-0x0F范围的设备进行特殊检测
        // 检查ADXL345的其他特征寄存器
        uint8_t reg_31 = I2C1_ReadDeviceID(addr, 0x31);  // DATA_FORMAT
        uint8_t reg_32 = I2C1_ReadDeviceID(addr, 0x32);  // DATAX0
        uint8_t reg_2C = I2C1_ReadDeviceID(addr, 0x2C);  // BW_RATE
        
        // ADXL345在不同地址的可能性检测
        if((reg_31 & 0x0F) <= 0x03 && reg_2C <= 0x0F) {
            LCD_ShowString(400, 240 + (addr & 0x0F) * 15, 200, 16, 12, (u8*)"-> Maybe ADXL345?");
        } else {
            LCD_ShowString(400, 240 + (addr & 0x0F) * 15, 200, 16, 12, (u8*)"-> Unknown");
        }
    }
}

// UART5硬件层诊断
void UART5_HardwareDiagnose(void)
{
    char buffer[100];
    
    // 检查GPIO配置
    uint32_t gpioc_moder = GPIOC->MODER;
    uint32_t gpiod_moder = GPIOD->MODER;
    uint32_t gpioc_afr_h = GPIOC->AFR[1];  // PC12 AFR
    uint32_t gpiod_afr_l = GPIOD->AFR[0];  // PD2 AFR
    
    sprintf(buffer, "GPIO: PC_MODE=%08lX PD_MODE=%08lX", gpioc_moder, gpiod_moder);
    LCD_ShowString(420, 300, 350, 16, 12, (u8*)buffer);
    
    sprintf(buffer, "AFR: PC12=%02lX PD2=%02lX", 
           (gpioc_afr_h >> 16) & 0xF, (gpiod_afr_l >> 8) & 0xF);
    LCD_ShowString(420, 315, 350, 16, 12, (u8*)buffer);
    
    // 检查UART5时钟
    uint32_t uart5_enabled = RCC->APB1ENR & RCC_APB1ENR_UART5EN;
    sprintf(buffer, "UART5_CLK: %s", uart5_enabled ? "ENABLED" : "DISABLED");
    LCD_ShowString(420, 330, 200, 16, 12, (u8*)buffer);
    
    // 检查UART5寄存器状态
    sprintf(buffer, "SR=%04X CR1=%04X CR2=%04X CR3=%04X", 
           UART5->SR, UART5->CR1, UART5->CR2, UART5->CR3);
    LCD_ShowString(420, 345, 350, 16, 12, (u8*)buffer);
    
    sprintf(buffer, "BRR=%04X (calc_baud=%lu)", 
           UART5->BRR, 84000000 / (16 * UART5->BRR));
    LCD_ShowString(420, 360, 350, 16, 12, (u8*)buffer);
    
    // 检查中断配置状态
    uint32_t rxne_enabled = UART5->CR1 & USART_CR1_RXNEIE;
    uint32_t uart_enabled = UART5->CR1 & USART_CR1_UE;
    sprintf(buffer, "INT: RXNE=%s UE=%s", rxne_enabled ? "ON" : "OFF", uart_enabled ? "ON" : "OFF");
    LCD_ShowString(420, 375, 200, 16, 12, (u8*)buffer);
    
    // 如果中断被禁用，重新启用
    if(!rxne_enabled || !uart_enabled) {
        UART5_Init(460800);  // 重新初始化
        LCD_ShowString(420, 390, 200, 16, 12, (u8*)"UART5 REINIT!");
    }
}

// === N10P雷达数据结构定义 ===
#define N10P_FRAME_SIZE         108
#define N10P_POINTS_PER_FRAME   32
#define N10P_FRAME_HEADER1      0xA5
#define N10P_FRAME_HEADER2      0x5A

typedef struct {
    uint16_t distance;      // 距离，单位：mm
    uint8_t  intensity;     // 强度值
    float    angle;         // 角度，单位：度
} N10P_Point_t;

typedef struct {
    uint8_t  valid;             // 帧是否有效
    uint16_t start_angle;       // 起始角度 (单位：角度×100)
    uint16_t end_angle;         // 结束角度 (单位：角度×100)
    uint16_t speed;             // 转速
    N10P_Point_t points[N10P_POINTS_PER_FRAME];  // 32个点的数据
    uint32_t timestamp;         // 时间戳
} N10P_Frame_t;

typedef struct {
    uint8_t  rx_buffer[N10P_FRAME_SIZE * 2];
    uint16_t rx_index;
    uint8_t  frame_ready;
    uint32_t frame_count;
    uint32_t error_count;
    N10P_Frame_t current_frame;
} N10P_Status_t;

// N10P雷达状态变量
N10P_Status_t n10p_status = {0};

// N10P雷达成功读取计数
uint32_t n10p_success_count = 0;
uint32_t n10p_total_attempts = 0;

// === N10P雷达处理函数的前向声明 ===
uint8_t N10P_CalculateCRC(uint8_t* data, uint16_t length);
float N10P_GetPointAngle(uint16_t start_angle, uint16_t end_angle, uint8_t point_index);
uint8_t N10P_ParseFrame(void);
uint8_t N10P_ParsePartialFrame(void);
void N10P_ProcessData(void);
void N10P_SendTestCommand(void);

// 新增函数：获取系统时间戳
uint32_t GetSystemTick(void)
{
    static uint32_t tick = 0;
    return tick++;
}

// === N10P雷达处理函数 ===
uint8_t N10P_CalculateCRC(uint8_t* data, uint16_t length)
{
    uint32_t sum = 0;
    for(uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

float N10P_GetPointAngle(uint16_t start_angle, uint16_t end_angle, uint8_t point_index)
{
    float start_deg = (float)start_angle / 100.0f;
    float end_deg = (float)end_angle / 100.0f;
    
    // 处理角度跨越360度的情况
    if(end_deg < start_deg) {
        end_deg += 360.0f;
    }
    
    float angle_step = (end_deg - start_deg) / (N10P_POINTS_PER_FRAME - 1);
    float angle = start_deg + (point_index * angle_step);
    
    // 标准化角度到0-360度
    while(angle >= 360.0f) angle -= 360.0f;
    while(angle < 0.0f) angle += 360.0f;
    
    return angle;
}

uint8_t N10P_ParseFrame(void)
{
    if(n10p_status.rx_index < N10P_FRAME_SIZE) {
        return 0; // 数据不够
    }
    
    // 查找帧头
    uint16_t header_pos = 0;
    uint8_t found_header = 0;
    
    for(uint16_t i = 0; i <= n10p_status.rx_index - N10P_FRAME_SIZE; i++) {
        if(n10p_status.rx_buffer[i] == N10P_FRAME_HEADER1 && 
           n10p_status.rx_buffer[i+1] == N10P_FRAME_HEADER2) {
            header_pos = i;
            found_header = 1;
            break;
        }
    }
    
    if(!found_header) {
        // 没找到帧头，清空缓冲区
        n10p_status.rx_index = 0;
        return 0;
    }
    
    // 检查数据长度
    if(n10p_status.rx_buffer[header_pos + 2] != N10P_FRAME_SIZE) {
        n10p_status.error_count++;
        n10p_status.rx_index = 0;
        return 0;
    }
    
    // CRC校验
    uint8_t calculated_crc = N10P_CalculateCRC(&n10p_status.rx_buffer[header_pos], N10P_FRAME_SIZE - 1);
    uint8_t received_crc = n10p_status.rx_buffer[header_pos + N10P_FRAME_SIZE - 1];
    
    if(calculated_crc != received_crc) {
        n10p_status.error_count++;
        n10p_status.rx_index = 0;
        return 0;
    }
    
    // 解析数据
    uint8_t* frame_data = &n10p_status.rx_buffer[header_pos];
    
    n10p_status.current_frame.speed = (frame_data[3] << 8) | frame_data[4];
    n10p_status.current_frame.start_angle = (frame_data[5] << 8) | frame_data[6];
    n10p_status.current_frame.end_angle = (frame_data[105] << 8) | frame_data[106];
    
    // 解析32个点的数据
    for(uint8_t i = 0; i < N10P_POINTS_PER_FRAME; i++) {
        uint16_t point_offset = 7 + (i * 3);
        n10p_status.current_frame.points[i].distance = (frame_data[point_offset] << 8) | frame_data[point_offset + 1];
        n10p_status.current_frame.points[i].intensity = frame_data[point_offset + 2];
        n10p_status.current_frame.points[i].angle = N10P_GetPointAngle(
            n10p_status.current_frame.start_angle,
            n10p_status.current_frame.end_angle,
            i
        );
    }
    
    n10p_status.current_frame.valid = 1;
    n10p_status.current_frame.timestamp = n10p_status.frame_count;
    n10p_status.frame_count++;
    n10p_status.frame_ready = 1;
    
    // 移除已处理的数据
    uint16_t remaining = n10p_status.rx_index - header_pos - N10P_FRAME_SIZE;
    if(remaining > 0) {
        memmove(n10p_status.rx_buffer, &n10p_status.rx_buffer[header_pos + N10P_FRAME_SIZE], remaining);
        n10p_status.rx_index = remaining;
    } else {
        n10p_status.rx_index = 0;
    }
    
    return 1;
}

// 修改N10P_ProcessData函数，使其能处理不完整的数据帧
void N10P_ProcessData(void)
{
    // 检查UART5是否有新数据
    uint16_t uart_length = UART5_GetReceiveLength();
    
    if(uart_length > 0 && UART5_IsReceiveFinished()) {
        n10p_total_attempts++;  // 有数据时才增加尝试计数
        
        // 复制数据到N10P缓冲区（累积模式）
        uint16_t copy_length = uart_length;
        if(n10p_status.rx_index + copy_length > sizeof(n10p_status.rx_buffer)) {
            // 缓冲区溢出，重置并重新开始
            n10p_status.rx_index = 0;
            copy_length = (uart_length > sizeof(n10p_status.rx_buffer)) ? sizeof(n10p_status.rx_buffer) : uart_length;
        }
        
        memcpy(&n10p_status.rx_buffer[n10p_status.rx_index], Uart5ReceiveBuf, copy_length);
        n10p_status.rx_index += copy_length;
        
        // 修改：尝试处理不完整的数据帧
        // 即使数据不足N10P_FRAME_SIZE，也尝试解析
        if(n10p_status.rx_index >= 10) { // 至少需要帧头和一些基本信息
            if(N10P_ParsePartialFrame()) {
                n10p_success_count++;  // 成功解析帧，增加成功计数
            }
        }
        
        UART5_ClearReceiveFlag();
    }
}

// 修改ParsePartialFrame函数，提高部分帧解析成功率
uint8_t N10P_ParsePartialFrame(void)
{
    // 查找帧头
    uint16_t header_pos = 0;
    uint8_t found_header = 0;
    
    // 增加健壮性：即使缓冲区较小也尝试找帧头
    for(uint16_t i = 0; i <= n10p_status.rx_index - 2; i++) { // 至少需要2字节帧头
        if(n10p_status.rx_buffer[i] == 0xA5 && n10p_status.rx_buffer[i+1] == 0x5A) {
            header_pos = i;
            found_header = 1;
            break;
        }
    }
    
    if(!found_header) {
        // 没找到帧头，但不立即清空缓冲区，只清除前一半数据
        // 这样可以保留可能包含部分帧头的后半部分数据
        if(n10p_status.rx_index > 10) {
            uint16_t half_size = n10p_status.rx_index / 2;
            memmove(n10p_status.rx_buffer, &n10p_status.rx_buffer[half_size], n10p_status.rx_index - half_size);
            n10p_status.rx_index -= half_size;
        }
        return 0;
    }
    
    // 检查是否至少有帧头+长度字段
    if(header_pos + 3 >= n10p_status.rx_index) {
        // 数据不足，保留缓冲区等待更多数据
        // 但将帧头移到缓冲区开始位置，为后续数据腾出空间
        if(header_pos > 0) {
            memmove(n10p_status.rx_buffer, &n10p_status.rx_buffer[header_pos], n10p_status.rx_index - header_pos);
            n10p_status.rx_index -= header_pos;
        }
        return 0;
    }
    
    // 获取帧长度
    uint8_t frame_len = n10p_status.rx_buffer[header_pos + 2];
    
    // 自适应帧长度：如果长度异常，尝试使用经验值
    if(frame_len < 10 || frame_len > 200) {
        // 测试常见的帧长度值
        uint8_t common_lengths[] = {108, 104, 112, 100};
        for(int i = 0; i < sizeof(common_lengths); i++) {
            // 尝试验证这个长度是否合理
            if(header_pos + common_lengths[i] <= n10p_status.rx_index) {
                // 计算简单校验和
                uint8_t sum = 0;
                for(int j = 0; j < common_lengths[i] - 1; j++) {
                    if(header_pos + j < n10p_status.rx_index) {
                        sum += n10p_status.rx_buffer[header_pos + j];
                    }
                }
                // 检查校验和是否匹配
                if(sum == n10p_status.rx_buffer[header_pos + common_lengths[i] - 1]) {
                    frame_len = common_lengths[i];
                    break;
                }
            }
        }
        
        // 如果仍然无法确定长度，使用默认值
        if(frame_len < 10 || frame_len > 200) {
            frame_len = 108; // 使用默认长度
        }
    }
    
    // 检查是否有完整帧
    if(header_pos + frame_len <= n10p_status.rx_index) {
        // 有完整帧，执行标准解析
        // 先验证校验和
        uint8_t crc_calc = 0;
        for(int i = header_pos; i < header_pos + frame_len - 1; i++) {
            crc_calc += n10p_status.rx_buffer[i];
        }
        uint8_t crc_recv = n10p_status.rx_buffer[header_pos + frame_len - 1];
        
        // 即使校验和不匹配也尝试解析，但记录错误
        if(crc_calc != crc_recv) {
            n10p_status.error_count++;
        }
        
        // 解析基本信息
        if(header_pos + 9 < n10p_status.rx_index) {
            uint16_t start_angle = (n10p_status.rx_buffer[header_pos+3] << 8) | n10p_status.rx_buffer[header_pos+4];
            uint16_t end_angle = (n10p_status.rx_buffer[header_pos+5] << 8) | n10p_status.rx_buffer[header_pos+6];
            uint16_t speed = (n10p_status.rx_buffer[header_pos+7] << 8) | n10p_status.rx_buffer[header_pos+8];
            
            // 验证角度值是否合理
            if(start_angle <= 36000 && end_angle <= 36000 && speed <= 1800) {
                // 更新当前帧信息
                n10p_status.current_frame.valid = 1;
                n10p_status.current_frame.start_angle = start_angle;
                n10p_status.current_frame.end_angle = end_angle;
                n10p_status.current_frame.speed = speed;
                n10p_status.current_frame.timestamp = GetSystemTick();
                
                // 计算可以解析的点数
                uint16_t max_points = (frame_len - 9) / 3; // 每点3字节
                if(max_points > N10P_POINTS_PER_FRAME) max_points = N10P_POINTS_PER_FRAME;
                
                // 解析点数据
                for(uint8_t i = 0; i < max_points; i++) {
                    uint16_t offset = header_pos + 9 + i * 3;
                    if(offset + 2 < n10p_status.rx_index) {
                        uint16_t distance = (n10p_status.rx_buffer[offset] << 8) | n10p_status.rx_buffer[offset+1];
                        uint8_t intensity = n10p_status.rx_buffer[offset+2];
                        float angle = N10P_GetPointAngle(start_angle, end_angle, i);
                        
                        // 过滤无效数据（距离太近或太远）
                        if(distance > 10 && distance < 10000) {
                            n10p_status.current_frame.points[i].distance = distance;
                            n10p_status.current_frame.points[i].intensity = intensity;
                            n10p_status.current_frame.points[i].angle = angle;
                        } else {
                            // 无效距离设为0
                            n10p_status.current_frame.points[i].distance = 0;
                            n10p_status.current_frame.points[i].intensity = 0;
                            n10p_status.current_frame.points[i].angle = angle;
                        }
                    }
                }
                
                // 清除处理过的数据
                if(header_pos + frame_len < n10p_status.rx_index) {
                    memmove(n10p_status.rx_buffer, 
                           &n10p_status.rx_buffer[header_pos + frame_len], 
                           n10p_status.rx_index - header_pos - frame_len);
                    n10p_status.rx_index -= (header_pos + frame_len);
                } else {
                    n10p_status.rx_index = 0;
                }
                
                n10p_status.frame_count++;
                return 1;
            }
        }
    }
    
    // 处理不完整帧 - 尝试解析可用部分
    // 检查是否至少有帧头+起始角度+结束角度+速度信息
    if(header_pos + 9 <= n10p_status.rx_index) {
        // 解析基本信息
        uint16_t start_angle = (n10p_status.rx_buffer[header_pos+3] << 8) | n10p_status.rx_buffer[header_pos+4];
        uint16_t end_angle = (n10p_status.rx_buffer[header_pos+5] << 8) | n10p_status.rx_buffer[header_pos+6];
        uint16_t speed = (n10p_status.rx_buffer[header_pos+7] << 8) | n10p_status.rx_buffer[header_pos+8];
        
        // 验证数据是否合理
        if(start_angle <= 36000 && end_angle <= 36000 && speed <= 1800) {
            // 更新当前帧信息
            n10p_status.current_frame.valid = 1;
            n10p_status.current_frame.start_angle = start_angle;
            n10p_status.current_frame.end_angle = end_angle;
            n10p_status.current_frame.speed = speed;
            n10p_status.current_frame.timestamp = GetSystemTick();
            
            // 计算可以解析的点数
            uint16_t available_points = (n10p_status.rx_index - header_pos - 9) / 3; // 每点3字节
            available_points = (available_points > N10P_POINTS_PER_FRAME) ? 
                                N10P_POINTS_PER_FRAME : available_points;
            
            // 解析可用的点数据
            for(uint8_t i = 0; i < available_points; i++) {
                uint16_t offset = header_pos + 9 + i * 3;
                if(offset + 2 < n10p_status.rx_index) {
                    uint16_t distance = (n10p_status.rx_buffer[offset] << 8) | n10p_status.rx_buffer[offset+1];
                    uint8_t intensity = n10p_status.rx_buffer[offset+2];
                    float angle = N10P_GetPointAngle(start_angle, end_angle, i);
                    
                    // 过滤无效数据
                    if(distance > 10 && distance < 10000) {
                        n10p_status.current_frame.points[i].distance = distance;
                        n10p_status.current_frame.points[i].intensity = intensity;
                        n10p_status.current_frame.points[i].angle = angle;
                    } else {
                        n10p_status.current_frame.points[i].distance = 0;
                        n10p_status.current_frame.points[i].intensity = 0;
                        n10p_status.current_frame.points[i].angle = angle;
                    }
                }
            }
            
            // 不清除缓冲区，等待完整帧
            return 1; // 返回成功，即使是部分解析
        } else {
            // 数据不合理，清除帧头
            memmove(n10p_status.rx_buffer, &n10p_status.rx_buffer[header_pos+2], 
                   n10p_status.rx_index - header_pos - 2);
            n10p_status.rx_index -= (header_pos + 2);
            n10p_status.error_count++;
        }
    } else {
        // 将帧头移到缓冲区开始位置，为后续数据腾出空间
        if(header_pos > 0) {
            memmove(n10p_status.rx_buffer, &n10p_status.rx_buffer[header_pos], 
                   n10p_status.rx_index - header_pos);
            n10p_status.rx_index -= header_pos;
        }
    }
    
    // 数据不足以解析基本信息，保留缓冲区等待更多数据
    return 0;
}

// 修改N10P_SendTestCommand函数，增加重试和多种指令
void N10P_SendTestCommand(void)
{
    static uint8_t test_step = 0;
    static uint8_t retry_count = 0;
    static uint32_t last_success_tick = 0;
    uint32_t current_tick = GetSystemTick();
    
    // 如果最近有成功解析帧，重置重试计数
    if(n10p_status.frame_count > 0 && 
       n10p_status.current_frame.timestamp > last_success_tick) {
        retry_count = 0;
        last_success_tick = n10p_status.current_frame.timestamp;
    }
    
    // 清除接收缓冲区（仅在重试多次时）
    if(retry_count > 3) {
        UART5_ClearReceiveFlag();
        n10p_status.rx_index = 0;
    }
    
    // 根据重试次数调整命令策略
    if(retry_count > 10) {
        // 多次重试失败，尝试重新初始化UART
        UART5_Init(460800);
        delay_ms(10);
        retry_count = 0;
        
        char buffer[50];
        sprintf(buffer, "UART5 RE-INIT @ %lu", current_tick);
        LCD_ShowString(20, 480-60, 300, 16, 16, (u8*)buffer);
    }
    
    switch(test_step) {
        case 0:
            // 发送停止扫描命令
            {
                uint8_t stop_cmd[] = {0xA5, 0x65, 0x00, 0x0A};
                UART5_SendBuffer(stop_cmd, sizeof(stop_cmd));
            }
            break;
        case 1:
            // 发送启动扫描命令 
            {
                uint8_t start_cmd[] = {0xA5, 0x60, 0x00, 0x05};
                UART5_SendBuffer(start_cmd, sizeof(start_cmd));
            }
            break;
        case 2:
            // 发送获取设备信息命令
            {
                uint8_t info_cmd[] = {0xA5, 0x90, 0x00, 0x35};
                UART5_SendBuffer(info_cmd, sizeof(info_cmd));
            }
            break;
        case 3:
            // 发送软复位命令
            {
                uint8_t reset_cmd[] = {0xA5, 0x40, 0x00, 0xE5};
                UART5_SendBuffer(reset_cmd, sizeof(reset_cmd));
                delay_ms(50); // 复位后等待一段时间
            }
            break;
        case 4:
            // 发送启动扫描命令（再次尝试）
            {
                uint8_t start_cmd2[] = {0xA5, 0x60, 0x00, 0x05};
                UART5_SendBuffer(start_cmd2, sizeof(start_cmd2));
            }
            break;
        case 5:
            // 发送设置扫描频率命令
            {
                uint8_t freq_cmd[] = {0xA5, 0x0B, 0x01, 0x00, 0xB1};
                UART5_SendBuffer(freq_cmd, sizeof(freq_cmd));
            }
            break;
        default:
            // 尝试其他波特率（多次重试后）
            if(retry_count > 5) {
                static uint32_t baudrates[] = {460800, 115200, 230400, 921600};
                static uint8_t baud_index = 0;
                
                baud_index = (baud_index + 1) % 4;
                UART5_Init(baudrates[baud_index]);
                
                char buffer[50];
                sprintf(buffer, "Try baudrate: %lu", baudrates[baud_index]);
                LCD_ShowString(20, 480-60, 300, 16, 16, (u8*)buffer);
                
                // 发送唤醒命令
                delay_ms(10);
                uint8_t wake_cmd[] = {0xA5, 0x60, 0x00, 0x05};
                UART5_SendBuffer(wake_cmd, sizeof(wake_cmd));
            } else {
                // 普通重试启动命令
                uint8_t start_cmd[] = {0xA5, 0x60, 0x00, 0x05};
                UART5_SendBuffer(start_cmd, sizeof(start_cmd));
            }
            break;
    }
    
    test_step = (test_step + 1) % 7;  // 循环7个步骤
    retry_count++;
}

// I2C诊断函数实现 - 修复版本
int8_t I2C1_ScanDevice(uint8_t addr)
{
    // 确保I2C总线空闲
    if(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        // 如果总线忙，尝试复位
        I2C_GenerateSTOP(I2C1, ENABLE);
        uint32_t reset_timeout = 1000;
        while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && reset_timeout--);
        if(reset_timeout == 0) return -1;
    }
    
    // 发送START条件
    I2C_GenerateSTART(I2C1, ENABLE);
    
    uint32_t timeout = 1000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && timeout--);
    if(timeout == 0) {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return -1;
    }
    
    // 发送设备地址
    I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Transmitter);
    
    timeout = 1000;
    // 等待ACK或NACK
    while(timeout--) {
        if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
            // 收到ACK，设备存在
            I2C_GenerateSTOP(I2C1, ENABLE);
            return 0;
        }
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
            // 收到NACK，设备不存在
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            I2C_GenerateSTOP(I2C1, ENABLE);
            return -1;
        }
    }
    
    // 超时，设备不存在
    I2C_GenerateSTOP(I2C1, ENABLE);
    return -1;
}

uint8_t I2C1_ReadDeviceID(uint8_t addr, uint8_t reg)
{
    uint8_t data = 0xFF;
    
    I2C_GenerateSTART(I2C1, ENABLE);
    uint32_t timeout = 1000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && timeout--);
    if(timeout == 0) return 0xFF;
    
    I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Transmitter);
    timeout = 1000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && timeout--);
    if(timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0xFF; }
    
    I2C_SendData(I2C1, reg);
    timeout = 1000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) && timeout--);
    if(timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0xFF; }
    
    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = 1000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) && timeout--);
    if(timeout == 0) return 0xFF;
    
    I2C_Send7bitAddress(I2C1, addr << 1, I2C_Direction_Receiver);
    timeout = 1000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && timeout--);
    if(timeout == 0) { I2C_GenerateSTOP(I2C1, ENABLE); return 0xFF; }
    
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
    
    timeout = 1000;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) && timeout--);
    if(timeout > 0) {
        data = I2C_ReceiveData(I2C1);
    }
    
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return data;
}

// UART5状态监控函数
void UART5_StatusMonitor(void)
{
    char buffer[100];
    
    // 检查GPIO引脚电平状态
    uint8_t pc12_level = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12);  // TX引脚
    uint8_t pd2_level = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2);    // RX引脚
    
    sprintf(buffer, "GPIO Level: PC12=%d PD2=%d", pc12_level, pd2_level);
    LCD_ShowString(420, 420, 300, 16, 12, (u8*)buffer);
    
    // 检查UART5状态寄存器各位
    uint16_t sr = UART5->SR;
    sprintf(buffer, "SR Bits: TC=%d TXE=%d RXNE=%d ORE=%d", 
           (sr & USART_SR_TC) ? 1 : 0,
           (sr & USART_SR_TXE) ? 1 : 0, 
           (sr & USART_SR_RXNE) ? 1 : 0,
           (sr & USART_SR_ORE) ? 1 : 0);
    LCD_ShowString(420, 435, 300, 16, 12, (u8*)buffer);
    
    // 显示中断向量表状态
    uint32_t *nvic_iser = (uint32_t*)0xE000E100;  // NVIC中断使能寄存器
    uint32_t uart5_int_enabled = nvic_iser[1] & (1 << (UART5_IRQn - 32));
    
    sprintf(buffer, "NVIC: UART5_INT=%s IRQn=%d", 
           uart5_int_enabled ? "EN" : "DIS", UART5_IRQn);
    LCD_ShowString(420, 450, 300, 16, 12, (u8*)buffer);
}

// === 添加模式定义和SLAM相关结构 ===
#define MODE_DEBUG    0  // 调试模式
#define MODE_SLAM     1  // SLAM模式

// SLAM显示模式
#define SLAM_DISPLAY_NORMAL   0  // 正常模式：动态更新地图
#define SLAM_DISPLAY_PERSIST  1  // 持续模式：点云保持不刷新
#define SLAM_DISPLAY_WALL     2  // 墙体模式：突出显示墙体

// 当前工作模式
uint8_t current_mode = MODE_DEBUG;
uint8_t slam_display_mode = SLAM_DISPLAY_NORMAL;

// 地图元素类型
#define MAP_EMPTY     0    // 空白区域
#define MAP_OBSTACLE  1    // 障碍物
#define MAP_WALL      2    // 墙体
#define MAP_UNCERTAIN 3    // 不确定区域

// SLAM数据结构
typedef struct {
    uint8_t map[100][100];        // 10m x 10m地图，每个单元格10cm，值表示类型和置信度
    int16_t robot_x;              // 机器人X坐标 (单位: cm)
    int16_t robot_y;              // 机器人Y坐标 (单位: cm)
    int16_t robot_orientation;    // 机器人朝向 (单位: 0.1度, 0-3600)
    float velocity_x;             // X方向速度 (单位: cm/s)
    float velocity_y;             // Y方向速度 (单位: cm/s)
    float angular_velocity;       // 角速度 (单位: 度/s)
    uint32_t last_update_time;    // 上次更新时间
    uint16_t persistence_counter[100][100]; // 用于追踪点的持久性
    uint8_t movement_mode;        // 移动模式: 0=静止模式, 1=运动模式
    uint8_t touch_active;         // 触摸状态: 0=无触摸, 1=触摸有效
    float touch_x;                // 触摸点X坐标
    float touch_y;                // 触摸点Y坐标
} SLAM_Status_t;

SLAM_Status_t slam_status = {0};

// 清除地图数据
void SLAM_ClearMap(void)
{
    memset(slam_status.map, 0, sizeof(slam_status.map));
    memset(slam_status.persistence_counter, 0, sizeof(slam_status.persistence_counter));
    slam_status.robot_x = 50 * 10; // 初始位置在地图中央 (5m, 5m)
    slam_status.robot_y = 50 * 10;
    slam_status.robot_orientation = 0; // 初始朝向0度
    slam_status.velocity_x = 0;
    slam_status.velocity_y = 0;
    slam_status.angular_velocity = 0;
    slam_status.last_update_time = GetSystemTick();
}

// 更新机器人位置 (基于加速度计数据)
void SLAM_UpdatePosition(int16_t accel_x, int16_t accel_y, int16_t accel_z)
{
    uint32_t current_time = GetSystemTick();
    float dt = (current_time - slam_status.last_update_time) * 0.2f;
    if (dt > 0.5f) dt = 0.2f;
  
    // 加速度转换与死区
    float ax = accel_x * 0.032f * 9.81f;
    float ay = accel_y * 0.032f * 9.81f;
    float az = accel_z * 0.032f * 9.81f;
    const float deadzone = 0.5f;
    if (fabsf(ax) < deadzone) ax = 0;
    if (fabsf(ay) < deadzone) ay = 0;
    if (fabsf(az) < deadzone) az = 0;
  
    // 运动/静止检测
    static uint8_t static_counter = 0;
    static float prev_mag = 0;
    static bool is_static = true;
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    if (fabsf(mag - prev_mag) < 0.3f) {
        if (static_counter < 10) static_counter++;
        if (static_counter >= 5) is_static = true;
    } else {
        if (static_counter) static_counter--;
        if (static_counter == 0) is_static = false;
    }
    prev_mag = mag;
  
    // 静止时快速衰减，不积分速度
    if (is_static) {
        const float decay = 0.5f;            // 更强衰减
        slam_status.velocity_x *= decay;
        slam_status.velocity_y *= decay;
        if (fabsf(slam_status.velocity_x) < 1.0f) slam_status.velocity_x = 0;
        if (fabsf(slam_status.velocity_y) < 1.0f) slam_status.velocity_y = 0;
    }
    // 运动时施加更大阻尼并积分加速度
    else {
        // 根据移动模式决定是否处理加速度
        if (slam_status.movement_mode == 1) { // 只有在运动模式下才处理加速度
            // 基础阻尼 + 固定额外衰减
            const float base_damping = 0.80f;    // 提高基础阻尼
            const float extra_damp  = 0.05f;    // 恒定衰减
            float vmag = sqrtf(slam_status.velocity_x*slam_status.velocity_x +
                              slam_status.velocity_y*slam_status.velocity_y);
            float damping = base_damping + extra_damp * (vmag/100.0f);
            if (damping > 0.95f) damping = 0.95f; // 限制最大阻尼
    
            slam_status.velocity_x *= damping;
            slam_status.velocity_y *= damping;
    
            // 积分加速度
            const float accel_sens = 0.6f;       // 进一步降低灵敏度
            slam_status.velocity_x += ax * dt * 100.0f * accel_sens;
            slam_status.velocity_y += ay * dt * 100.0f * accel_sens;
        } else {
            // 静止模式下，持续衰减速度但不积分加速度
            const float decay = 0.7f;
            slam_status.velocity_x *= decay;
            slam_status.velocity_y *= decay;
            if (fabsf(slam_status.velocity_x) < 1.0f) slam_status.velocity_x = 0;
            if (fabsf(slam_status.velocity_y) < 1.0f) slam_status.velocity_y = 0;
        }
    }
  
    // 二次速度限幅
    const float max_v = 150.0f;
    if (slam_status.velocity_x >  max_v) slam_status.velocity_x =  max_v;
    if (slam_status.velocity_x < -max_v) slam_status.velocity_x = -max_v;
    if (slam_status.velocity_y >  max_v) slam_status.velocity_y =  max_v;
    if (slam_status.velocity_y < -max_v) slam_status.velocity_y = -max_v;
  
    // 转换到全局坐标并更新位置
    float ang_rad = slam_status.robot_orientation * 0.1f * 3.14159f / 180.0f;
    float vx = slam_status.velocity_x * cosf(ang_rad)
             - slam_status.velocity_y * sinf(ang_rad);
    float vy = slam_status.velocity_x * sinf(ang_rad)
             + slam_status.velocity_y * cosf(ang_rad);
    slam_status.robot_x += vx * dt;
    slam_status.robot_y += vy * dt;
  
    // 地图范围约束
    if (slam_status.robot_x <   0) slam_status.robot_x =   0;
    if (slam_status.robot_x > 999) slam_status.robot_x = 999;
    if (slam_status.robot_y <   0) slam_status.robot_y =   0;
    if (slam_status.robot_y > 999) slam_status.robot_y = 999;
  
    // 角速度—静止更强衰减，运动基础阻尼
    float ang_acc = az * 0.01f;
    if (fabsf(ang_acc) < 0.02f || is_static) ang_acc = 0;
    const float ang_base = 0.85f;
    slam_status.angular_velocity = slam_status.angular_velocity * ang_base
                                 + ang_acc * (1.0f - ang_base);
    if (is_static) {
        slam_status.angular_velocity *= 0.5f;
        if (fabsf(slam_status.angular_velocity) < 0.2f)
            slam_status.angular_velocity = 0;
    }
    slam_status.robot_orientation += slam_status.angular_velocity * dt * 10.0f;
  
    // 朝向约束
    while (slam_status.robot_orientation <    0) slam_status.robot_orientation += 3600;
    while (slam_status.robot_orientation >= 3600) slam_status.robot_orientation -= 3600;
  
    slam_status.last_update_time = current_time;
  
    // 静止/运动模式指示灯
    if (slam_status.movement_mode == 0) {
        LED3_ON;  // 静止模式点亮LED3
        LED4_OFF;
    } else {
        LED3_OFF;
        LED4_ON;  // 运动模式点亮LED4
    }
}

// 检测是否为墙体
bool SLAM_IsWall(int16_t x, int16_t y, int radius)
{
    // 检查周围点是否有连续的障碍物，表明可能是墙
    int count = 0;
    int max_line = 0;
    int current_line = 0;
    
    // 水平检测
    for(int i = -radius; i <= radius; i++) {
        int check_x = x + i;
        if(check_x >= 0 && check_x < 100 && 
           slam_status.map[y][check_x] == MAP_OBSTACLE) {
            current_line++;
            if(current_line > max_line) max_line = current_line;
        } else {
            current_line = 0;
        }
        if(check_x >= 0 && check_x < 100 && slam_status.map[y][check_x] == MAP_OBSTACLE) count++;
    }
    
    // 垂直检测
    current_line = 0;
    for(int j = -radius; j <= radius; j++) {
        int check_y = y + j;
        if(check_y >= 0 && check_y < 100 && 
           slam_status.map[check_y][x] == MAP_OBSTACLE) {
            current_line++;
            if(current_line > max_line) max_line = current_line;
        } else {
            current_line = 0;
        }
        if(check_y >= 0 && check_y < 100 && slam_status.map[check_y][x] == MAP_OBSTACLE) count++;
    }
    
    // 如果连续的点超过3个，认为是墙体
    return (max_line >= 3 || count >= radius);
}

// 更新地图 (基于雷达数据)
void SLAM_UpdateMap(void)
{
    if(!n10p_status.current_frame.valid) return;
    
    if(slam_display_mode == SLAM_DISPLAY_PERSIST) {
        // 持续模式下不更新地图，只更新机器人位置
        return;
    }
    
    // 清除机器人位置附近的地图 (动态更新)
    int16_t robot_map_x = slam_status.robot_x / 10; // 转换为地图单元格
    int16_t robot_map_y = slam_status.robot_y / 10;
    
    // 只在正常模式下清除附近区域
    if(slam_display_mode == SLAM_DISPLAY_NORMAL) {
        for(int y = -3; y <= 3; y++) {
            for(int x = -3; x <= 3; x++) {
                int map_x = robot_map_x + x;
                int map_y = robot_map_y + y;
                if(map_x >= 0 && map_x < 100 && map_y >= 0 && map_y < 100) {
                    // 减少持久性计数器，不立即清除点
                    if(slam_status.persistence_counter[map_y][map_x] > 0) {
                        slam_status.persistence_counter[map_y][map_x]--;
                    }
                    
                    // 如果计数器降至0，清除此点
                    if(slam_status.persistence_counter[map_y][map_x] == 0) {
                        slam_status.map[map_y][map_x] = MAP_EMPTY;
                    }
                }
            }
        }
    }
    
    // 使用雷达数据更新地图
    float robot_angle_rad = slam_status.robot_orientation * 0.1f * 3.14159f / 180.0f;
    
    for(int i = 0; i < N10P_POINTS_PER_FRAME; i++) {
        if(n10p_status.current_frame.points[i].distance > 0 && 
           n10p_status.current_frame.points[i].distance < 5000) { // 过滤5米以外的点
            // 将雷达点从极坐标转换为笛卡尔坐标
            float point_angle_rad = n10p_status.current_frame.points[i].angle * 3.14159f / 180.0f;
            float point_distance_m = n10p_status.current_frame.points[i].distance / 1000.0f; // 毫米转米
            
            // 计算点相对于机器人的坐标 (米)
            float point_rel_x = point_distance_m * cosf(point_angle_rad);
            float point_rel_y = point_distance_m * sinf(point_angle_rad);
            
            // 旋转到全局坐标系
            float point_global_x = point_rel_x * cosf(robot_angle_rad) - point_rel_y * sinf(robot_angle_rad);
            float point_global_y = point_rel_x * sinf(robot_angle_rad) + point_rel_y * cosf(robot_angle_rad);
            
            // 转换为厘米并加上机器人位置
            int16_t point_map_x = (slam_status.robot_x + point_global_x * 100) / 10;
            int16_t point_map_y = (slam_status.robot_y + point_global_y * 100) / 10;
            
            // 更新地图 (检查边界)
            if(point_map_x >= 0 && point_map_x < 100 && point_map_y >= 0 && point_map_y < 100) {
                // 增加持久性计数器，最大为20
                if(slam_status.persistence_counter[point_map_y][point_map_x] < 20) {
                    slam_status.persistence_counter[point_map_y][point_map_x]++;
                }
                
                // 根据持久性设置障碍物
                if(slam_status.persistence_counter[point_map_y][point_map_x] > 5) {
                    slam_status.map[point_map_y][point_map_x] = MAP_OBSTACLE;
                } else {
                    slam_status.map[point_map_y][point_map_x] = MAP_UNCERTAIN;
                }
            }
        }
    }
    
    // 在墙体检测模式下进行墙体识别
    if(slam_display_mode == SLAM_DISPLAY_WALL) {
        // 对整个地图进行墙体检测
        for(int y = 1; y < 99; y++) {
            for(int x = 1; x < 99; x++) {
                if(slam_status.map[y][x] == MAP_OBSTACLE) {
                    // 检查是否是墙体
                    if(SLAM_IsWall(x, y, 3)) {
                        slam_status.map[y][x] = MAP_WALL;
                    }
                }
            }
        }
    }
}

// 绘制SLAM地图
void SLAM_DrawMap(void)
{
    // 清除显示区域
    LCD_Fill(10, 120, 790, 450, 0x0000);
    
    // 绘制地图边框
    LCD_Fill(10, 120, 510, 122, 0x07FF); // 上边框
    LCD_Fill(10, 120, 12, 420, 0x07FF);  // 左边框
    LCD_Fill(10, 420, 510, 422, 0x07FF); // 下边框
    LCD_Fill(510, 120, 512, 422, 0x07FF); // 右边框
    
    // 定义不同元素的颜色
    // 黑色背景，不需要单独定义
    uint16_t color_obstacle = 0xFFFF;  // 白色
    uint16_t color_wall = 0xFD20;      // 橙色
    uint16_t color_uncertain = 0x7BEF;  // 灰色
    uint16_t color_robot = 0xF800;     // 红色
    uint16_t color_heading = 0x001F;   // 蓝色
    
    // 计算每个单元格的显示大小
    const int cell_width = 5;
    const int cell_height = 3;
    
    // 显示地图标题和当前模式
    char buffer[100];
    switch(slam_display_mode) {
        case SLAM_DISPLAY_NORMAL:
            sprintf(buffer, "SLAM Map - Dynamic Mode");
            break;
        case SLAM_DISPLAY_PERSIST:
            sprintf(buffer, "SLAM Map - Persistent Mode");
            break;
        case SLAM_DISPLAY_WALL:
            sprintf(buffer, "SLAM Map - Wall Detection Mode");
            break;
    }
    LCD_ShowString(15, 125, 300, 16, 16, (u8*)buffer);
    
    // 每个地图单元格大小为5x3像素
    for(int y = 0; y < 100; y++) {
        for(int x = 0; x < 100; x++) {
            // 跳过机器人位置附近的绘制，留待后面单独绘制
            int16_t robot_map_x = slam_status.robot_x / 10;
            int16_t robot_map_y = slam_status.robot_y / 10;
            if(abs(x - robot_map_x) <= 1 && abs(y - robot_map_y) <= 1) {
                continue;
            }
            
            // 根据地图内容选择颜色
            uint16_t color;
            switch(slam_status.map[y][x]) {
                case MAP_OBSTACLE:
                    color = color_obstacle;
                    break;
                case MAP_WALL:
                    color = color_wall;
                    break;
                case MAP_UNCERTAIN:
                    color = color_uncertain;
                    break;
                default:
                    continue; // 空白区域不绘制
            }
            
            // 计算显示坐标
            int16_t disp_x = 12 + x * cell_width;
            int16_t disp_y = 145 + y * cell_height;
            
            // 绘制单元格
            LCD_Fill(disp_x, disp_y, disp_x + cell_width - 1, disp_y + cell_height - 1, color);
        }
    }
    
    // 绘制机器人位置 (红色圆点)
    int16_t robot_disp_x = 12 + (slam_status.robot_x / 10) * cell_width + cell_width/2;
    int16_t robot_disp_y = 145 + (slam_status.robot_y / 10) * cell_height + cell_height/2;
    
    // 使用更大、更明显的标记
    LCD_Fill(robot_disp_x-4, robot_disp_y-4, robot_disp_x+4, robot_disp_y+4, color_robot);
    
    // 绘制机器人朝向 (蓝色线)
    float angle_rad = slam_status.robot_orientation * 0.1f * 3.14159f / 180.0f;
    int16_t dir_x = robot_disp_x + 15 * cosf(angle_rad);
    int16_t dir_y = robot_disp_y + 15 * sinf(angle_rad);
    
    // 简单线绘制算法 (Bresenham算法)
    int16_t dx = abs(dir_x - robot_disp_x);
    int16_t dy = abs(dir_y - robot_disp_y);
    int16_t sx = (robot_disp_x < dir_x) ? 1 : -1;
    int16_t sy = (robot_disp_y < dir_y) ? 1 : -1;
    int16_t err = dx - dy;
    int16_t e2;
    
    int16_t x0 = robot_disp_x;
    int16_t y0 = robot_disp_y;
    
    while(1) {
        LCD_Fill(x0, y0, x0+1, y0+1, color_heading);
        if(x0 == dir_x && y0 == dir_y) break;
        e2 = 2 * err;
        if(e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if(e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    // 显示状态信息
    sprintf(buffer, "Position: %d,%d cm", 
           slam_status.robot_x/10, slam_status.robot_y/10);
    LCD_ShowString(520, 130, 260, 16, 16, (u8*)buffer);
    
    sprintf(buffer, "Angle: %d.%d deg", 
           slam_status.robot_orientation/10, slam_status.robot_orientation%10);
    LCD_ShowString(520, 150, 260, 16, 16, (u8*)buffer);
    
    // 显示速度信息
    sprintf(buffer, "Velocity: %.1f,%.1f cm/s", 
           slam_status.velocity_x, slam_status.velocity_y);
    LCD_ShowString(520, 170, 260, 16, 16, (u8*)buffer);
    
    sprintf(buffer, "Angular: %.1f deg/s", 
           slam_status.angular_velocity);
    LCD_ShowString(520, 190, 260, 16, 16, (u8*)buffer);
    
    // 显示传感器数据
    sprintf(buffer, "ADXL: X=%d Y=%d Z=%d", adxl_x_g, adxl_y_g, adxl_z_g);
    LCD_ShowString(520, 210, 260, 16, 16, (u8*)buffer);
    
    // 显示雷达状态
    sprintf(buffer, "Lidar: %lu pts, %lu err", n10p_status.frame_count, n10p_status.error_count);
    LCD_ShowString(520, 230, 260, 16, 16, (u8*)buffer);
    
    // 显示移动模式状态
    if (slam_status.movement_mode == 0) {
        sprintf(buffer, "Mode: Static (KEY3 to enable move)");
    } else {
        sprintf(buffer, "Mode: Movement (KEY2 to lock)");
    }
    LCD_ShowString(520, 250, 260, 16, 16, (u8*)buffer);
    
    // 显示触摸状态
    if (slam_status.touch_active) {
        sprintf(buffer, "Touch: %.0f, %.0f", slam_status.touch_x, slam_status.touch_y);
        LCD_ShowString(520, 270, 260, 16, 16, (u8*)buffer);
    }
    
    // 显示地图统计信息
    int obstacles = 0, walls = 0, uncertain = 0;
    for(int y = 0; y < 100; y++) {
        for(int x = 0; x < 100; x++) {
            if(slam_status.map[y][x] == MAP_OBSTACLE) obstacles++;
            else if(slam_status.map[y][x] == MAP_WALL) walls++;
            else if(slam_status.map[y][x] == MAP_UNCERTAIN) uncertain++;
        }
    }
    
    sprintf(buffer, "Map: %d obstacles, %d walls", obstacles, walls);
    LCD_ShowString(520, 290, 260, 16, 16, (u8*)buffer);
    
    // 显示操作提示
    LCD_ShowString(520, 320, 260, 16, 16, (u8*)"S1: Reset Map");
    LCD_ShowString(520, 340, 260, 16, 16, (u8*)"S2: Change Mode");
    LCD_ShowString(520, 360, 260, 16, 16, (u8*)"S3: Change Display");
    
    // 图例说明
    LCD_ShowString(520, 390, 260, 16, 16, (u8*)"Legend:");
    LCD_Fill(520, 410, 530, 420, color_obstacle);
    LCD_ShowString(535, 410, 80, 16, 16, (u8*)"Obstacle");
    LCD_Fill(520, 430, 530, 440, color_wall);
    LCD_ShowString(535, 430, 80, 16, 16, (u8*)"Wall");
    LCD_Fill(520, 450, 530, 460, color_uncertain);
    LCD_ShowString(535, 450, 80, 16, 16, (u8*)"Uncertain");
    LCD_Fill(640, 390, 650, 400, color_robot);
    LCD_ShowString(655, 390, 80, 16, 16, (u8*)"Robot");
    LCD_Fill(640, 410, 650, 420, color_heading);
    LCD_ShowString(655, 410, 100, 16, 16, (u8*)"Heading");
}

// 显示DEBUG模式界面
// === DEBUG 模式界面绘制函数 ===
void DEBUG_DrawInterface(void)
{
    // 清除调试模式显示区域
    LCD_Fill(0, 120, 800, 480, 0x0000);

    // 绘制 I2C 设备诊断列表 (地址 0x08 - 0x0F 示例)
    for(uint8_t addr = 0x08; addr <= 0x0F; addr++) {
        if(I2C1_ScanDevice(addr) == 0) {
            I2C1_DiagnoseDevice(addr);
        }
    }

    // 绘制 UART5 硬件诊断信息
    UART5_HardwareDiagnose();
}


// 检测触摸状态并更新触摸坐标
void SLAM_CheckTouch(void)
{
    if (tp_dev.sta & TP_PRES_DOWN) {  // 触摸按下
        slam_status.touch_active = 1;
        slam_status.touch_x = tp_dev.x[0];
        slam_status.touch_y = tp_dev.y[0];
        
        // 如果处于运动模式，通过触摸设置机器人位置
        if (slam_status.movement_mode == 1) {
            // 将触摸坐标转换为机器人坐标
            slam_status.robot_x = tp_dev.x[0] * 1000.0f / 800.0f;  // 屏幕宽度800像素
            slam_status.robot_y = tp_dev.y[0] * 1000.0f / 480.0f;  // 屏幕高度480像素
            
            // 停止当前速度
            slam_status.velocity_x = 0;
            slam_status.velocity_y = 0;
            slam_status.angular_velocity = 0;
        }
    } else {
        slam_status.touch_active = 0;
    }
}

int main(void)
{
	delay_init();
    
    LCD_Init();
    
    // 触摸屏初始化
    TP_Init();
    
    // 初始化LED用于状态指示
    LED_Init();
    LED1_OFF; LED2_OFF; LED3_OFF; LED4_OFF;
    
    // 初始化按键
    KEY_Init();
    
    // 初始化TIM3用于UART5超时管理
    TIM3_Init(); 
    
    // 初始化ADXL345
    ADXL345_Init();
    
    // 初始化UART5和N10P激光雷达通信
    UART5_Init(460800);
    
    // 初始化SLAM状态
    SLAM_ClearMap();
    
    // 设置背景色
    uint16_t black_color = 0x0000; // 黑色
    uint16_t green_color = 0x07E0; // 绿色
    uint16_t blue_color = 0x001F;  // 蓝色
    uint16_t red_color = 0xF800;   // 红色
    
    LCD_Clear(black_color);
    
    int counter = 0;
    
    while(1)
    {
		// 处理按键
		unsigned char key = KeyScan();
		// KEY1：切换 DEBUG ↔ SLAM 模式
		if (key == 1) {
			current_mode = (current_mode == MODE_DEBUG) ? MODE_SLAM : MODE_DEBUG;
			if (current_mode == MODE_SLAM) {
				SLAM_ClearMap();
				LCD_ShowString(20, 460, 300, 16, 16, (u8*)"进入 SLAM 模式");
			} else {
				// 切回 DEBUG 时可以清空雷达或者做别的
				UART5_ClearReceiveFlag();
				n10p_status.rx_index   = 0;
				n10p_status.frame_count = 0;
				n10p_status.error_count = 0;
				LCD_ShowString(20, 460, 300, 16, 16, (u8*)"进入 DEBUG 模式");
			}
			// 间隔一下防止连按
			delay_ms(200);
		}

		// 如果当前是 SLAM 模式，才响应 KEY2/KEY3 锁定/解锁
		if (current_mode == MODE_SLAM) {
			if (key == 2) {
				// 按键2：设置为静止模式，但保留原有的模式切换功能
				if(current_mode == MODE_DEBUG) {
					// 切换到SLAM模式
					current_mode = MODE_SLAM;
					// 切换标题显示
					LCD_Fill(0, 25, 800, 50, blue_color);
					LCD_ShowString(20, 28, 400, 16, 16, (u8*)"SLAM Mode - Lidar + ADXL345 Fusion");
					LED2_ON;
					delay_ms(200);
					LED2_OFF;
				} else {
					// 在SLAM模式下，设置静止模式
					slam_status.movement_mode = 0;
					slam_status.velocity_x = 0;
					slam_status.velocity_y = 0;
					slam_status.angular_velocity = 0;
					LCD_ShowString(20, 460, 800, 16, 16, (u8*)"Static Mode - Position Locked");
					LED3_ON;
					LED4_OFF;
					delay_ms(200);
				}
			} else if (key == 3) {
				if(current_mode == MODE_DEBUG) {
					// 在DEBUG模式下按3也切换到SLAM模式
					current_mode = MODE_SLAM;
					// 切换标题显示
					LCD_Fill(0, 25, 800, 50, blue_color);
					LCD_ShowString(20, 28, 400, 16, 16, (u8*)"SLAM Mode - Lidar + ADXL345 Fusion");
					LED2_ON;
					delay_ms(200);
					LED2_OFF;
				} else {
					// 在SLAM模式下，切换显示模式或设置运动模式
					if(slam_status.movement_mode == 0) {
						// 设置为运动模式
						slam_status.movement_mode = 1;
						LCD_ShowString(20, 460, 800, 16, 16, (u8*)"Movement Mode - Use touch to set position");
						LED3_OFF;
						LED4_ON;
					} else {
						// 已经是运动模式，则切换显示模式
						slam_display_mode = (slam_display_mode + 1) % 3;
					}
					delay_ms(200);
				}
			}
		}

        
        // 检测触摸状态
        SLAM_CheckTouch();
        
        // === 处理N10P雷达数据 ===
        N10P_ProcessData();
        
        // === ADXL345加速度计读取 ===
        short adxl_x, adxl_y, adxl_z;
        ADXL345_RD_XYZ(&adxl_x, &adxl_y, &adxl_z);
        adxl_x_g = adxl_x;  // 保存到全局变量
        adxl_y_g = adxl_y;
        adxl_z_g = adxl_z;
        
        // 根据模式显示不同界面
        if(current_mode == MODE_DEBUG) {
            // DEBUG模式 - 保留原有显示
            // ... existing debug display code ...
            // 清除数据显示区域
            LCD_Fill(0, 120, 800, 480, 0x0000);
            
            // === N10P激光雷达详细调试 - 扩展到整个屏幕 ===
            LCD_Fill(10, 130, 790, 320, green_color);
            LCD_Fill(15, 135, 785, 315, 0x0000);
            
            LCD_ShowString(20, 140, 300, 16, 16, (u8*)"N10P UART5 Interrupt Debug");
            
            // 检查UART5原始数据和中断处理状态
            uint16_t uart_length = UART5_GetReceiveLength();
            uint8_t uart_finished = UART5_IsReceiveFinished();
            
            char buffer[100];
            sprintf(buffer, "Length: %d, Finished: %d", uart_length, uart_finished);
            LCD_ShowString(20, 160, 300, 16, 16, (u8*)buffer);
            
            // 显示UART5寄存器状态用于调试中断
            sprintf(buffer, "UART5 SR: 0x%04X, CR1: 0x%04X", UART5->SR, UART5->CR1);
            LCD_ShowString(20, 180, 300, 16, 16, (u8*)buffer);
            
            sprintf(buffer, "ParsedFrames: %lu, Errors: %lu", n10p_status.frame_count, n10p_status.error_count);
            LCD_ShowString(20, 200, 300, 16, 16, (u8*)buffer);
            
            // 显示雷达成功率统计
            uint32_t success_rate = (n10p_total_attempts > 0) ? (n10p_success_count * 100 / n10p_total_attempts) : 0;
            sprintf(buffer, "N10P Success: %lu/%lu (%lu%%)", n10p_success_count, n10p_total_attempts, success_rate);
            LCD_ShowString(20, 220, 300, 16, 16, (u8*)buffer);
            
            // ... rest of the debug display code ...
            
            // 继续原有的DEBUG显示逻辑
        } else {
            // SLAM模式 - 显示地图和融合数据
            // 更新SLAM状态
            SLAM_UpdatePosition(adxl_x, adxl_y, adxl_z);
            SLAM_UpdateMap();
            
            // 绘制SLAM地图
            SLAM_DrawMap();
        }
        
        // 更积极的命令重试策略
        // 增加测试命令发送频率，当成功率低时
        if(n10p_total_attempts > 10) {
            uint32_t success_rate = (n10p_success_count * 100 / n10p_total_attempts);
            if(success_rate < 50) {
                // 成功率低于50%，更频繁地发送测试命令
                if(counter % 15 == 0) {
                    N10P_SendTestCommand();
                }
            } else {
                // 成功率高于50%，保持正常频率
                if(counter % 30 == 0) {
                    N10P_SendTestCommand();
                }
            }
        } else {
            // 初始阶段，保持较高频率
            if(counter % 20 == 0) {
                N10P_SendTestCommand();
            }
        }
        
        // 在屏幕底部添加解析后的雷达数据显示区域
        if(n10p_status.current_frame.valid && current_mode == MODE_DEBUG) {
            // 只在DEBUG模式下显示这些数据
            // 显示雷达基本信息
            char buffer[100];
            sprintf(buffer, "Angle: %d.%02d - %d.%02d deg, Speed: %d rpm", 
                    n10p_status.current_frame.start_angle/100, n10p_status.current_frame.start_angle%100,
                    n10p_status.current_frame.end_angle/100, n10p_status.current_frame.end_angle%100,
                    n10p_status.current_frame.speed);
            LCD_ShowString(20, 480-40, 500, 16, 16, (u8*)buffer);
            
            // 显示前几个点的数据，同时增加有效点计数
            sprintf(buffer, "Points: ");
            char temp[30];
            int valid_points = 0;
            
            // 显示最多5个点的数据
            for(int i = 0; i < 5 && i < N10P_POINTS_PER_FRAME; i++) {
                if(n10p_status.current_frame.points[i].distance > 0) {
                    sprintf(temp, "[%d:%dmm,%d] ", 
                           (int)n10p_status.current_frame.points[i].angle,
                           n10p_status.current_frame.points[i].distance,
                           n10p_status.current_frame.points[i].intensity);
                    strcat(buffer, temp);
                    valid_points++;
                }
            }
            
            // 添加有效点统计
            char count_str[20];
            sprintf(count_str, "Valid:%d/%d", valid_points, N10P_POINTS_PER_FRAME);
            strcat(buffer, count_str);
            
            LCD_ShowString(20, 480-20, 760, 16, 16, (u8*)buffer);
        }
        
        delay_ms(200); // 5Hz刷新
        counter++;
    }
}
