#include "ADXL345.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// STM32 库文件包含
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

// 正确的引脚配置 - 基于能工作的加速度计实验
#define ADXL345_SDA_PORT    GPIOA
#define ADXL345_SDA_PIN     GPIO_Pin_3
#define ADXL345_SCL_PORT    GPIOA  
#define ADXL345_SCL_PIN     GPIO_Pin_6

// 引脚操作宏
#define ADXL345_SDA_H()     GPIO_SetBits(ADXL345_SDA_PORT, ADXL345_SDA_PIN)
#define ADXL345_SDA_L()     GPIO_ResetBits(ADXL345_SDA_PORT, ADXL345_SDA_PIN)
#define ADXL345_SCL_H()     GPIO_SetBits(ADXL345_SCL_PORT, ADXL345_SCL_PIN)
#define ADXL345_SCL_L()     GPIO_ResetBits(ADXL345_SCL_PORT, ADXL345_SCL_PIN)
#define ADXL345_SDA_READ()  GPIO_ReadInputDataBit(ADXL345_SDA_PORT, ADXL345_SDA_PIN)

// 正确的ADXL345地址和寄存器定义
#define ADXL345_DEVICE_ADDR  0x53    // 7位地址
#define ADXL345_WRITE_ADDR   0xA6    // 写地址 (0x53 << 1)  
#define ADXL345_READ_ADDR    0xA7    // 读地址 (0x53 << 1 | 1)

// 寄存器地址定义
#define ADXL345_DEVID_REG       0x00
#define ADXL345_POWER_CTL_REG   0x2D
#define ADXL345_DATA_FORMAT_REG 0x31
#define ADXL345_BW_RATE_REG     0x2C
#define ADXL345_INT_ENABLE_REG  0x2E
#define ADXL345_OFSX_REG        0x1E
#define ADXL345_OFSY_REG        0x1F
#define ADXL345_OFSZ_REG        0x20
#define ADXL345_DATAX0_REG      0x32
#define ADXL345_DATAX1_REG      0x33
#define ADXL345_DATAY0_REG      0x34
#define ADXL345_DATAY1_REG      0x35
#define ADXL345_DATAZ0_REG      0x36
#define ADXL345_DATAZ1_REG      0x37

// 全局变量定义
ADXL345_DATA adxl345_data = {0};
ADXL345_STATUS adxl345_status = {0};
ADXL345_CALIBRATION adxl345_calibration = {0};

// 私有函数声明
static void ADXL345_GPIO_Init(void);
static void ADXL345_I2C_Start(void);
static void ADXL345_I2C_Stop(void);
static void ADXL345_I2C_SendByte(uint8_t byte);
static uint8_t ADXL345_I2C_ReceiveByte(uint8_t ack);
static uint8_t ADXL345_I2C_WaitAck(void);
static void ADXL345_SDA_Output(void);
static void ADXL345_SDA_Input(void);

/**
 * @brief GPIO初始化 - 使用PA3(SDA)和PA6(SCL)
 */
static void ADXL345_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIOA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    // 配置SCL引脚(PA6)为开漏输出
    GPIO_InitStructure.GPIO_Pin = ADXL345_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ADXL345_SCL_PORT, &GPIO_InitStructure);
    
    // 配置SDA引脚(PA3)为开漏输出
    GPIO_InitStructure.GPIO_Pin = ADXL345_SDA_PIN;
    GPIO_Init(ADXL345_SDA_PORT, &GPIO_InitStructure);
    
    // 初始状态设为高电平
    ADXL345_SCL_H();
    ADXL345_SDA_H();
}

/**
 * @brief 配置SDA为输出模式
 */
static void ADXL345_SDA_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Pin = ADXL345_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ADXL345_SDA_PORT, &GPIO_InitStructure);
}

/**
 * @brief 配置SDA为输入模式
 */
static void ADXL345_SDA_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Pin = ADXL345_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ADXL345_SDA_PORT, &GPIO_InitStructure);
}

/**
 * @brief I2C开始信号
 */
static void ADXL345_I2C_Start(void)
{
    ADXL345_SDA_Output();
    ADXL345_SDA_H();
    ADXL345_SCL_H();
    delay_us(4);
    ADXL345_SDA_L();
    delay_us(4);
    ADXL345_SCL_L();
}

/**
 * @brief I2C停止信号
 */
static void ADXL345_I2C_Stop(void)
{
    ADXL345_SDA_Output();
    ADXL345_SCL_L();
    ADXL345_SDA_L();
    delay_us(4);
    ADXL345_SCL_H();
    delay_us(4);
    ADXL345_SDA_H();
    delay_us(4);
}

/**
 * @brief I2C发送一个字节
 */
static void ADXL345_I2C_SendByte(uint8_t byte)
{
    uint8_t i;
    
    ADXL345_SDA_Output();
    ADXL345_SCL_L();
    
    for(i = 0; i < 8; i++)
    {
        if(byte & 0x80)
            ADXL345_SDA_H();
        else
            ADXL345_SDA_L();
        
        byte <<= 1;
        delay_us(2);
        ADXL345_SCL_H();
        delay_us(2);
        ADXL345_SCL_L();
        delay_us(2);
    }
}

/**
 * @brief I2C接收一个字节
 */
static uint8_t ADXL345_I2C_ReceiveByte(uint8_t ack)
{
    uint8_t i, receive = 0;
    
    ADXL345_SDA_Input();
    
    for(i = 0; i < 8; i++)
    {
        ADXL345_SCL_L();
        delay_us(2);
        ADXL345_SCL_H();
        receive <<= 1;
        
        if(ADXL345_SDA_READ())
            receive++;
        
        delay_us(1);
    }
    
    ADXL345_SCL_L();
    
    ADXL345_SDA_Output();
    if(ack)
        ADXL345_SDA_L();
    else
        ADXL345_SDA_H();
    
    delay_us(2);
    ADXL345_SCL_H();
    delay_us(2);
    ADXL345_SCL_L();
    
    return receive;
}

/**
 * @brief 等待ACK信号
 */
static uint8_t ADXL345_I2C_WaitAck(void)
{
    uint8_t ucErrTime = 0;
    
    ADXL345_SDA_Input();
    ADXL345_SDA_H();
    delay_us(1);
    ADXL345_SCL_H();
    delay_us(1);
    
    while(ADXL345_SDA_READ())
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            ADXL345_I2C_Stop();
            return 1;
        }
    }
    
    ADXL345_SCL_L();
    return 0;
}

/**
 * @brief 写ADXL345寄存器
 */
static int8_t ADXL345_WriteReg(uint8_t reg, uint8_t data)
{
    ADXL345_I2C_Start();
    ADXL345_I2C_SendByte(ADXL345_WRITE_ADDR);
    
    if(ADXL345_I2C_WaitAck())
    {
        ADXL345_I2C_Stop();
        return ADXL345_COMM_ERROR;
    }
    
    ADXL345_I2C_SendByte(reg);
    ADXL345_I2C_WaitAck();
    
    ADXL345_I2C_SendByte(data);
    ADXL345_I2C_WaitAck();
    
    ADXL345_I2C_Stop();
    return ADXL345_SUCCESS;
}

/**
 * @brief 读ADXL345寄存器
 */
static int8_t ADXL345_ReadReg(uint8_t reg, uint8_t *data)
{
    ADXL345_I2C_Start();
    ADXL345_I2C_SendByte(ADXL345_WRITE_ADDR);
    
    if(ADXL345_I2C_WaitAck())
    {
        ADXL345_I2C_Stop();
        return ADXL345_COMM_ERROR;
    }
    
    ADXL345_I2C_SendByte(reg);
    ADXL345_I2C_WaitAck();
    
    ADXL345_I2C_Start();
    ADXL345_I2C_SendByte(ADXL345_READ_ADDR);
    ADXL345_I2C_WaitAck();
    
    *data = ADXL345_I2C_ReceiveByte(0);
    ADXL345_I2C_Stop();
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 读取多个寄存器
 */
static int8_t ADXL345_ReadMultiRegs(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t i;
    
    ADXL345_I2C_Start();
    ADXL345_I2C_SendByte(ADXL345_WRITE_ADDR);
    
    if(ADXL345_I2C_WaitAck())
    {
        ADXL345_I2C_Stop();
        return ADXL345_COMM_ERROR;
    }
    
    ADXL345_I2C_SendByte(reg);
    ADXL345_I2C_WaitAck();
    
    ADXL345_I2C_Start();
    ADXL345_I2C_SendByte(ADXL345_READ_ADDR);
    ADXL345_I2C_WaitAck();
    
    for(i = 0; i < length; i++)
    {
        if(i == length - 1)
            buffer[i] = ADXL345_I2C_ReceiveByte(0);  // 最后一个字节NACK
        else
            buffer[i] = ADXL345_I2C_ReceiveByte(1);  // ACK
    }
    
    ADXL345_I2C_Stop();
    return ADXL345_SUCCESS;
}

/**
 * @brief ADXL345初始化
 */
int8_t ADXL345_Init(void)
{
    uint8_t device_id;
    
    // 初始化GPIO
    ADXL345_GPIO_Init();
    delay_ms(50);
    
    // 读取器件ID
    if(ADXL345_ReadReg(ADXL345_DEVID_REG, &device_id) != ADXL345_SUCCESS)
    {
        adxl345_status.comm_errors++;
        return ADXL345_COMM_ERROR;
    }
    
    // 检查器件ID
    if(device_id != ADXL345_DEVICE_ID)
    {
        return ADXL345_NOT_FOUND;
    }
    
    // 复位设备 - 先进入待机模式
    if(ADXL345_WriteReg(ADXL345_POWER_CTL_REG, 0x00) != ADXL345_SUCCESS)
        return ADXL345_COMM_ERROR;
    
    delay_ms(5);
    
    // 配置数据格式：±16g量程，全分辨率，右对齐
    if(ADXL345_WriteReg(ADXL345_DATA_FORMAT_REG, 0x0B) != ADXL345_SUCCESS)
        return ADXL345_COMM_ERROR;
    
    // 设置数据输出速率为100Hz
    if(ADXL345_WriteReg(ADXL345_BW_RATE_REG, 0x0A) != ADXL345_SUCCESS)
        return ADXL345_COMM_ERROR;
    
    // 禁用中断
    if(ADXL345_WriteReg(ADXL345_INT_ENABLE_REG, 0x00) != ADXL345_SUCCESS)
        return ADXL345_COMM_ERROR;
    
    // 清除偏移
    ADXL345_WriteReg(ADXL345_OFSX_REG, 0x00);
    ADXL345_WriteReg(ADXL345_OFSY_REG, 0x00);
    ADXL345_WriteReg(ADXL345_OFSZ_REG, 0x00);
    
    // 启动测量模式
    if(ADXL345_WriteReg(ADXL345_POWER_CTL_REG, 0x08) != ADXL345_SUCCESS)
        return ADXL345_COMM_ERROR;
    
    delay_ms(10);
    
    // 更新状态
    adxl345_status.initialized = true;
    adxl345_status.comm_errors = 0;
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 读取原始数据
 */
int8_t ADXL345_ReadRawData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];
    
    if(!adxl345_status.initialized)
        return ADXL345_ERROR;
    
    // 从0x32开始连续读取6个字节
    if(ADXL345_ReadMultiRegs(ADXL345_DATAX0_REG, buffer, 6) != ADXL345_SUCCESS)
    {
        adxl345_status.comm_errors++;
        return ADXL345_COMM_ERROR;
    }
    
    // 组合数据 (小端格式)
    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 读取加速度数据
 */
int8_t ADXL345_ReadData(void)
{
    int16_t raw_x, raw_y, raw_z;
    
    if(ADXL345_ReadRawData(&raw_x, &raw_y, &raw_z) != ADXL345_SUCCESS)
        return ADXL345_ERROR;
    
    // 转换为g值 (假设±16g量程，13位分辨率)
    adxl345_data.x = raw_x * 0.03125f;  // 32mg/LSB for ±16g
    adxl345_data.y = raw_y * 0.03125f;
    adxl345_data.z = raw_z * 0.03125f;
    
    adxl345_status.data_ready = true;
    adxl345_status.last_update = 0; // 简化时间戳
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 设置量程
 */
int8_t ADXL345_SetRange(uint8_t range)
{
    uint8_t format = 0x08; // 全分辨率模式
    
    switch(range)
    {
        case ADXL345_RANGE_2G:  format |= 0x00; break;
        case ADXL345_RANGE_4G:  format |= 0x01; break;
        case ADXL345_RANGE_8G:  format |= 0x02; break;
        case ADXL345_RANGE_16G: format |= 0x03; break;
        default: return ADXL345_INVALID;
    }
    
    if(ADXL345_WriteReg(ADXL345_DATA_FORMAT_REG, format) != ADXL345_SUCCESS)
        return ADXL345_COMM_ERROR;
    
    adxl345_status.range = range;
    return ADXL345_SUCCESS;
}

/**
 * @brief 设置数据输出速率
 */
int8_t ADXL345_SetDataRate(uint8_t rate)
{
    return ADXL345_WriteReg(ADXL345_BW_RATE_REG, rate);
}

/**
 * @brief 设置电源模式
 */
int8_t ADXL345_SetPowerMode(uint8_t mode)
{
    return ADXL345_WriteReg(ADXL345_POWER_CTL_REG, mode);
}

/**
 * @brief 获取状态
 */
ADXL345_STATUS ADXL345_GetStatus(void)
{
    return adxl345_status;
}

/**
 * @brief 检查数据是否就绪
 */
bool ADXL345_IsDataReady(void)
{
    return adxl345_status.data_ready;
}

/**
 * @brief 复位设备
 */
int8_t ADXL345_Reset(void)
{
    // 进入待机模式
    ADXL345_WriteReg(ADXL345_POWER_CTL_REG, 0x00);
    delay_ms(10);
    
    // 重新初始化
    return ADXL345_Init();
}

/**
 * @brief 反初始化
 */
int8_t ADXL345_Deinit(void)
{
    // 进入待机模式
    ADXL345_WriteReg(ADXL345_POWER_CTL_REG, 0x00);
    
    adxl345_status.initialized = false;
    return ADXL345_SUCCESS;
}

// 其他必需的占位符函数实现
int8_t ADXL345_EnableInterrupt(uint8_t int_type, bool enable) { return ADXL345_SUCCESS; }
int8_t ADXL345_MapInterrupt(uint8_t int_type, uint8_t pin) { return ADXL345_SUCCESS; }
int8_t ADXL345_Calibrate(void) { return ADXL345_SUCCESS; }
int8_t ADXL345_SelfTest(void) { return ADXL345_SUCCESS; }
int8_t ADXL345_SetOffset(float x_offset, float y_offset, float z_offset) { return ADXL345_SUCCESS; }
ADXL345_CALIBRATION ADXL345_GetCalibration(void) { return adxl345_calibration; }
uint8_t ADXL345_GetInterruptSource(void) { return 0; }
void ADXL345_SetInterruptCallback(ADXL345_InterruptCallback callback) { }
void ADXL345_IRQHandler(void) { }
int8_t ADXL345_Sleep(void) { return ADXL345_SetPowerMode(0x04); }
int8_t ADXL345_Wakeup(void) { return ADXL345_SetPowerMode(0x08); }
int8_t ADXL345_SetLowPowerMode(bool enable) { return ADXL345_SUCCESS; }
int8_t ADXL345_ConfigTapDetection(uint8_t threshold, uint8_t duration) { return ADXL345_SUCCESS; }
int8_t ADXL345_ConfigActivityDetection(uint8_t threshold, uint8_t time) { return ADXL345_SUCCESS; }
int8_t ADXL345_ConfigFreeFallDetection(uint8_t threshold, uint8_t time) { return ADXL345_SUCCESS; }

