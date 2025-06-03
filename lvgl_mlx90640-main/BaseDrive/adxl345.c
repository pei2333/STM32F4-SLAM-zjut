#include "adxl345.h"
#include "delay.h"
#include <math.h>
#include <stddef.h>

// 强制启用STM32功能 - 修复I2C被禁用的致命问题
#define STM32F4XX
#define ENABLE_STM32_I2C

// STM32 库文件包含
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"

#define I2C_PORT            I2C1
#define I2C_CLOCK           RCC_APB1Periph_I2C1
#define GPIO_CLOCK          RCC_AHB1Periph_GPIOB
#define SCL_PIN             GPIO_Pin_6
#define SDA_PIN             GPIO_Pin_7
#define SCL_PIN_SOURCE      GPIO_PinSource6
#define SDA_PIN_SOURCE      GPIO_PinSource7
#define GPIO_PORT           GPIOB
#define I2C_AF              GPIO_AF_I2C1

// ADXL345 寄存器地址定义
#define ADXL345_ADDR            0x53    // ADXL345 器件地址
#define ADXL345_DEVID           0x00    // 器件ID
#define ADXL345_POWER_CTL       0x2D    // 电源控制寄存器
#define ADXL345_DATA_FORMAT     0x31    // 数据格式寄存器
#define ADXL345_DATAX0          0x32    // X轴数据0
#define ADXL345_DATAX1          0x33    // X轴数据1
#define ADXL345_DATAY0          0x34    // Y轴数据0
#define ADXL345_DATAY1          0x35    // Y轴数据1
#define ADXL345_DATAZ0          0x36    // Z轴数据0
#define ADXL345_DATAZ1          0x37    // Z轴数据1

// 滤波参数
#define FS          200.0f    // 采样频率 200Hz
#define FC_LP       5.0f      // 低通截止频率 5Hz
#define FC_HP       0.1f      // 高通截止频率 0.1Hz
#define MA_SIZE     4         // 滑动平均窗口大小
#define PI          3.14159265358979323846f

// 滤波状态变量
static float alpha_lp;        // 低通滤波系数
static float alpha_hp;        // 高通滤波系数
static float y_lp_x_prev = 0, y_lp_y_prev = 0, y_lp_z_prev = 0;  // 低通滤波状态
static float x_hp_prev_x = 0, y_hp_prev_x = 0;  // X轴高通滤波状态
static float x_hp_prev_y = 0, y_hp_prev_y = 0;  // Y轴高通滤波状态
static float x_hp_prev_z = 0, y_hp_prev_z = 0;  // Z轴高通滤波状态
static float buffer_x[MA_SIZE] = {0};  // X轴滑动平均缓冲区
static float buffer_y[MA_SIZE] = {0};  // Y轴滑动平均缓冲区
static float buffer_z[MA_SIZE] = {0};  // Z轴滑动平均缓冲区
static int ma_index = 0;      // 滑动平均缓冲区索引

// 校准偏置
static float bias_x_g = 0, bias_y_g = 0, bias_z_g = 0;

// 全局变量
ADXL345_DATA adxl345_data;

// 滤波器参数
#define TIMEOUT_COUNT       10000   // I2C超时计数

// 静态变量
static bool g_adxl345_initialized = false;
static ADXL345_InterruptCallback g_interrupt_callback = NULL;

// 滤波器状态变量
static float alpha_lp = 0, alpha_hp = 0;
static float y_lp_prev[3] = {0};      // 低通滤波状态 [x,y,z]
static float x_hp_prev[3] = {0};      // 高通滤波输入状态 [x,y,z]
static float y_hp_prev[3] = {0};      // 高通滤波输出状态 [x,y,z]
static float ma_buffer[3][ADXL345_MA_SIZE] = {0}; // 滑动平均缓冲区
static uint8_t ma_buffer_index = 0;   // 改名避免重复定义

// 全局变量定义
ADXL345_DATA adxl345_data = {0};
ADXL345_STATUS adxl345_status = {0};
ADXL345_CALIBRATION adxl345_calibration = {0};

// 私有函数声明
static int8_t I2C_Configuration(void);
static int8_t ADXL345_WriteByte(uint8_t reg, uint8_t data);
static int8_t ADXL345_ReadByte(uint8_t reg, uint8_t *data);
static int8_t ADXL345_ReadMultiBytes(uint8_t reg, uint8_t *buffer, uint8_t length);
static void Filter_Init(void);
static void Lowpass_Filter(float input[3], float output[3]);
static void Highpass_Filter(float input[3], float output[3]);
static void MovingAverage_Filter(float input[3], float output[3]);
static int8_t ADXL345_WaitForFlag(uint32_t flag, bool state);
static void ADXL345_UpdateStatus(void);
static uint32_t GetSystemTick(void);

/**
 * @brief 获取系统时钟节拍（毫秒）
 */
static uint32_t GetSystemTick(void)
{
    // 这里应该调用系统的毫秒计时函数
    // 例如 HAL_GetTick() 或自定义的毫秒计数器
    static uint32_t tick_count = 0;
    return tick_count++;  // 简单的计数器，实际应用中需要使用真实的时钟
}

/**
 * @brief 等待I2C标志位
 */
static int8_t ADXL345_WaitForFlag(uint32_t flag, bool state)
{
    uint32_t timeout = TIMEOUT_COUNT;
    
    while(timeout--) {
        bool current_state = (I2C_GetFlagStatus(I2C_PORT, flag) == SET);
        if(current_state == state) {
            return ADXL345_SUCCESS;
        }
    }
    
    adxl345_status.comm_errors++;
    return ADXL345_TIMEOUT;
}

/**
 * @brief 初始化I2C接口
 */
static int8_t I2C_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能时钟
    RCC_APB1PeriphClockCmd(I2C_CLOCK, ENABLE);
    RCC_AHB1PeriphClockCmd(GPIO_CLOCK, ENABLE);

    // 配置I2C引脚
    GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

    // 配置引脚复用
    GPIO_PinAFConfig(GPIO_PORT, SCL_PIN_SOURCE, I2C_AF);
    GPIO_PinAFConfig(GPIO_PORT, SDA_PIN_SOURCE, I2C_AF);

    // 复位I2C
    I2C_DeInit(I2C_PORT);

    // 配置I2C
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;  // 400kHz
    I2C_Init(I2C_PORT, &I2C_InitStructure);

    // 配置中断（可选）
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 使能I2C
    I2C_Cmd(I2C_PORT, ENABLE);

    return ADXL345_SUCCESS;
}

/**
 * @brief 写入单个字节到ADXL345
 */
static int8_t ADXL345_WriteByte(uint8_t reg, uint8_t data)
{
    // 等待总线空闲
    if(ADXL345_WaitForFlag(I2C_FLAG_BUSY, false) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送起始条件
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    if(ADXL345_WaitForFlag(I2C_FLAG_SB, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送设备地址（写）
    I2C_Send7bitAddress(I2C_PORT, ADXL345_ADDR << 1, I2C_Direction_Transmitter);
    if(ADXL345_WaitForFlag(I2C_FLAG_ADDR, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 清除ADDR标志
    I2C_PORT->SR2;
    
    // 发送寄存器地址
    I2C_SendData(I2C_PORT, reg);
    if(ADXL345_WaitForFlag(I2C_FLAG_BTF, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送数据
    I2C_SendData(I2C_PORT, data);
    if(ADXL345_WaitForFlag(I2C_FLAG_BTF, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送停止条件
    I2C_GenerateSTOP(I2C_PORT, ENABLE);
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 从ADXL345读取单个字节
 */
static int8_t ADXL345_ReadByte(uint8_t reg, uint8_t *data)
{
    if(data == NULL) return ADXL345_INVALID;
    
    // 等待总线空闲
    if(ADXL345_WaitForFlag(I2C_FLAG_BUSY, false) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送起始条件
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    if(ADXL345_WaitForFlag(I2C_FLAG_SB, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送设备地址（写）
    I2C_Send7bitAddress(I2C_PORT, ADXL345_ADDR << 1, I2C_Direction_Transmitter);
    if(ADXL345_WaitForFlag(I2C_FLAG_ADDR, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 清除ADDR标志
    I2C_PORT->SR2;
    
    // 发送寄存器地址
    I2C_SendData(I2C_PORT, reg);
    if(ADXL345_WaitForFlag(I2C_FLAG_BTF, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 重新发送起始条件
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    if(ADXL345_WaitForFlag(I2C_FLAG_SB, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送设备地址（读）
    I2C_Send7bitAddress(I2C_PORT, ADXL345_ADDR << 1, I2C_Direction_Receiver);
    if(ADXL345_WaitForFlag(I2C_FLAG_ADDR, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 禁用ACK
    I2C_AcknowledgeConfig(I2C_PORT, DISABLE);
    
    // 清除ADDR标志
    I2C_PORT->SR2;
    
    // 发送停止条件
    I2C_GenerateSTOP(I2C_PORT, ENABLE);
    
    // 等待数据接收
    if(ADXL345_WaitForFlag(I2C_FLAG_RXNE, true) != ADXL345_SUCCESS) {
        I2C_AcknowledgeConfig(I2C_PORT, ENABLE);
        return ADXL345_TIMEOUT;
    }
    
    // 读取数据
    *data = I2C_ReceiveData(I2C_PORT);
    
    // 重新使能ACK
    I2C_AcknowledgeConfig(I2C_PORT, ENABLE);
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 从ADXL345读取多个字节
 */
static int8_t ADXL345_ReadMultiBytes(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if(buffer == NULL || length == 0) return ADXL345_INVALID;
    
    // 等待总线空闲
    if(ADXL345_WaitForFlag(I2C_FLAG_BUSY, false) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送起始条件
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    if(ADXL345_WaitForFlag(I2C_FLAG_SB, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送设备地址（写）
    I2C_Send7bitAddress(I2C_PORT, ADXL345_ADDR << 1, I2C_Direction_Transmitter);
    if(ADXL345_WaitForFlag(I2C_FLAG_ADDR, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 清除ADDR标志
    I2C_PORT->SR2;
    
    // 发送寄存器地址
    I2C_SendData(I2C_PORT, reg);
    if(ADXL345_WaitForFlag(I2C_FLAG_BTF, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 重新发送起始条件
    I2C_GenerateSTART(I2C_PORT, ENABLE);
    if(ADXL345_WaitForFlag(I2C_FLAG_SB, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 发送设备地址（读）
    I2C_Send7bitAddress(I2C_PORT, ADXL345_ADDR << 1, I2C_Direction_Receiver);
    if(ADXL345_WaitForFlag(I2C_FLAG_ADDR, true) != ADXL345_SUCCESS) {
        return ADXL345_TIMEOUT;
    }
    
    // 清除ADDR标志
    I2C_PORT->SR2;
    
    // 读取数据
    for(uint8_t i = 0; i < length; i++) {
        if(i == length - 1) {
            // 最后一个字节，禁用ACK
            I2C_AcknowledgeConfig(I2C_PORT, DISABLE);
            I2C_GenerateSTOP(I2C_PORT, ENABLE);
        }
        
        // 等待数据接收
        if(ADXL345_WaitForFlag(I2C_FLAG_RXNE, true) != ADXL345_SUCCESS) {
            I2C_AcknowledgeConfig(I2C_PORT, ENABLE);
            return ADXL345_TIMEOUT;
        }
        
        buffer[i] = I2C_ReceiveData(I2C_PORT);
    }
    
    // 重新使能ACK
    I2C_AcknowledgeConfig(I2C_PORT, ENABLE);
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 初始化滤波器参数
 */
static void Filter_Init(void)
{
    float dt = 1.0f / FS;
    
    // 计算低通滤波系数
    float rc_lp = 1.0f / (2.0f * PI * FC_LP);
    alpha_lp = expf(-dt / rc_lp);
    
    // 计算高通滤波系数
    float rc_hp = 1.0f / (2.0f * PI * FC_HP);
    alpha_hp = rc_hp / (rc_hp + dt);
    
    // 清零滤波器状态
    for(int i = 0; i < 3; i++) {
        y_lp_prev[i] = 0;
        x_hp_prev[i] = 0;
        y_hp_prev[i] = 0;
        for(int j = 0; j < MA_SIZE; j++) {
            buffer_x[j] = 0;
            buffer_y[j] = 0;
            buffer_z[j] = 0;
        }
    }
    ma_index = 0;
}

/**
 * @brief 低通滤波器
 */
static void Lowpass_Filter(float input[3], float output[3])
{
    for(int i = 0; i < 3; i++) {
        output[i] = alpha_lp * input[i] + (1.0f - alpha_lp) * y_lp_prev[i];
        y_lp_prev[i] = output[i];
    }
}

/**
 * @brief 高通滤波器
 */
static void Highpass_Filter(float input[3], float output[3])
{
    for(int i = 0; i < 3; i++) {
        float x_diff = input[i] - x_hp_prev[i];
        output[i] = alpha_hp * (y_hp_prev[i] + x_diff);
        x_hp_prev[i] = input[i];
        y_hp_prev[i] = output[i];
    }
}

/**
 * @brief 滑动平均滤波器
 */
static void MovingAverage_Filter(float input[3], float output[3])
{
    // 更新缓冲区
    for(int i = 0; i < 3; i++) {
        buffer_x[ma_index] = input[i];
    }
    ma_index = (ma_index + 1) % MA_SIZE;
    
    // 计算平均值
    for(int i = 0; i < 3; i++) {
        float sum = 0;
        for(int j = 0; j < MA_SIZE; j++) {
            sum += buffer_x[j];
        }
        output[i] = sum / MA_SIZE;
    }
}

/**
 * @brief 更新ADXL345状态
 */
static void ADXL345_UpdateStatus(void)
{
    adxl345_status.last_update = GetSystemTick();
    adxl345_status.data_ready = true;
}

/**
 * @brief 初始化ADXL345
 */
int8_t ADXL345_Init(void)
{
    uint8_t device_id;
    int8_t result;
    
    // 初始化状态
    adxl345_status.initialized = false;
    adxl345_status.comm_errors = 0;
    adxl345_status.data_ready = false;
    adxl345_status.range = ADXL345_RANGE_16G;
    adxl345_status.self_test_passed = false;
    
    // 初始化I2C接口
    result = I2C_Configuration();
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 短暂延时等待器件启动
    delay_ms(10);
    
    // 检查设备ID
    result = ADXL345_ReadByte(ADXL345_DEVID, &device_id);
    if(result != ADXL345_SUCCESS) {
        return ADXL345_COMM_ERROR;
    }
    
    if(device_id != ADXL345_DEVICE_ID) {
        return ADXL345_NOT_FOUND;
    }
    
    // 软复位 - 写入电源控制寄存器
    result = ADXL345_WriteByte(ADXL345_POWER_CTL, 0x00);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    delay_ms(5);
    
    // 配置数据格式：全分辨率模式，±16g量程
    result = ADXL345_WriteByte(ADXL345_DATA_FORMAT, 
                              ADXL345_FULL_RES | ADXL345_RANGE_16G);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 设置数据速率：200Hz
    result = ADXL345_WriteByte(ADXL345_BW_RATE, 0x0C);  // 200Hz
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 启用测量模式
    result = ADXL345_WriteByte(ADXL345_POWER_CTL, ADXL345_PCTL_MEASURE);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 初始化滤波器
    Filter_Init();
    
    // 执行校准
    result = ADXL345_Calibrate();
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 执行自检
    result = ADXL345_SelfTest();
    if(result != ADXL345_SUCCESS) {
        adxl345_status.self_test_passed = false;
        // 自检失败不阻止初始化，仅记录状态
    } else {
        adxl345_status.self_test_passed = true;
    }
    
    // 标记初始化完成
    adxl345_status.initialized = true;
    g_adxl345_initialized = true;
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 反初始化ADXL345
 */
int8_t ADXL345_Deinit(void)
{
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 进入待机模式
    ADXL345_WriteByte(ADXL345_POWER_CTL, ADXL345_PCTL_WAKEUP);
    
    // 禁用所有中断
    ADXL345_WriteByte(ADXL345_INT_ENABLE, 0x00);
    
    // 复位I2C
    I2C_DeInit(I2C_PORT);
    
    // 清除状态
    adxl345_status.initialized = false;
    g_adxl345_initialized = false;
    g_interrupt_callback = NULL;
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 软复位ADXL345
 */
int8_t ADXL345_Reset(void)
{
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 先反初始化再重新初始化
    ADXL345_Deinit();
    delay_ms(10);
    return ADXL345_Init();
}

/**
 * @brief 读取原始数据
 */
int8_t ADXL345_ReadRawData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[6];
    int8_t result;
    
    if(!g_adxl345_initialized || x == NULL || y == NULL || z == NULL) {
        return ADXL345_ERROR;
    }
    
    // 读取6字节原始数据
    result = ADXL345_ReadMultiBytes(ADXL345_DATAX0, buffer, 6);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 转换为16位有符号数
    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 读取并处理ADXL345数据
 */
int8_t ADXL345_ReadData(void)
{
    int16_t raw_x, raw_y, raw_z;
    float acc_raw[3], acc_lp[3], acc_hp[3], acc_final[3];
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 读取原始数据
    result = ADXL345_ReadRawData(&raw_x, &raw_y, &raw_z);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 转换为g单位（16g量程，13位分辨率）
    const float scale = 0.004f;  // 4mg/LSB
    acc_raw[0] = raw_x * scale - adxl345_calibration.x_bias;
    acc_raw[1] = raw_y * scale - adxl345_calibration.y_bias;
    acc_raw[2] = raw_z * scale - adxl345_calibration.z_bias;
    
    // 应用滤波器链
    Lowpass_Filter(acc_raw, acc_lp);
    Highpass_Filter(acc_lp, acc_hp);
    MovingAverage_Filter(acc_hp, acc_final);
    
    // 更新全局数据
    adxl345_data.x = acc_final[0];
    adxl345_data.y = acc_final[1];
    adxl345_data.z = acc_final[2];
    adxl345_data.timestamp = GetSystemTick();
    
    // 更新状态
    ADXL345_UpdateStatus();
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 校准ADXL345
 */
int8_t ADXL345_Calibrate(void)
{
    const uint16_t sample_count = ADXL345_CALIB_SAMPLES;
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t raw_x, raw_y, raw_z;
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 清除当前偏置
    adxl345_calibration.x_bias = 0;
    adxl345_calibration.y_bias = 0;
    adxl345_calibration.z_bias = 0;
    
    // 收集校准样本
    for(uint16_t i = 0; i < sample_count; i++) {
        result = ADXL345_ReadRawData(&raw_x, &raw_y, &raw_z);
        if(result != ADXL345_SUCCESS) {
            return result;
        }
        
        // 累加原始值
        const float scale = 0.004f;
        sum_x += raw_x * scale;
        sum_y += raw_y * scale;
        sum_z += raw_z * scale;
        
        delay_ms(5);  // 5ms间隔
    }
    
    // 计算平均值作为偏置
    adxl345_calibration.x_bias = sum_x / sample_count;
    adxl345_calibration.y_bias = sum_y / sample_count;
    adxl345_calibration.z_bias = sum_z / sample_count - 1.0f;  // Z轴减去1g重力
    
    adxl345_calibration.calibrated = true;
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 自检功能
 */
int8_t ADXL345_SelfTest(void)
{
    uint8_t original_format;
    int16_t normal_x, normal_y, normal_z;
    int16_t selftest_x, selftest_y, selftest_z;
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 读取当前数据格式设置
    result = ADXL345_ReadByte(ADXL345_DATA_FORMAT, &original_format);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 读取正常模式下的数据
    result = ADXL345_ReadRawData(&normal_x, &normal_y, &normal_z);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 启用自检模式
    result = ADXL345_WriteByte(ADXL345_DATA_FORMAT, original_format | 0x80);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    delay_ms(10);  // 等待稳定
    
    // 读取自检模式下的数据
    result = ADXL345_ReadRawData(&selftest_x, &selftest_y, &selftest_z);
    if(result != ADXL345_SUCCESS) {
        // 恢复原始设置
        ADXL345_WriteByte(ADXL345_DATA_FORMAT, original_format);
        return result;
    }
    
    // 恢复原始设置
    result = ADXL345_WriteByte(ADXL345_DATA_FORMAT, original_format);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 计算自检变化量（应该在50-540 LSB范围内）
    int16_t diff_x = abs(selftest_x - normal_x);
    int16_t diff_y = abs(selftest_y - normal_y);
    int16_t diff_z = abs(selftest_z - normal_z);
    
    // 检查自检结果
    if(diff_x >= 50 && diff_x <= 540 &&
       diff_y >= 50 && diff_y <= 540 &&
       diff_z >= 75 && diff_z <= 875) {  // Z轴阈值略有不同
        return ADXL345_SUCCESS;
    }
    
    return ADXL345_ERROR;
}

/**
 * @brief 设置量程
 */
int8_t ADXL345_SetRange(uint8_t range)
{
    uint8_t data_format;
    int8_t result;
    
    if(!g_adxl345_initialized || range > ADXL345_RANGE_16G) {
        return ADXL345_ERROR;
    }
    
    // 读取当前数据格式
    result = ADXL345_ReadByte(ADXL345_DATA_FORMAT, &data_format);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 更新量程设置
    data_format = (data_format & 0xFC) | range;
    result = ADXL345_WriteByte(ADXL345_DATA_FORMAT, data_format);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    adxl345_status.range = range;
    return ADXL345_SUCCESS;
}

/**
 * @brief 设置数据速率
 */
int8_t ADXL345_SetDataRate(uint8_t rate)
{
    if(!g_adxl345_initialized || rate > 0x0F) {
        return ADXL345_ERROR;
    }
    
    return ADXL345_WriteByte(ADXL345_BW_RATE, rate);
}

/**
 * @brief 设置电源模式
 */
int8_t ADXL345_SetPowerMode(uint8_t mode)
{
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    return ADXL345_WriteByte(ADXL345_POWER_CTL, mode);
}

/**
 * @brief 进入睡眠模式
 */
int8_t ADXL345_Sleep(void)
{
    return ADXL345_SetPowerMode(ADXL345_PCTL_SLEEP);
}

/**
 * @brief 唤醒设备
 */
int8_t ADXL345_Wakeup(void)
{
    return ADXL345_SetPowerMode(ADXL345_PCTL_MEASURE);
}

/**
 * @brief 设置偏移量
 */
int8_t ADXL345_SetOffset(float x_offset, float y_offset, float z_offset)
{
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 将g转换为偏移寄存器值（15.6mg/LSB）
    int8_t x_off = (int8_t)(x_offset / 0.0156f);
    int8_t y_off = (int8_t)(y_offset / 0.0156f);
    int8_t z_off = (int8_t)(z_offset / 0.0156f);
    
    result = ADXL345_WriteByte(ADXL345_OFSX, x_off);
    if(result != ADXL345_SUCCESS) return result;
    
    result = ADXL345_WriteByte(ADXL345_OFSY, y_off);
    if(result != ADXL345_SUCCESS) return result;
    
    result = ADXL345_WriteByte(ADXL345_OFSZ, z_off);
    if(result != ADXL345_SUCCESS) return result;
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 使能/禁用中断
 */
int8_t ADXL345_EnableInterrupt(uint8_t int_type, bool enable)
{
    uint8_t int_enable;
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 读取当前中断使能状态
    result = ADXL345_ReadByte(ADXL345_INT_ENABLE, &int_enable);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 更新中断使能
    if(enable) {
        int_enable |= int_type;
    } else {
        int_enable &= ~int_type;
    }
    
    return ADXL345_WriteByte(ADXL345_INT_ENABLE, int_enable);
}

/**
 * @brief 映射中断到引脚
 */
int8_t ADXL345_MapInterrupt(uint8_t int_type, uint8_t pin)
{
    uint8_t int_map;
    int8_t result;
    
    if(!g_adxl345_initialized || pin > 1) {
        return ADXL345_ERROR;
    }
    
    // 读取当前中断映射
    result = ADXL345_ReadByte(ADXL345_INT_MAP, &int_map);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 更新中断映射（0=INT1, 1=INT2）
    if(pin == 0) {
        int_map &= ~int_type;  // 映射到INT1
    } else {
        int_map |= int_type;   // 映射到INT2
    }
    
    return ADXL345_WriteByte(ADXL345_INT_MAP, int_map);
}

/**
 * @brief 获取中断源
 */
uint8_t ADXL345_GetInterruptSource(void)
{
    uint8_t int_source = 0;
    
    if(g_adxl345_initialized) {
        ADXL345_ReadByte(ADXL345_INT_SOURCE, &int_source);
    }
    
    return int_source;
}

/**
 * @brief 检查数据是否就绪
 */
bool ADXL345_IsDataReady(void)
{
    uint8_t int_source = ADXL345_GetInterruptSource();
    return (int_source & ADXL345_INT_DATA_READY) ? true : false;
}

/**
 * @brief 获取状态
 */
ADXL345_STATUS ADXL345_GetStatus(void)
{
    return adxl345_status;
}

/**
 * @brief 获取校准信息
 */
ADXL345_CALIBRATION ADXL345_GetCalibration(void)
{
    return adxl345_calibration;
}

/**
 * @brief 设置中断回调函数
 */
void ADXL345_SetInterruptCallback(ADXL345_InterruptCallback callback)
{
    g_interrupt_callback = callback;
}

/**
 * @brief 中断处理函数
 */
void ADXL345_IRQHandler(void)
{
    if(!g_adxl345_initialized) {
        return;
    }
    
    uint8_t int_source = ADXL345_GetInterruptSource();
    
    if(int_source & ADXL345_INT_DATA_READY) {
        // 数据就绪中断 - 自动读取数据
        ADXL345_ReadData();
    }
    
    // 调用用户回调函数
    if(g_interrupt_callback != NULL) {
        g_interrupt_callback(int_source);
    }
}

/**
 * @brief 配置敲击检测
 */
int8_t ADXL345_ConfigTapDetection(uint8_t threshold, uint8_t duration)
{
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 设置敲击阈值
    result = ADXL345_WriteByte(ADXL345_THRESH_TAP, threshold);
    if(result != ADXL345_SUCCESS) return result;
    
    // 设置敲击持续时间
    result = ADXL345_WriteByte(ADXL345_DUR, duration);
    if(result != ADXL345_SUCCESS) return result;
    
    // 使能XYZ轴敲击检测
    result = ADXL345_WriteByte(ADXL345_TAP_AXES, 0x07);
    if(result != ADXL345_SUCCESS) return result;
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 配置活动检测
 */
int8_t ADXL345_ConfigActivityDetection(uint8_t threshold, uint8_t time)
{
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 设置活动阈值
    result = ADXL345_WriteByte(ADXL345_THRESH_ACT, threshold);
    if(result != ADXL345_SUCCESS) return result;
    
    // 设置静止阈值
    result = ADXL345_WriteByte(ADXL345_THRESH_INACT, threshold / 2);
    if(result != ADXL345_SUCCESS) return result;
    
    // 设置静止时间
    result = ADXL345_WriteByte(ADXL345_TIME_INACT, time);
    if(result != ADXL345_SUCCESS) return result;
    
    // 配置活动/静止控制
    result = ADXL345_WriteByte(ADXL345_ACT_INACT_CTL, 0xFF);
    if(result != ADXL345_SUCCESS) return result;
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 配置自由落体检测
 */
int8_t ADXL345_ConfigFreeFallDetection(uint8_t threshold, uint8_t time)
{
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 设置自由落体阈值
    result = ADXL345_WriteByte(ADXL345_THRESH_FF, threshold);
    if(result != ADXL345_SUCCESS) return result;
    
    // 设置自由落体时间
    result = ADXL345_WriteByte(ADXL345_TIME_FF, time);
    if(result != ADXL345_SUCCESS) return result;
    
    return ADXL345_SUCCESS;
}

/**
 * @brief 设置低功耗模式
 */
int8_t ADXL345_SetLowPowerMode(bool enable)
{
    uint8_t power_ctl;
    int8_t result;
    
    if(!g_adxl345_initialized) {
        return ADXL345_ERROR;
    }
    
    // 读取当前电源控制状态
    result = ADXL345_ReadByte(ADXL345_POWER_CTL, &power_ctl);
    if(result != ADXL345_SUCCESS) {
        return result;
    }
    
    // 更新低功耗位
    if(enable) {
        power_ctl |= 0x10;  // 设置低功耗位
    } else {
        power_ctl &= ~0x10; // 清除低功耗位
    }
    
    return ADXL345_WriteByte(ADXL345_POWER_CTL, power_ctl);
}

