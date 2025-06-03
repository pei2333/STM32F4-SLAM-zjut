#ifndef __ADXL345_H
#define __ADXL345_H

#include <stdint.h>
#include <stdbool.h>

// 错误代码定义
#define ADXL345_SUCCESS      0
#define ADXL345_ERROR       -1
#define ADXL345_TIMEOUT     -2
#define ADXL345_INVALID     -3
#define ADXL345_NOT_FOUND   -4
#define ADXL345_COMM_ERROR  -5

// ADXL345 寄存器地址定义
#define ADXL345_ADDR            0x53    // ADXL345 I2C器件地址
#define ADXL345_DEVID           0x00    // 器件ID寄存器
#define ADXL345_THRESH_TAP      0x1D    // 敲击阈值
#define ADXL345_OFSX            0x1E    // X轴偏移
#define ADXL345_OFSY            0x1F    // Y轴偏移
#define ADXL345_OFSZ            0x20    // Z轴偏移
#define ADXL345_DUR             0x21    // 敲击持续时间
#define ADXL345_LATENT          0x22    // 敲击延迟
#define ADXL345_WINDOW          0x23    // 敲击窗口
#define ADXL345_THRESH_ACT      0x24    // 活动阈值
#define ADXL345_THRESH_INACT    0x25    // 静止阈值
#define ADXL345_TIME_INACT      0x26    // 静止时间
#define ADXL345_ACT_INACT_CTL   0x27    // 活动/静止控制
#define ADXL345_THRESH_FF       0x28    // 自由落体阈值
#define ADXL345_TIME_FF         0x29    // 自由落体时间
#define ADXL345_TAP_AXES        0x2A    // 敲击轴控制
#define ADXL345_ACT_TAP_STATUS  0x2B    // 活动/敲击状态
#define ADXL345_BW_RATE         0x2C    // 带宽和输出数据速率
#define ADXL345_POWER_CTL       0x2D    // 电源控制
#define ADXL345_INT_ENABLE      0x2E    // 中断使能
#define ADXL345_INT_MAP         0x2F    // 中断映射
#define ADXL345_INT_SOURCE      0x30    // 中断源
#define ADXL345_DATA_FORMAT     0x31    // 数据格式
#define ADXL345_DATAX0          0x32    // X轴数据0
#define ADXL345_DATAX1          0x33    // X轴数据1
#define ADXL345_DATAY0          0x34    // Y轴数据0
#define ADXL345_DATAY1          0x35    // Y轴数据1
#define ADXL345_DATAZ0          0x36    // Z轴数据0
#define ADXL345_DATAZ1          0x37    // Z轴数据1
#define ADXL345_FIFO_CTL        0x38    // FIFO控制
#define ADXL345_FIFO_STATUS     0x39    // FIFO状态

// 器件ID值
#define ADXL345_DEVICE_ID       0xE5

// 电源控制位定义
#define ADXL345_PCTL_MEASURE    0x08    // 测量模式
#define ADXL345_PCTL_SLEEP      0x04    // 睡眠模式
#define ADXL345_PCTL_WAKEUP     0x00    // 待机模式

// 数据格式位定义
#define ADXL345_RANGE_2G        0x00    // ±2g量程
#define ADXL345_RANGE_4G        0x01    // ±4g量程
#define ADXL345_RANGE_8G        0x02    // ±8g量程
#define ADXL345_RANGE_16G       0x03    // ±16g量程
#define ADXL345_FULL_RES        0x08    // 全分辨率模式
#define ADXL345_JUSTIFY         0x04    // 左对齐模式

// 中断类型定义
#define ADXL345_INT_DATA_READY  0x80    // 数据就绪中断
#define ADXL345_INT_SINGLE_TAP  0x40    // 单击中断
#define ADXL345_INT_DOUBLE_TAP  0x20    // 双击中断
#define ADXL345_INT_ACTIVITY    0x10    // 活动中断
#define ADXL345_INT_INACTIVITY  0x08    // 静止中断
#define ADXL345_INT_FREE_FALL   0x04    // 自由落体中断
#define ADXL345_INT_WATERMARK   0x02    // 水位标记中断
#define ADXL345_INT_OVERRUN     0x01    // 溢出中断

// 滤波参数
#define ADXL345_SAMPLE_RATE     200.0f  // 采样频率 200Hz
#define ADXL345_LP_CUTOFF       5.0f    // 低通截止频率 5Hz
#define ADXL345_HP_CUTOFF       0.1f    // 高通截止频率 0.1Hz
#define ADXL345_MA_SIZE         4       // 滑动平均窗口大小
#define ADXL345_CALIB_SAMPLES   200     // 校准样本数量

// 数据结构定义
typedef struct {
    float x;          // X轴加速度 (g)
    float y;          // Y轴加速度 (g)
    float z;          // Z轴加速度 (g)
    uint32_t timestamp; // 时间戳
} ADXL345_DATA;

typedef struct {
    bool initialized;        // 初始化状态
    uint8_t comm_errors;     // 通信错误计数
    bool data_ready;         // 数据就绪标志
    uint8_t range;           // 量程设置
    bool self_test_passed;   // 自检状态
    float temperature;       // 温度值（如果支持）
    uint32_t last_update;    // 最后更新时间
} ADXL345_STATUS;

typedef struct {
    float x_bias;     // X轴偏置
    float y_bias;     // Y轴偏置
    float z_bias;     // Z轴偏置
    bool calibrated;  // 校准状态
} ADXL345_CALIBRATION;

// 中断回调函数类型
typedef void (*ADXL345_InterruptCallback)(uint8_t int_source);

// 函数声明 - 基本功能
int8_t ADXL345_Init(void);
int8_t ADXL345_Deinit(void);
int8_t ADXL345_Reset(void);
int8_t ADXL345_ReadData(void);
int8_t ADXL345_ReadRawData(int16_t *x, int16_t *y, int16_t *z);

// 函数声明 - 配置功能
int8_t ADXL345_SetRange(uint8_t range);
int8_t ADXL345_SetDataRate(uint8_t rate);
int8_t ADXL345_SetPowerMode(uint8_t mode);
int8_t ADXL345_EnableInterrupt(uint8_t int_type, bool enable);
int8_t ADXL345_MapInterrupt(uint8_t int_type, uint8_t pin);

// 函数声明 - 校准和自检
int8_t ADXL345_Calibrate(void);
int8_t ADXL345_SelfTest(void);
int8_t ADXL345_SetOffset(float x_offset, float y_offset, float z_offset);

// 函数声明 - 状态查询
ADXL345_STATUS ADXL345_GetStatus(void);
ADXL345_CALIBRATION ADXL345_GetCalibration(void);
bool ADXL345_IsDataReady(void);
uint8_t ADXL345_GetInterruptSource(void);

// 函数声明 - 中断处理
void ADXL345_SetInterruptCallback(ADXL345_InterruptCallback callback);
void ADXL345_IRQHandler(void);

// 函数声明 - 电源管理
int8_t ADXL345_Sleep(void);
int8_t ADXL345_Wakeup(void);
int8_t ADXL345_SetLowPowerMode(bool enable);

// 函数声明 - 高级功能
int8_t ADXL345_ConfigTapDetection(uint8_t threshold, uint8_t duration);
int8_t ADXL345_ConfigActivityDetection(uint8_t threshold, uint8_t time);
int8_t ADXL345_ConfigFreeFallDetection(uint8_t threshold, uint8_t time);

// 全局变量声明
extern ADXL345_DATA adxl345_data;
extern ADXL345_STATUS adxl345_status;
extern ADXL345_CALIBRATION adxl345_calibration;

#endif // __ADXL345_H
