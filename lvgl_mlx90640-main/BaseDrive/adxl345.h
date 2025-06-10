#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include <stdbool.h>

// 状态码定义
#define ADXL345_SUCCESS        0
#define ADXL345_ERROR         -1
#define ADXL345_COMM_ERROR    -2
#define ADXL345_NOT_FOUND     -3
#define ADXL345_INVALID       -4

// 设备常量
#define ADXL345_DEVICE_ID     0xE5

// 量程设置
#define ADXL345_RANGE_2G      0
#define ADXL345_RANGE_4G      1
#define ADXL345_RANGE_8G      2
#define ADXL345_RANGE_16G     3

// 电源模式
#define ADXL345_POWERCTL_STANDBY 0x00
#define ADXL345_POWERCTL_MEASURE 0x08
#define ADXL345_POWERCTL_SLEEP   0x04

// 数据率
#define ADXL345_DATARATE_100HZ  0x0A
#define ADXL345_DATARATE_200HZ  0x0B
#define ADXL345_DATARATE_400HZ  0x0C
#define ADXL345_DATARATE_800HZ  0x0D
#define ADXL345_DATARATE_1600HZ 0x0E
#define ADXL345_DATARATE_3200HZ 0x0F

// 数据结构体
typedef struct {
    float x;
    float y;
    float z;
} ADXL345_DATA;

// 状态结构体
typedef struct {
    bool initialized;
    bool data_ready;
    uint8_t range;
    uint32_t comm_errors;
    uint32_t last_update;
} ADXL345_STATUS;

// 校准数据结构体
typedef struct {
    float x_offset;
    float y_offset;
    float z_offset;
    float x_scale;
    float y_scale;
    float z_scale;
} ADXL345_CALIBRATION;

// 中断回调函数类型
typedef void (*ADXL345_InterruptCallback)(uint8_t int_source);

// 函数声明
int8_t ADXL345_Init(void);
int8_t ADXL345_ReadRawData(int16_t *x, int16_t *y, int16_t *z);
int8_t ADXL345_ReadData(void);
int8_t ADXL345_SetRange(uint8_t range);
int8_t ADXL345_SetDataRate(uint8_t rate);
int8_t ADXL345_SetPowerMode(uint8_t mode);
int8_t ADXL345_Reset(void);
int8_t ADXL345_Deinit(void);
int8_t ADXL345_EnableInterrupt(uint8_t int_type, bool enable);
int8_t ADXL345_MapInterrupt(uint8_t int_type, uint8_t pin);
int8_t ADXL345_Calibrate(void);
int8_t ADXL345_SelfTest(void);
int8_t ADXL345_SetOffset(float x_offset, float y_offset, float z_offset);
int8_t ADXL345_Sleep(void);
int8_t ADXL345_Wakeup(void);
int8_t ADXL345_SetLowPowerMode(bool enable);
int8_t ADXL345_ConfigTapDetection(uint8_t threshold, uint8_t duration);
int8_t ADXL345_ConfigActivityDetection(uint8_t threshold, uint8_t time);
int8_t ADXL345_ConfigFreeFallDetection(uint8_t threshold, uint8_t time);

// 获取函数
ADXL345_STATUS ADXL345_GetStatus(void);
bool ADXL345_IsDataReady(void);
ADXL345_CALIBRATION ADXL345_GetCalibration(void);
uint8_t ADXL345_GetInterruptSource(void);
void ADXL345_SetInterruptCallback(ADXL345_InterruptCallback callback);
void ADXL345_IRQHandler(void);

#endif /* ADXL345_H */

