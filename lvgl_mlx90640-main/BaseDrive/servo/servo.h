#ifndef __SYMC_SERVO_H__
#define __SYMC_SERVO_H__

#include "stm32f4xx.h"

void ServoInit(void);
void SetServoAngle(uint8_t id, uint8_t angle);

#endif