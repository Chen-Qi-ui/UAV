#ifndef __CALIBARATION_H
#define __CALIBARATION_H
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "arm_math.h"
#include "math.h"
void vSensor_ReceiveTask(void *pvParameters);
uint64_t get_time_usec_precise(void);
typedef struct{
	double gyro[3];
	double acc[3];
	double mag[3];
}IMU_Data;
#endif