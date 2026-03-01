#ifndef __BMI088_H
#define __BMI088_H
#define Gyro_CS GPIO_Pin_10
#define ACC_CS  GPIO_Pin_0
#define MISO    GPIO_Pin_14
#define MOSI    GPIO_Pin_15
#include "stm32f4xx.h"
#include "FreeRTOS.h"
void BMI088_Acc_Init(void);
void BMI088_Acc_ReadID(uint8_t *acc_id);
void BMI088_Acc_Data(uint16_t *acc_x,uint16_t *acc_y,uint16_t *acc_z) ;
void vBMI088_Task(void *pvParameters);
extern uint16_t acc_x;
extern uint16_t acc_y;
extern uint16_t acc_z;
extern float acc_x_g ;
extern float acc_y_g;
extern float acc_z_g ;

typedef struct{
	float acc_x_g;
    float acc_y_g;		
	float acc_z_g;
	float gyro_x_w;
	float gyro_y_w;
	float gyro_z_w;
}BMI088_Info;
#endif
