#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "../SystemClock/SystemClock.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "MySPI.h"
#include "W25Q32_Ins.h"
#include "BMI088.h"
#include "AK8975.h"
#include "../main/Task_Create.h"
#include "../main/Queues.h"
uint16_t acc_x;
uint16_t acc_y;
uint16_t acc_z;
BMI088_Info bmi_info;
uint8_t gyro_id;
void BMI088_Acc_Init(void)
{
	MySPI_AK_Stop();
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 等待1ms
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // 1. 使能加速度计
    MySPI_Acc_Start();
    MySPI_SwapByte(0x7D);      // ACC_PWR_CTRL地址（写操作）
    MySPI_SwapByte(0x04);      // 数据：0x04（使能加速度计）
    MySPI_Acc_Stop();
    
   
    vTaskDelay(pdMS_TO_TICKS(50));
    
    
    MySPI_Acc_Start();
    MySPI_SwapByte(0x41);      // ACC_RANGE地址（写操作）
    MySPI_SwapByte(0x01);      // 数据：0x01（±6g）
    MySPI_Acc_Stop();
    
    
    MySPI_Acc_Start();
    MySPI_SwapByte(0x40);      // ACC_CONF地址
    MySPI_SwapByte(0xA8);      // 数据：0xA8（100Hz，正常模式）
    MySPI_Acc_Stop();
}

void BMI088_Gyro_Init(void)
{
	MySPI_AK_Stop();
	
    MySPI_Gyro_Start();
	MySPI_SwapByte(0x0F);
	MySPI_SwapByte(0x01);  //0x01(/32.768)
	MySPI_Gyro_Stop();
	 
    vTaskDelay(pdMS_TO_TICKS(30));
	MySPI_Gyro_Start();
    MySPI_SwapByte(0x10);      // GYRO_BANDWIDTH地址
    MySPI_SwapByte(0x03);      
    MySPI_Gyro_Stop();
	
}



void BMI088_Acc_ReadID(uint8_t *acc_id)  
{
    MySPI_Acc_Start();
    MySPI_SwapByte(0x80);      // 发送读命令
    MySPI_SwapByte(0x00);      // 丢弃哑元字节（加速度计特性）
    *acc_id = MySPI_SwapByte(0x00);  // 读取实际ID
    MySPI_Acc_Stop();
}


void BMI088_Gyro_ReadID(uint8_t *gyro_id)  
{
    MySPI_Gyro_Start();
    MySPI_SwapByte(0x80);      // 发送读命令         
    *gyro_id = MySPI_SwapByte(0x00);  // 读取实际ID
    MySPI_Gyro_Stop();
}
uint8_t buffer[8];
uint8_t buffer2[8];
void BMI088_Acc_Data(uint16_t *acc_x,uint16_t *acc_y,uint16_t *acc_z) 
{
    MySPI_Acc_Start();
    MySPI_SwapByte(0x12 | 0x80);      // 发送读命令
	
	for(int i =0;i<7;i++)
	{
		buffer[i]=MySPI_SwapByte(0x00);
	}
	*acc_x = buffer[2]<<8 | buffer[1];
	*acc_y = buffer[4]<<8 | buffer[3];
	*acc_z = buffer[6]<<8 | buffer[5];
    MySPI_Acc_Stop();
}

void BMI088_Gyro_Data(uint16_t *gyro_x,uint16_t *gyro_y,uint16_t *gyro_z) 
{
    MySPI_Gyro_Start();
    MySPI_SwapByte(0x02 | 0x80);      // 发送读命令,从0X02开始，能够自增
	
	for(int i =0;i<7;i++)
	{
		buffer2[i]=MySPI_SwapByte(0x00);
	}
	*gyro_x = buffer2[1]<<8 | buffer2[0];
	*gyro_y = buffer2[3]<<8 | buffer2[2];
	*gyro_z = buffer2[5]<<8 | buffer2[4];
    MySPI_Gyro_Stop();
}

double X;
void vBMI088_Task(void *pvParameters)
{
   	BMI088_Gyro_Init();
	BMI088_Acc_Init();
	
	vTaskDelay(pdMS_TO_TICKS(5)); 
    for(;;)
    {
		BMI088_Gyro_ReadID(&gyro_id);
        uint16_t Acc_x, Acc_y, Acc_z;
		uint16_t Gyro_x, Gyro_y, Gyro_z;
        BMI088_Acc_Data(&Acc_x, &Acc_y, &Acc_z);
		BMI088_Gyro_Data(&Gyro_x, &Gyro_y, &Gyro_z);
		TickType_t previousTime = xTaskGetTickCount();
        
        // 转为有符号！
        int16_t signedAcc_x = (int16_t)Acc_x;
        int16_t signedAcc_y = (int16_t)Acc_y;
        int16_t signedAcc_z = (int16_t)Acc_z;
        int16_t signedGyro_x = (int16_t)Gyro_x;
        int16_t signedGyro_y = (int16_t)Gyro_y;
        int16_t signedGyro_z = (int16_t)Gyro_z;

        bmi_info.acc_x_g = signedAcc_x / 5460.0f;
        bmi_info.acc_y_g = signedAcc_y / 5460.0f;
        bmi_info.acc_z_g = signedAcc_z / 5460.0f;
		bmi_info.gyro_x_w = signedGyro_x / 32.768f;
		bmi_info.gyro_y_w = signedGyro_y / 32.768f;
		bmi_info.gyro_z_w = signedGyro_z / 32.768f;
		X = bmi_info.gyro_z_w;
		xQueueOverwrite(xBMI088Queue, &bmi_info);

        vTaskDelayUntil(&previousTime,pdMS_TO_TICKS(10));
    }
}
